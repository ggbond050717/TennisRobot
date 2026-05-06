import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from math import sin, cos, pi, fabs
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from msg_interfaces.msg import WheelSpeed

class DiffTf(Node):
    def __init__(self):
        super().__init__('diff_tf')
        # 参数声明
        self.declare_parameters(namespace='',
            parameters=[
                ('rate', 10.0),           # 与编码器更新率同步
                ('ticks_meter', 2472.8362),      # 每米脉冲数,1/0.085*3.14/660
                ('base_width', 0.185),    # 轮距
                ('base_frame_id', 'base_link'),
                ('odom_frame_id', 'odom'),
                ('filter_tau', 0.2),     # 速度滤波系数
                ('max_encoder_delta', 1000)  # 单次最大合理脉冲变化量
            ])
        
        # 初始化变量
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.lwheel = 0.0
        self.rwheel = 0.0
        self.last_update = self.get_clock().now()
        self.last_encoder_time = self.get_clock().now()
        self.is_first_update = True
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 预计算参数
        self.filter_tau = self.get_parameter('filter_tau').value
        self.inv_ticks_meter = 1.0 / self.get_parameter('ticks_meter').value
        self.inv_base_width = 1.0 / self.get_parameter('base_width').value
        self.max_delta = self.get_parameter('max_encoder_delta').value

        # 编码器订阅
        self.wheel_sub = self.create_subscription(
                WheelSpeed, 
                '/wheelspeed', 
                self.wheel_callback,
                qos_profile=QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT  # 与发布者一致
                )
            ) 
        # 里程计发布
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # 定时器与编码器更新同步（100ms周期）
        self.create_timer(0.1, self.update)  # 固定10ms周期

    def wheel_callback(self, msg):
        """带符号的脉冲值（正转/反转）"""
        now = self.get_clock().now()

        # 数据有效性检查
        if (abs(msg.left) > self.max_delta or 
            abs(msg.right) > self.max_delta):
            self.get_logger().warn(
                f"异常脉冲数据: left={msg.left}, right={msg.right}",
                throttle_duration_sec=1.0
            )
            return
        
        self.lwheel = msg.left   # 负值表示左轮反转
        self.rwheel = msg.right  # 负值表示右轮反转
        self.last_encoder_time = now

    def update(self):
        now = self.get_clock().now()

        # 初始化检查
        if self.is_first_update:
            self.last_update = now
            self.is_first_update = False
            return
        
        # 检查数据新鲜度（超过300ms丢弃）
        if (now - self.last_encoder_time).nanoseconds > 300e6:
            self.get_logger().warn(
                "编码器数据过期，停止更新",
                throttle_duration_sec=1.0
            )
            self.filtered_dx = 0.0
            self.filtered_dr = 0.0
            return
        
        # 计算时间间隔（限制在0.05-0.15秒之间）
        dt = (now - self.last_update).nanoseconds / 1e9
        dt = min(max(dt, 0.05), 0.15)
        try:
            # 计算位移和角度变化
            left_dist = self.lwheel * self.inv_ticks_meter
            right_dist = self.rwheel * self.inv_ticks_meter

            # 差分运动学模型[7]
            d = (left_dist + right_dist) * 0.5     # 线位移（考虑正负）
            th = (right_dist - left_dist) / self.inv_base_width  # 角位移

            # 改进的位姿积分
            if abs(d) > 1e-6:
                half_th = th * 0.5
                avg_th = self.th + half_th
                self.x += d * cos(avg_th)
                self.y += d * sin(avg_th)
        
            if fabs(th) > 1e-6:  # 角度阈值
                self.th += th
                self.th = (self.th + pi) % (2*pi) - pi # 角度归一化

            # 速度计算与滤波
            current_dx = d / dt if dt > 1e-6 else 0.0
            current_dr = th / dt if dt > 1e-6 else 0.0
            alpha = dt / (self.filter_tau + dt)
            self.filtered_dx = alpha * current_dx + (1-alpha)*self.filtered_dx
            self.filtered_dr = alpha * current_dr + (1-alpha)*self.filtered_dr
            

            # 四元数转换（使用标准库）
            quaternion = Quaternion()
            q = quaternion_from_euler(0, 0, self.th)
            quaternion.x, quaternion.y, quaternion.z, quaternion.w = q

            # 发布TF
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.get_parameter('odom_frame_id').value
            t.child_frame_id = self.get_parameter('base_frame_id').value
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = quaternion
            self.tf_broadcaster.sendTransform(t)
        
            # 发布Odometry（带协方差）
            odom = Odometry()

            odom.header = t.header
            
            #odom.header.stamp = now.to_msg()
            #odom.header.frame_id = self.get_parameter('odom_frame_id').value

            odom.child_frame_id = self.get_parameter('base_frame_id').value

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = self.filtered_dx
            odom.twist.twist.angular.z = self.filtered_dr
            
            # 协方差矩阵配置[7]
            odom.pose.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.03, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.03, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.03
            ]

            odom.twist.covariance = [
                0.01, 0.0,  0.0,  0.0,  0.0,  0.0,  # vx
                0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  # vy
                0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  # vz
                0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  # vroll
                0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  # vpitch
                0.0,  0.0,  0.0,  0.0,  0.0,  0.03  # vyaw
            ]
        
            self.odom_pub.publish(odom)
        
        except Exception as e:
            self.get_logger().error(f"里程计计算异常: {str(e)}")
        finally:
            self.last_update = now

def main(args=None):
    rclpy.init(args=args)
    node = DiffTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()