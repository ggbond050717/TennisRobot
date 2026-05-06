#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('virtual_display_node')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建图像显示窗口
        cv2.namedWindow("Camera Stream", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Stream", 1280, 720)
        
        # 配置订阅者（启用零拷贝）
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data  # 优化传输配置
        )
        self.subscription  # 防止未使用警告
        
        # 性能计数器
        self.frame_count = 0
        self.fps = 0
        self.timer = self.create_timer(1.0, self.update_fps)

    def image_callback(self, msg):
        try:
            # 将JPEG字节流转换为OpenCV格式
            if msg.encoding == 'jpeg':
                # 直接解码JPEG字节流
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # 错误处理
                if cv_image is None:
                    self.get_logger().error("JPEG解码失败")
                    return
            else:
                # 处理非压缩格式
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 显示性能指标
            self.frame_count += 1
            cv2.putText(cv_image, f"FPS: {self.fps}", (10,30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            
            # 显示图像
            cv2.imshow("Camera Stream", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")

    def update_fps(self):
        self.fps = self.frame_count
        self.frame_count = 0
        self.get_logger().info(f"当前帧率: {self.fps} FPS")

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    # 初始化ROS
    rclpy.init(args=args)
    
    try:
        # 创建节点
        display_node = ImageSubscriber()
        
        # 配置执行器（优化多线程）
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        executor.add_node(display_node)
        
        # 启动显示循环
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        if 'display_node' in locals():
            display_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()