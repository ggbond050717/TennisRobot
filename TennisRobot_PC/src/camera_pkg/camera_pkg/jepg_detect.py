#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from msg_interfaces.msg import ImageResult


class JpegDetect(Node):
    def __init__(self):
        super().__init__('jpeg_detect')
        self.subscription = self.create_subscription(
            ImageResult,
            '/detection/visualization',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # 视觉参数动态声明
        self.declare_parameter('focal_length', 1400) 
        self.declare_parameter('tennis_diameter', 6.7)
        self.declare_parameter('frame_center_x', 640)
        self.declare_parameter('frame_center_y', 320)

        # 创建图像显示窗口
        cv2.namedWindow("Camera Stream", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Stream", 1280, 720)

        # 性能计数器
        self.frame_count = 0
        self.fps = 0
        self.timer = self.create_timer(1.0, self.update_fps)

    def image_callback(self, msg):
        try:
            # 将JPEG字节流转换为OpenCV格式
            if msg.image.encoding == 'jpeg':
                # 直接解码JPEG字节流
                np_arr = np.frombuffer(msg.image.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # 错误处理
                if cv_image is None:
                    self.get_logger().error("JPEG解码失败")
                    return
            else:
                # 处理非压缩格式
                cv_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")

             # 处理每个检测结果
            for detect_result in msg.results:
                x1, y1, x2, y2 = detect_result.xyxy
                conf = detect_result.conf
                # 获取动态参数
                focal = self.get_parameter('focal_length').value
                real_dia = self.get_parameter('tennis_diameter').value
                center_x = self.get_parameter('frame_center_x').value
                center_y = self.get_parameter('frame_center_y').value

                # 计算几何参数
                pixel_width = x2 - x1
                obj_center_x = (x1 + x2) // 2
                obj_center_y = (y1 + y2) // 2

                 # 物理量计算 
                distance = (real_dia * focal) / pixel_width
                h_angle = np.degrees(np.arctan(
                    (obj_center_x - center_x)/focal))
                v_angle = np.degrees(np.arctan(
                    (obj_center_y - center_y)/focal))
                
                # 信息叠加显示
                cv2.rectangle(cv_image, (x1,y1), (x2,y2), (0,255,0), 3)
                info_line1 = f"Distance: {distance:.1f}cm"
                info_line2 = f"H:{h_angle:.1f} V:{v_angle:.1f}"
                cv2.putText(cv_image, info_line1, (x1, y2+25), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.putText(cv_image, info_line2, (x1, y2+55), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            # 显示性能指标
            self.frame_count += 1
            #cv2.putText(cv_image, f"FPS: {self.fps}", (10,30),
            #        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            
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
        display_node = JpegDetect()
        
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