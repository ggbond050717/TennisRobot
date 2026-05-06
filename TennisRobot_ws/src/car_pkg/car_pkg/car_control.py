#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import fcntl
import os

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fd = sys.stdin.fileno()

        self.speed = 0.20 # 线速度
        self.angular_speed = 1.0 # 角速度
        self.twist = Twist()
        
        # 保存原始终端设置
        self.old_settings = termios.tcgetattr(self.fd)
        
        # 设置非阻塞读取模式
        tty.setraw(sys.stdin.fileno())
        fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        self.get_logger().info("键盘控制启动（WASD控制方向，空格键停止，Q退出）")

    def get_key(self):
        try:
            return sys.stdin.read(1)
        except (IOError, KeyboardInterrupt):
            return None

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key is None:
                    continue  # 没有按键按下时跳过
                
                # 按键处理逻辑
                if key == 'w':
                    self.twist.linear.x = self.speed
                    self.twist.angular.z = 0.0
                elif key == 's':
                    self.twist.linear.x = -self.speed
                    self.twist.angular.z = 0.0
                elif key == 'a':
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.angular_speed
                elif key == 'd':
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -self.angular_speed
                elif key == ' ':
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                elif key == 'q':
                    break
                else:
                    # 无效按键时清零
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    continue  # 跳过无效按键的发布

                # 仅在有效按键按下时发布一次
                self.publisher.publish(self.twist)
                self.get_logger().info(f"发送指令：线速度={self.twist.linear.x}, 角速度={self.twist.angular.z}\r\n")

        except Exception as e:
            self.get_logger().error(f"错误: {str(e)}")
        finally:
            # 恢复终端设置
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            
            # 发送最终停止指令
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            self.get_logger().info("已停止并恢复终端设置")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()