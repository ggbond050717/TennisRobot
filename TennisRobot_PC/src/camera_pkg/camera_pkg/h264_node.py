#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import signal
import time

class H264DecoderNode(Node):
    def __init__(self):
        super().__init__('h264_decoder_node')
        
        # 初始化GStreamer管道
        Gst.init(None)
        
        # 创建简化后的GStreamer管道
        self.pipeline = Gst.parse_launch(
            "appsrc name=src ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "autovideosink sync=false"
        )
        
        # 配置appsrc
        self.appsrc = self.pipeline.get_by_name('src')
        self.appsrc.set_property('caps', Gst.Caps.from_string(
            "video/x-h264,stream-format=byte-stream,alignment=nal"
        ))
        self.appsrc.set_property('block', True)
        
        # 创建ROS2订阅者
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # FPS计数器
        self.frame_count = 0
        self.last_time = time.time()
        
        # 启动管道
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 创建定时器更新FPS
        self.timer = self.create_timer(1.0, self.update_fps)

    def image_callback(self, msg):
        try:
            # 推送数据到GStreamer
            data = bytes(msg.data)
            buffer = Gst.Buffer.new_wrapped(data)
            buffer.pts = msg.header.stamp.sec * Gst.SECOND + msg.header.stamp.nanosec
            buffer.duration = Gst.CLOCK_TIME_NONE
            self.appsrc.emit('push-buffer', buffer)
            
            # 更新帧计数器
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"处理帧错误: {str(e)}")

    def update_fps(self):
        # 计算实际帧率
        current_time = time.time()
        elapsed = current_time - self.last_time
        actual_fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        # 输出带颜色的日志信息
        fps_str = f"\033[92m当前帧率: {actual_fps:.1f} FPS\033[0m"  # 绿色文本
        self.get_logger().info(fps_str)
        
        # 重置计数器
        self.frame_count = 0
        self.last_time = current_time

    def signal_handler(self, sig, frame):
        self.get_logger().info("接收到中断信号，正在关闭...")
        self.cleanup()
        rclpy.shutdown()

    def cleanup(self):
        # 发送EOS信号
        self.appsrc.emit('end-of-stream')
        # 停止管道
        self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info("GStreamer管道已停止")

    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = H264DecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()