#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image
import psutil

class OptimizedCameraNode(Node):
    def __init__(self):
        super().__init__('optimized_camera_node')
        
        # 零拷贝QoS配置
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', qos_profile)
        
        # 修改后的JPEG硬件加速管道
        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM),width=1280,height=720,framerate=24/1,format=NV12 ! "
            "nvvidconv flip-method=0 ! "  # 保持图像翻转
            "video/x-raw,format=I420 ! "  # 转换格式以适配JPEG编码器
            "nvjpegenc ! "  # 使用NVIDIA硬件JPEG编码器
            "image/jpeg,framerate=24/1 ! "
            "appsink name=sink emit-signals=True sync=False"
        )
        
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect("new-sample", self.on_new_frame)
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # 内存管理
        self.create_timer(2.0, self.gc_callback)

    def on_new_frame(self, sink):
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            success, map_info = buf.map(Gst.MapFlags.READ)
            
            if success:
                try:
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera"
                    msg.height = 720
                    msg.width = 1280
                    msg.encoding = "jpeg"  # 修改编码类型为JPEG
                    msg.is_bigendian = 0
                    msg.step = 0  # 压缩格式不需要步长
                    msg.data = memoryview(map_info.data).tobytes() if map_info else b''
                    
                    self.publisher_.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"发布失败: {str(e)}")
                finally:
                    buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def gc_callback(self):
        import gc
        gc.collect()

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedCameraNode()
    try:
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()