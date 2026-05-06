import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from msg_interfaces.msg import WheelSpeed
from geometry_msgs.msg import Twist
import serial
import ctypes
import time
import threading
import queue
from memory_profiler import profile

class CarSerial(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 串口配置
        self._init_serial()
        
        # ROS2通信配置
        self._init_ros_communication()
        
        # 缓冲区与协议配置
        self.PACKET_HEADER = b'\xAA\x55'
        self.PACKET_SIZE = 12
        self.buffer = bytearray()
        
        # 性能监控
        self.rx_counter = 0
        self.tx_counter = 0
        self.last_stat_time = time.time()

        self.left_sum = 0
        self.right_sum = 0
        
        # 异步写入队列
        self.write_queue = queue.Queue(maxsize=50)
        self._start_worker_threads()

    def _init_serial(self):
        """初始化串口参数"""
        try:
            self.serial_port = serial.Serial(
                port='/dev/car_serial',
                baudrate=115200,
                timeout=0.01,
                write_timeout=0.1,
                rtscts=True  # 启用硬件流控制
            )
            #self.serial_port.set_buffer_size(rx_size=8192, tx_size=8192)
            self.serial_port.reset_input_buffer()
            #self.serial_port.write(b'\x01')  # 初始化命令
        except serial.SerialException as e:
            self.get_logger().error(f"串口初始化失败: {str(e)}")
            raise

    def _init_ros_communication(self):
        """初始化ROS2通信组件"""
        self.publisher = self.create_publisher(
            WheelSpeed, 
            '/wheelspeed', 
            qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.control_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )
        # 性能监控定时器
        # self.create_timer(1.0, self._performance_monitor)

    def _start_worker_threads(self):
        """启动工作线程"""
        self.writer_thread = threading.Thread(
            target=self._write_worker, 
            daemon=True
        )
        self.writer_thread.start()
        
        self.reader_thread = threading.Thread(
            target=self._read_worker,
            daemon=True
        )
        self.reader_thread.start()

    def control_callback(self, msg):
        """控制指令回调（非阻塞队列化处理）"""
        try:
            # 小车参数
            wheel_diameter = 0.085  # 轮子直径(m)
            wheel_base = 0.275      # 轮距(m)
            encoder_resolution = 660  # 编码器每转脉冲数
            
            # 计算轮子周长
            wheel_circumference = 3.1415926 * wheel_diameter
            
            # 从Twist消息计算左右轮速度(m/s)
            linear = msg.linear.x
            angular = msg.angular.z
            
            # 差速驱动运动学模型
            left_speed = linear - (angular * wheel_base) / 2.0
            right_speed = linear + (angular * wheel_base) / 2.0
            
            # 将速度(m/s)转换为编码器计数/秒
            left_encoder = int((left_speed / wheel_circumference / 100.0) * encoder_resolution)
            right_encoder = int((right_speed / wheel_circumference / 100.0) * encoder_resolution)
            
            # 打印调试信息
            self.get_logger().info(
                f"转换结果 | 线速度: {linear:.2f}m/s | 角速度: {angular:.2f}rad/s\n"
                f"左轮: {left_encoder} counts/s | 右轮: {right_encoder} counts/s",
                throttle_duration_sec=1
            )
            
            # 构建控制数据包
            packet = self._build_control_packet(left_encoder, right_encoder)
            self.write_queue.put_nowait(packet)
            
        except queue.Full:
            self.get_logger().warn("发送队列已满，丢弃指令", throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().error(f"指令处理异常: {str(e)}")

    def _build_control_packet(self, left, right):
        """构造控制数据包（预计算优化版）"""
        packet = bytearray(12)
        packet[0:2] = self.PACKET_HEADER
        packet[2] = 9  # 固定长度
        
        # 小端序编码
        packet[3:7] = left.to_bytes(4, 'big', signed=True)
        packet[7:11] = right.to_bytes(4, 'big', signed=True)
        
        # 计算校验和
        checksum = 0
        for b in packet[2:11]:
            checksum ^= b
        packet[11] = checksum
        
        return bytes(packet)

    def _write_worker(self):
        """异步写入工作线程"""
        while rclpy.ok():
            try:
                packet = self.write_queue.get(timeout=0.1)
                written = self.serial_port.write(packet)
                # self.tx_counter += written
            except queue.Empty:
                continue
            except serial.SerialTimeoutException:
                self.get_logger().warn("串口写入超时", throttle_duration_sec=1)
            except Exception as e:
                self.get_logger().error(f"写入线程异常: {str(e)}")
                time.sleep(0.5)

    def _read_worker(self):
        """专用读取线程"""
        while rclpy.ok():
            try:
                # 动态调整读取块大小
                in_waiting = self.serial_port.in_waiting
                if in_waiting == 0:
                    time.sleep(0.001)
                    continue
                
                read_size = min(512, in_waiting)
                data = self.serial_port.read(read_size)
                # self.rx_counter += len(data)
                
                with threading.Lock():
                    self.buffer.extend(data)
                    self._parse_buffer()

            except serial.SerialException as e:
                self.get_logger().error(f"串口读取错误: {str(e)}")
                time.sleep(1)
            except Exception as e:
                self.get_logger().error(f"读取线程异常: {str(e)}")
                time.sleep(0.1)
    
    def _parse_buffer(self):
        """高效解析缓冲区"""
        mv = memoryview(self.buffer)
        processed = 0
        packet_size = self.PACKET_SIZE

        while len(mv) >= packet_size:
            # 快速查找帧头
            header_pos = mv.obj.find(self.PACKET_HEADER, processed)
            if header_pos == -1:
                processed = len(mv)
                break

            # 跳转到帧头位置
            if header_pos > processed:
                processed = header_pos

            # 检查剩余长度
            if len(mv) - processed < packet_size:
                break

            # 提取数据包
            packet = mv[processed:processed+packet_size]
            
            # 校验和验证
            if not self._validate_packet(packet):
                processed += 1
                continue

            # 解析有效数据
            self._process_packet(packet)
            processed += packet_size

        # 清理已处理数据
        self.buffer = self.buffer[processed:]

    def _validate_packet(self, packet):
        """数据包校验"""
        checksum = 0
        for b in packet[2:-1]:
            checksum ^= b
        return checksum == packet[-1]

    def _process_packet(self, packet):
        """数据包处理（带类型转换优化）"""
        try:
            left = int.from_bytes(packet[3:7], 'little', signed=True)
            right = int.from_bytes(packet[7:11], 'little', signed=True)
            
            # 发布消息
            msg = WheelSpeed()
            msg.left = ctypes.c_int16(left).value
            msg.right = ctypes.c_int16(right).value
            self.left_sum += msg.left
            self.right_sum += msg.right
            #self.get_logger().info(
            #    f"左轮: {self.left_sum} counts/s | 右轮: {self.right_sum} counts/s",
            #    throttle_duration_sec=1
            #)
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"数据解析错误: {str(e)}")

    def _performance_monitor(self):
        """性能监控与统计"""
        now = time.time()
        interval = now - self.last_stat_time
        rx_rate = self.rx_counter / interval / 1024
        tx_rate = self.tx_counter / interval / 1024
        
        self.get_logger().info(
            f"吞吐量 | 接收: {rx_rate:.1f}KB/s | 发送: {tx_rate:.1f}KB/s\n"
            f"队列状态 | 写入队列: {self.write_queue.qsize()}/50 | 接收缓冲: {len(self.buffer)}字节",
            throttle_duration_sec=2
        )
        
        # 重置计数器
        self.rx_counter = 0
        self.tx_counter = 0
        self.last_stat_time = now

    def stop(self):
        """安全关闭"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        self.destroy_node()

def main():
    rclpy.init()
    node = CarSerial("car_serial")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()