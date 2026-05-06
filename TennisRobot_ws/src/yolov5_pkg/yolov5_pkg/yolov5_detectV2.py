#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msg_interfaces.msg import DetectResult, ImageResult

import cv2
import time
import torch
import numpy as np
from pathlib import Path


from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.plots import Annotator, colors

class YOLOv5Detect(Node):
    def __init__(self):
        super().__init__('yolov5_detect')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', str(Path.home()/ 'TennisRobot_ws/src/yolov5_pkg/tennis-s-C2f-SE.pt')),
                ('img_size', 640),
                ('conf_thres', 0.5),
                ('iou_thres', 0.45),
                ('classes', None),
                ('device', 'cuda:0' if torch.cuda.is_available() else 'cpu'),
                ('class_name', 'tennis'),
                ('line_thickness', 2),
                ('camera_width', 1280),
                ('camera_height', 720),
                ('camera_fps', 10),
                ('jpeg_quality', 85),
                ('jpeg_progressive', 0),
                ('jpeg_optimize', 1),
                ('font_size', 0.5)
            ]
        )
        
        # 初始化YOLOv5模型
        self._init_yolov5()

        # 创建发布者
        self.detection_pub = self.create_publisher(DetectResult, '/detection/results', 10)
        self.visualization_pub = self.create_publisher(ImageResult, '/detection/visualization', 10)

        # 初始化摄像头
        self.cap = None
        self._init_camera()

        # 创建定时器（匹配摄像头帧率）
        self.timer = self.create_timer(
            1/self.get_parameter('camera_fps').value, 
            self.process_frame
        )

        self.get_logger().info("YOLOv5 Camera Detector Initialized")


    def _init_camera(self):
        """改进的CSI摄像头初始化"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(1)

        gst_config = (
            'nvarguscamerasrc sensor-id=0 ! '
            'video/x-raw(memory:NVMM), '
            f'width={self.get_parameter("camera_width").value}, '
            f'height={self.get_parameter("camera_height").value}, '
            f'framerate={self.get_parameter("camera_fps").value}/1 ! '
            'nvvidconv flip-method=2 ! '
            'video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=true sync=false max-buffers=1'
        )

        for _ in range(3):  # 重试机制
            self.cap = cv2.VideoCapture(gst_config, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self.get_logger().info(f"摄像头初始化成功: {gst_config}")
                for _ in range(10):
                    self.cap.read()
                time.sleep(0.3)
                return
            else:
                self.get_logger().warning("摄像头初始化失败，5秒后重试...")
                time.sleep(5)

        self.get_logger().error("无法初始化摄像头")
        raise RuntimeError("Camera initialization failed")


    def _init_yolov5(self):
        """加载YOLOv5模型"""
        model_path = self.get_parameter('model_path').value
        self.img_size = self.get_parameter('img_size').value
        self.device = torch.device(
            self.get_parameter('device').value if torch.cuda.is_available() else 'cpu')
        self.half = self.device.type != 'cpu'

         # 加载模型
        self.model = DetectMultiBackend(model_path, device=self.device)
        self.stride = self.model.stride
        self.img_size = check_img_size(self.img_size, s=self.stride)
        self.class_id = 0
        self.class_name = self.get_parameter('class_name').value

        # 转换为半精度
        if self.half:
            self.model.half()

        # 预热模型
        self.model(torch.zeros(1, 3, self.img_size, self.img_size).to(
            self.device).type_as(next(self.model.parameters())))
        
        # 获取类别名称
        self.names = self.model.module.names if hasattr(
            self.model, 'module') else self.model.names
        
    def process_frame(self):
        """处理每一帧图像"""
        if self.cap is None:
            return
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("摄像头读取失败，尝试重新初始化")
                self._init_camera()
                return
            
            # 记录原始帧时间戳
            timestamp = self.get_clock().now().to_msg()

             # 预处理和推理
            img = self._preprocess_image(frame)
            with torch.no_grad():
                pred = self.model(img)[0]

            # NMS处理
            pred = non_max_suppression(
                pred,
                self.get_parameter('conf_thres').value,
                self.get_parameter('iou_thres').value,
                classes=self.get_parameter('classes').value,
                agnostic=False)
            
            # 初始化标注器
            annotator = Annotator(
                frame,
                line_width=self.get_parameter('line_thickness').value,
                font_size=self.get_parameter('font_size').value,
                pil=False
            )
            
            
            detections = []
            # 处理检测结果
            for i, det in enumerate(pred):
                if det is not None and len(det):
                    det[:, :4] = scale_boxes(
                        img.shape[2:],   # 输入特征图尺寸
                        det[:, :4],      # 原始坐标
                        frame.shape      # 原始图像尺寸
                    ).round()

                    for *xyxy, conf, cls in reversed(det):
                        annotator.box_label(
                            xyxy, 
                            f"{self.class_name} {conf:.2f}",
                            color=(0, 255, 0))
                        result = DetectResult()
                        result.xyxy = [int(x) for x in xyxy]
                        result.conf = float(conf)
                        self.detection_pub.publish(result)
                        detections.append(result)

            # 发布检测结果,JPEG压缩和发布
            try:
                # 设置编码参数
                encode_param = [
                    int(cv2.IMWRITE_JPEG_QUALITY), 
                    self.get_parameter('jpeg_quality').value,
                    cv2.IMWRITE_JPEG_PROGRESSIVE, 
                    self.get_parameter('jpeg_progressive').value,
                    cv2.IMWRITE_JPEG_OPTIMIZE, 
                    self.get_parameter('jpeg_optimize').value
                ]
                # 执行压缩编码
                success, jpeg_data = cv2.imencode('.jpg', annotator.result(), encode_param)
                if not success:
                    self.get_logger().error("JPEG compression failed", throttle_duration_sec=5.0)
                    return
                
            except Exception as e:
                self.get_logger().error(f"Compression failed: {str(e)}")

            # 构建压缩图像消息
            results = ImageResult()
            results.image.header.stamp = timestamp
            results.image.header.frame_id = 'camera_frame'
            results.image.height = annotator.result().shape[0]
            results.image.width = annotator.result().shape[1]
            results.image.encoding = 'jpeg'
            results.image.is_bigendian = False
            results.image.step = 0
            results.image.data = jpeg_data.tobytes()
            results.results = detections

            # 发布压缩图像消息
            self.visualization_pub.publish(results)
            self.get_logger().info('Published results')
        
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {str(e)}")

    def _preprocess_image(self, img):
        """改进的图像预处理（保持宽高比+边缘填充）"""
        # 目标尺寸和颜色参数
        target_size = self.img_size
        color = (114, 114, 114)  # 灰度填充色
        
        # 原始图像尺寸
        h, w = img.shape[:2]
        scale = min(target_size / w, target_size / h)
        
        # 等比例缩放
        new_w, new_h = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(img, (new_w, new_h), 
                            interpolation=cv2.INTER_LINEAR)
        
        # 计算填充尺寸
        dw = target_size - new_w
        dh = target_size - new_h
        top, bottom = dh // 2, dh - (dh // 2)
        left, right = dw // 2, dw - (dw // 2)
        
        # 添加灰边填充
        padded = cv2.copyMakeBorder(
            resized, top, bottom, left, right,
            cv2.BORDER_CONSTANT, value=color
        )
        
        # 转换张量
        img = padded[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255.0
        
        # 保存填充参数用于坐标转换
        self.pad_params = (scale, (dw, dh), (left, top))
        
        return img.unsqueeze(0) if img.ndimension() == 3 else img

    def destroy_node(self):
        """改进的资源释放"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        torch.cuda.empty_cache()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detector = None
    try:
        detector = YOLOv5Detect()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        if detector:
            detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()