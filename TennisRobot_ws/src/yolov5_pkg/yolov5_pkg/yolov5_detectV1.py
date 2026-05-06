#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msg_interfaces.msg import DetectResult, ImageResult

import os
import cv2
import torch
import numpy as np
from pathlib import Path

from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    non_max_suppression,
    scale_boxes,
    xyxy2xywh,
    set_logging,
)
from utils.augmentations import letterbox
from utils.torch_utils import select_device
from utils.plots import Annotator, colors

class YOLOv5Detect(Node):
    def __init__(self):
        super().__init__('yolov5_detect')
        # 完整参数声明（新增iou_thres）
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', str(Path.home()/ 'TennisRobot_ws/src/yolov5_pkg/tennis-s-C2f-SE.pt')),
                ('img_size', 640),
                ('conf_thres', 0.5),
                ('iou_thres', 0.45),  # 新增必须参数
                ('device', 'cuda:0' if torch.cuda.is_available() else 'cpu'),  # 明确设备
                ('line_thickness', 2),
                ('max_det', 1000),
                ('camera_framerate', 30)  # 新增摄像头参数
            ]
        )

        # 初始化YOLOv5模型
        self._init_yolov5()

        # 创建发布者
        self.detection_pub = self.create_publisher(DetectResult, '/detection/results', 10)
        self.visualization_pub = self.create_publisher(ImageResult, '/detection/visualization', 10)

        # 初始化摄像头（改进配置）
        self.cap = None
        self._init_camera()

        # 创建定时器（匹配摄像头帧率）
        self.timer = self.create_timer(
            1/self.get_parameter('camera_framerate').value, 
            self.process_frame
        )
        self.get_logger().info("YOLOv5 Camera Detector Initialized")

    def _init_yolov5(self):
        """改进的模型初始化"""
        set_logging()
        self.device = select_device(self.get_parameter('device').value)
        model_path = self.get_parameter('model_path').value
        
        # 加载模型（增加异常处理）
        try:
            self.model = DetectMultiBackend(
                weights=model_path,
                device=self.device,
                dnn=False,
                fp16=False
            )
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            raise

        # 参数初始化
        self.stride = self.model.stride
        self.img_size = check_img_size(
            self.get_parameter('img_size').value, 
            s=self.stride
        )
        self.names = self.model.names
        self.pt = self.model.pt

    def _init_camera(self):
        """改进的CSI摄像头初始化"""
        gst_config = (
            'nvarguscamerasrc sensor-id=0 ! '
            'video/x-raw(memory:NVMM), '
            f'width=1280, height=720, '
            f'framerate={self.get_parameter("camera_framerate").value}/1 ! '
            'nvvidconv flip-method=0 ! '
            'video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=true sync=false max-buffers=1'
        )
        
        for _ in range(3):  # 重试机制
            self.cap = cv2.VideoCapture(gst_config, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self.get_logger().info(f"摄像头初始化成功: {gst_config}")
                return
            else:
                self.get_logger().warning("摄像头初始化失败，5秒后重试...")
                rclpy.sleep(5)
        
        self.get_logger().error("无法初始化摄像头")
        raise RuntimeError("Camera initialization failed")

    def process_frame(self):
        """改进的推理流程"""
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("摄像头读取失败，尝试重新初始化")
                self._init_camera()
                return
            
            # 预处理
            img = self._preprocess(frame)

            # 推理（参数获取优化）
            with torch.no_grad():
                pred = self.model(img, augment=False)
                pred = non_max_suppression(
                    pred[0],
                    conf_thres=self.get_parameter('conf_thres').value,
                    iou_thres=self.get_parameter('iou_thres').value,
                    max_det=self.get_parameter('max_det').value
                )

            # 后处理
            self._postprocess(pred, frame, img)
            
        except Exception as e:
            self.get_logger().error(f"帧处理异常: {str(e)}")
            torch.cuda.empty_cache()

    def _preprocess(self, img):
        """预处理流程"""
        img = letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.float() / 255.0
        if len(img.shape) == 3:
            img = img[None]  # 扩展批次维度
        return img
    
    def _postprocess(self, pred, frame, img):
        annotator = Annotator(
            frame, 
            line_width=self.get_parameter('line_thickness').value,
            example=str(self.names)
        )
        
        detections = []
        for det in enumerate(pred):  # 遍历每张图像的检测结果
            if det is not None and len(det):
                # 修正缩放参数顺序
                det[:, :4] = scale_boxes(
                    img.shape[2:],  # 输入特征图尺寸
                    det[:, :4],     # 原始坐标
                    (frame.shape[1], frame.shape[0])     # 原始图像尺寸
                ).round()
                
                for *xyxy, conf, cls in reversed(det):  # 转换为numpy处理

                    # 构建检测消息
                    result = DetectResult()
                    result.xyxy = [int(x) for x in xyxy]
                    result.conf = float(conf)
                    detections.append(result)
                    
                    # 绘制标注
                    label = f"{self.names.get(int(cls), 'unknown')} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(int(cls), True))

        # 发布结果
        self._publish_results(annotator.result(), detections)

    def _publish_results(self, vis_img, detections):
        """结果发布"""
        # 图像编码
        success, jpeg = cv2.imencode('.jpg', vis_img, 
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not success:
            self.get_logger().error("JPEG encoding failed")
            return
        
        # 构建消息
        img_msg = ImageResult()
        img_msg.image = self._cv2_to_imgmsg(jpeg)
        img_msg.results = detections
        
        # 单独发布检测结果
        self.detection_pub.publish(DetectResult(results=detections))
        self.visualization_pub.publish(img_msg)

    def _cv2_to_imgmsg(self, jpeg):
        """OpenCV转ROS Image消息"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = jpeg.shape[0]
        msg.width = jpeg.shape[1]
        msg.encoding = 'jpeg'
        msg.is_bigendian = 0
        msg.step = 0
        msg.data = jpeg.tobytes()
        return msg
    def destroy_node(self):
        """改进的资源释放"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        torch.cuda.empty_cache()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        detector = YOLOv5Detect()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()