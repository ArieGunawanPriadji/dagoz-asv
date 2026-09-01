#!/usr/bin/env python3
import os
from pathlib import Path
import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

try:
    import torch
except Exception:
    torch = None


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.cb_image, 10)
        self.pub = self.create_publisher(String, 'ball_detector/detections', 10)

        # locate best.pt relative to this script: ../../ball_detector_pkg/weights/best.pt (from src/perception/scripts)
        script_path = Path(__file__).resolve()
        src_dir = script_path.parents[2]  # .../src
        weights_path = src_dir / 'ball_detector_pkg' / 'weights' / 'best.pt'
        self.weights = str(weights_path)

        self.model = None
        self.model_lock = threading.Lock()
        if torch is not None and weights_path.exists():
            try:
                # try to load a YOLOv5/Ultralytics style model via torch.hub
                self.get_logger().info(f'Attempting to load model from {self.weights}')
                self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.weights, force_reload=False)
                self.get_logger().info('Model loaded (torch.hub)')
            except Exception as e:
                self.get_logger().warn(f'Could not load model via torch.hub: {e}. Falling back to simple HSV detector.')
                self.model = None
        else:
            if torch is None:
                self.get_logger().warn('PyTorch not available; running fallback HSV detector.')
            else:
                self.get_logger().warn(f'Weights file not found at {self.weights}; running fallback HSV detector.')

    def cb_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        detections = []

        # If model available, run inference
        if self.model is not None:
            try:
                # the ultralytics model accepts numpy images in BGR or RGB depending on version
                results = self.model(cv_img)
                # results.xyxy[0] -> (x1, y1, x2, y2, conf, cls)
                xy = results.xyxy[0].cpu().numpy()
                for row in xy:
                    x1, y1, x2, y2, conf, cls = row.tolist()
                    cx = float((x1 + x2) / 2.0)
                    cy = float((y1 + y2) / 2.0)
                    detections.append({
                        'bbox': [float(x1), float(y1), float(x2), float(y2)],
                        'confidence': float(conf),
                        'class': int(cls),
                        'center': [cx, cy]
                    })
            except Exception as e:
                self.get_logger().error(f'Model inference failed: {e}')

        # fallback simple HSV circle detection when no model/detection available
        if not detections:
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            # generic orange-ish ball range; user can tune
            lower = (5, 100, 100)
            upper = (25, 255, 255)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.medianBlur(mask, 5)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < 200:  # skip small
                    continue
                (x, y), r = cv2.minEnclosingCircle(c)
                detections.append({
                    'bbox': [float(x - r), float(y - r), float(x + r), float(y + r)],
                    'confidence': 0.5,
                    'class': 0,
                    'center': [float(x), float(y)],
                    'area': float(area)
                })

        # publish detections as JSON string
        msg = String()
        msg.data = json.dumps({'header': {'frame_id': msg.header.frame_id if hasattr(msg, "header") else 'camera'}, 'detections': detections})
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
