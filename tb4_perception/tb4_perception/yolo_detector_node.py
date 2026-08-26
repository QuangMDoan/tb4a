"""YOLO Detector Node (v 0.04)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', '/home/qd/turtlebot4_ws/models/traffic_signs.pt')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_rate_limit', 15.0)
        self.declare_parameter('target_classes', [
            'stop sign', 'do not enter'
        ])
        self.declare_parameter('publish_visualisation', False)
        self.declare_parameter('visualisation_topic', '/detections/image')
        self.declare_parameter('image_transport', 'compressed')
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        device = self.get_parameter('device').value
        self.rate_limit = self.get_parameter('publish_rate_limit').value
        self.target_classes = self.get_parameter('target_classes').value
        self.publish_vis = self.get_parameter('publish_visualisation').value
        vis_topic = self.get_parameter('visualisation_topic').value
        self.image_transport = self.get_parameter('image_transport').value

        # --- YOLO model ---
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.class_names = self.model.names  

        # Build a set of target class indices for fast lookup.
        self.target_indices: set[int] = set()
        for idx, name in self.class_names.items():
            if name in self.target_classes:
                self.target_indices.add(idx)
        self.get_logger().info(
            f'Tracking classes: {[self.class_names[i] for i in sorted(self.target_indices)]}'
        )

        # --- ROS I/O ---
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray, detection_topic, 10
        )

        if self.publish_vis:
            self.vis_pub = self.create_publisher(Image, vis_topic, 10)
        else:
            self.vis_pub = None

        if self.image_transport == 'compressed':
            subscribed_topic = image_topic + '/compressed'
            self.image_sub = self.create_subscription(
                CompressedImage, subscribed_topic, self.compressed_callback, sensor_qos
            )
        else:
            subscribed_topic = image_topic
            self.image_sub = self.create_subscription(
                Image, image_topic, self.image_callback, sensor_qos
            )

        # Rate-limiting state
        self.last_publish_time = self.get_clock().now()
        self.min_period_ns = int(1e9 / self.rate_limit) if self.rate_limit > 0 else 0

        self.get_logger().info(
            f'YoloDetectorNode ready — subscribing to {subscribed_topic} '
            f'({self.image_transport}), publishing to {detection_topic}'
        )
    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def image_callback(self, msg: Image):
        if self.rate_limited():
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(cv_image, msg.header)

    def compressed_callback(self, msg: CompressedImage):
        if self.rate_limited():
            return
        cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn('Failed to decode CompressedImage frame.')
            return
        self.process_frame(cv_image, msg.header)

    def rate_limited(self) -> bool:
        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds < self.min_period_ns:
            return True
        self.last_publish_time = now
        return False

    def process_frame(self, cv_image: np.ndarray, header):

        # Traffic-sign YOLO model — STOP + DO NOT ENTER.
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        det_array = Detection2DArray()
        det_array.header = header

        boxes = results[0].boxes if (results and results[0].boxes is not None) else None
        if boxes is not None:
            for box in boxes:
                cls_id = int(box.cls[0].item())
                if cls_id not in self.target_indices:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                self.add_detection(
                    det_array, header, self.class_names[cls_id],
                    float(box.conf[0].item()), x1, y1, x2, y2)

        self.detection_pub.publish(det_array)

        # Publish annotated image for visualisation
        if self.vis_pub is not None:
            self.publish_visualisation(cv_image, det_array)

        if det_array.detections:
            summary = ', '.join(
                f'{d.results[0].hypothesis.class_id} '
                f'({d.results[0].hypothesis.score:.2f})'
                for d in det_array.detections
            )
            self.get_logger().debug(f'Published {len(det_array.detections)} detections: {summary}')

    # ------------------------------------------------------------------
    # Detection helpers
    # ------------------------------------------------------------------

    def add_detection(self, det_array, header, class_id, score, x1, y1, x2, y2):
        det = Detection2D()
        det.header = header
        det.bbox.center.position.x = float((x1 + x2) / 2.0)
        det.bbox.center.position.y = float((y1 + y2) / 2.0)
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score = float(score)
        det.results.append(hyp)
        det_array.detections.append(det)

    # ------------------------------------------------------------------
    # Visualisation helper
    # ------------------------------------------------------------------

    VIS_COLORS = [
        (0, 255, 0), (255, 0, 0), (0, 0, 255),
        (255, 255, 0), (0, 255, 255), (255, 0, 255),
    ]

    def publish_visualisation(self, cv_image: np.ndarray, det_array: Detection2DArray):
        vis = cv_image.copy()
        for i, det in enumerate(det_array.detections):
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            x1 = int(cx - w / 2.0)
            y1 = int(cy - h / 2.0)
            x2 = int(cx + w / 2.0)
            y2 = int(cy + h / 2.0)

            color = self.VIS_COLORS[i % len(self.VIS_COLORS)]
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

            if det.results:
                label = (
                    f'{det.results[0].hypothesis.class_id} '
                    f'{det.results[0].hypothesis.score:.2f}'
                )
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
                cv2.putText(
                    vis, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA,
                )

        vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        vis_msg.header = det_array.header
        self.vis_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()