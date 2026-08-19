"""YOLO Detector Node (v 0.02)

Runs YOLOv8 inference on the OAK-D RGB preview stream: receives each
camera frame, filters detections to a configurable set of target classes,
and publishes 2D bounding boxes with class labels and confidence scores
as a Detection2DArray for downstream fusion / planning nodes.
Optionally publishes an annotated image with drawn bounding boxes for
live visualization in RViz.

Subscribes to /oakd/rgb/preview/image_raw
    Msg type: sensor_msgs/msg/Image
Publishes to /detections 
    Msg type: vision_msgs/msg/Detection2DArray
Optional publishes to /detections/image 
    Msg type: sensor_msgs/msg/Image

Quick Test:
    Terminal 1: 
        tb4 undock
        ssh -t ubuntu@turtlebot4 "sudo systemctl restart turtlebot4.service"

        ros2 topic list
        ros2 topic info /oakd/rgb/preview/image_raw -v

        timeout 8 ros2 topic hz /oakd/rgb/preview/image_raw --window 10

    Terminal 2: 
        tb4 yolo viz

    Terminal 3:         
        ros2 topic list
        ros2 topic info /detections -v
        timeout 8 ros2 topic hz /detections --window 10
    
    Results:
        https://docs.google.com/document/d/1JeNdLnQ-b9u_FkvoEUyvisUP6JDX1xfCAqdooLeP6cg/edit?tab=t.0
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

# Optional: only needed when image_transport == 'ffmpeg' (OAK-D low_bandwidth).
try:
    from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
except ImportError:
    FFMPEGPacket = None

from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', '/home/qd/turtlebot4_ws/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_rate_limit', 15.0)
        self.declare_parameter('target_classes', [
            'person', 'stop sign', 'chair', 'dog', 'cat', 'bicycle',
        ])
        self.declare_parameter('publish_visualisation', False)
        self.declare_parameter('visualisation_topic', '/detections/image')
        # 'raw' subscribes to sensor_msgs/Image; 'ffmpeg' subscribes to the
        # OAK-D low_bandwidth MJPEG stream (<image_topic>/ffmpeg).
        self.declare_parameter('image_transport', 'raw')
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
        self.class_names = self.model.names  # {0: 'person', 1: 'bicycle', ...}

        # Build a set of target class indices for fast lookup
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
        elif self.image_transport == 'ffmpeg':
            if FFMPEGPacket is None:
                raise RuntimeError(
                    "image_transport 'ffmpeg' requires ffmpeg_image_transport_msgs "
                    '(sudo apt install ros-jazzy-ffmpeg-image-transport-msgs)'
                )
            subscribed_topic = image_topic + '/ffmpeg'
            self.image_sub = self.create_subscription(
                FFMPEGPacket, subscribed_topic, self.ffmpeg_callback, sensor_qos
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
        if self._rate_limited():
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(cv_image, msg.header)

    def compressed_callback(self, msg: CompressedImage):
        if self._rate_limited():
            return
        cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn('Failed to decode CompressedImage frame.')
            return
        self.process_frame(cv_image, msg.header)

    def ffmpeg_callback(self, msg):
        if self._rate_limited():
            return
        # OAK-D low_bandwidth uses MJPEG, so each packet is a standalone JPEG frame.
        cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn(
                f'Failed to decode FFMPEG packet (encoding={msg.encoding}); expected MJPEG.'
            )
            return
        self.process_frame(cv_image, msg.header)

    def _rate_limited(self) -> bool:
        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds < self.min_period_ns:
            return True
        self.last_publish_time = now
        return False

    def process_frame(self, cv_image: np.ndarray, header):

        # Run inference
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header = header

        if len(results) == 0 or results[0].boxes is None:
            self.detection_pub.publish(det_array)
            if self.vis_pub is not None:
                self.publish_visualisation(cv_image, det_array)
            return

        boxes = results[0].boxes
        for box in boxes:
            cls_id = int(box.cls[0].item())

            # Skip categories we don't care about
            if cls_id not in self.target_indices:
                continue

            confidence = float(box.conf[0].item())

            # xyxy bounding box (pixels)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            det = Detection2D()
            det.header = header

            # Center + size (Detection2D convention)
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.class_names[cls_id]
            hyp.hypothesis.score = confidence
            det.results.append(hyp)

            det_array.detections.append(det)

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