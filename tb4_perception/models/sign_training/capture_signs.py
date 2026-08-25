#!/usr/bin/env python3
"""Capture OAK-D preview frames for sign-detector fine-tuning.

Saves frames from the SAME compressed topic the YOLO node consumes, so the
training images match the runtime domain (resolution, JPEG compression, lens).
Drive the robot / move the sign to vary distance, angle, and lighting.

Usage (robot powered on, RGB preview publishing):
    python3 capture_signs.py --label do_not_enter          # positives
    python3 capture_signs.py --label stop                  # positives
    python3 capture_signs.py --label negative --rate 3     # no-sign frames

Frames land in <out>/<label>/frame_<ms>.jpg. Ctrl-C to stop.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class SignCapture(Node):
    def __init__(self, outdir, rate, topic):
        super().__init__('sign_capture')
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self.period = 1.0 / rate if rate > 0 else 0.0
        self.last = 0.0
        self.count = 0
        self.create_subscription(
            CompressedImage, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f'Capturing from {topic} @ {rate} Hz -> {outdir}  (Ctrl-C to stop)')

    def on_image(self, msg):
        now = time.time()
        if now - self.last < self.period:
            return
        self.last = now
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        fn = os.path.join(self.outdir, f'frame_{int(now * 1000)}.jpg')
        cv2.imwrite(fn, img)
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(
                f'saved {self.count} frames ({img.shape[1]}x{img.shape[0]})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True,
                    help='subfolder / intended class (e.g. do_not_enter, stop, negative)')
    ap.add_argument('--out', default=os.path.expanduser(
        '~/turtlebot4_ws/sign_training/raw'))
    ap.add_argument('--rate', type=float, default=2.0, help='frames per second')
    ap.add_argument('--topic',
                    default='/oakd/rgb/preview/image_raw/compressed')
    a = ap.parse_args()
    outdir = os.path.join(a.out, a.label)
    rclpy.init()
    node = SignCapture(outdir, a.rate, a.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info(f'Total saved to {outdir}: {node.count}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
