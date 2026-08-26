#!/usr/bin/env python3
"""Fine-tune yolov8n on captured sign images (UserPC GPU).

Expects a YOLOv8-format dataset: a data.yaml pointing at train/val image dirs
with matching label .txt files, and:
    names: ['stop sign', 'do not enter']
Using those exact names means the trained model emits our canonical class_ids
directly, so no post-training rename is needed -- just copy best.pt over
models/traffic_signs.pt.

STOP and DO NOT ENTER are not left-right symmetric (STOP text), so horizontal
flip is disabled; rotation/scale/brightness augmentation covers angle, distance
and lighting variation instead.
"""
import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True, help='path to data.yaml')
    ap.add_argument('--base', default='yolov8n.pt', help='starting weights')
    ap.add_argument('--epochs', type=int, default=120)
    ap.add_argument('--imgsz', type=int, default=416,
                    help='train size; keep near the OAK-D preview size')
    ap.add_argument('--device', default='0', help='GPU id, or cpu')
    ap.add_argument('--name', default='sign_ft')
    a = ap.parse_args()

    model = YOLO(a.base)
    model.train(
        data=a.data, epochs=a.epochs, imgsz=a.imgsz, device=a.device,
        name=a.name, patience=30,
        fliplr=0.0,            # signs aren't horizontally symmetric (STOP text)
        degrees=12, translate=0.1, scale=0.5, hsv_v=0.4, mosaic=1.0,
    )
    print(f'Best weights: runs/detect/{a.name}/weights/best.pt')
    print('Deploy: cp runs/detect/%s/weights/best.pt '
          '~/turtlebot4_ws/models/traffic_signs.pt' % a.name)


if __name__ == '__main__':
    main()
