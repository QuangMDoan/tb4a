#!/usr/bin/env python3
"""Auto-label captured sign frames into a YOLOv8 dataset (bootstrap, then review).

Each raw frame's class is known from its capture folder, so we only need a *box*.
An open-vocabulary detector (YOLO-World) localises the sign; the class is taken
from the folder name -- this sidesteps YOLO-World's tendency to mislabel a
DO NOT ENTER sign as a "stop sign" while still getting a tight, correct box.
  - do_not_enter / stop frames: highest-confidence YOLO-World box, folder class.
  - negative frames:            empty label files (what is NOT a sign).

Input layout (from capture_signs.py):  <raw>/{do_not_enter,stop,negative}/*.jpg
Output: a YOLOv8 dataset (images/, labels/, data.yaml) with class ids
  0 = 'stop sign', 1 = 'do not enter'  -- so a model trained on it emits our
  canonical class_ids directly.

Review the drawn boxes in <out>/review and delete any bad frame (image + its
labels/*/<name>.txt) before training.

Usage:
    python3 autolabel.py --raw ~/turtlebot4_ws/sign_training/raw \
                         --out ~/turtlebot4_ws/sign_training/dataset
"""
import argparse
import glob
import os
import random

import cv2

CLASS_STOP = 0
CLASS_DNE = 1

PROMPTS = ['stop sign', 'do not enter sign', 'no entry sign',
           'red traffic sign', 'red and white sign']


def best_box(model, img, conf, imgsz):
    """Return the highest-confidence YOLO-World box as (x1, y1, x2, y2) or None."""
    res = model.predict(img, conf=conf, imgsz=imgsz, verbose=False)[0]
    if res.boxes is None or len(res.boxes) == 0:
        return None
    b = max(res.boxes, key=lambda b: float(b.conf[0]))
    x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].cpu().numpy())
    return (x1, y1, x2, y2)


def yolo_line(cls, x1, y1, x2, y2, w, h):
    cx = (x1 + x2) / 2.0 / w
    cy = (y1 + y2) / 2.0 / h
    bw = (x2 - x1) / float(w)
    bh = (y2 - y1) / float(h)
    return f'{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', default=os.path.expanduser(
        '~/turtlebot4_ws/sign_training/raw'))
    ap.add_argument('--out', default=os.path.expanduser(
        '~/turtlebot4_ws/sign_training/dataset'))
    ap.add_argument('--world-model', default='yolov8x-worldv2.pt')
    ap.add_argument('--conf', type=float, default=0.03)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--val-split', type=float, default=0.2)
    a = ap.parse_args()

    from ultralytics import YOLOWorld
    model = YOLOWorld(a.world_model)
    model.set_classes(PROMPTS)

    for sub in ('images/train', 'images/val', 'labels/train', 'labels/val', 'review'):
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)

    jobs = []  # (path, label, class-or-None-for-negative)
    for label, cls in (('do_not_enter', CLASS_DNE), ('stop', CLASS_STOP),
                       ('negative', None)):
        for p in sorted(glob.glob(os.path.join(a.raw, label, '*.jpg'))):
            jobs.append((p, label, cls))

    random.seed(0)
    random.shuffle(jobs)
    n_val = int(len(jobs) * a.val_split)
    stats = {'do_not_enter': 0, 'stop': 0, 'negative': 0, 'no_box': 0}

    for i, (path, label, cls) in enumerate(jobs):
        img = cv2.imread(path)
        if img is None:
            continue
        h, w = img.shape[:2]
        box = None if cls is None else best_box(model, img, a.conf, a.imgsz)
        boxes = [(box, cls)] if box is not None else []

        split = 'val' if i < n_val else 'train'
        stem = f'{label}_{os.path.splitext(os.path.basename(path))[0]}'
        cv2.imwrite(os.path.join(a.out, 'images', split, stem + '.jpg'), img)
        lines = [yolo_line(c, *b, w, h) for (b, c) in boxes]
        with open(os.path.join(a.out, 'labels', split, stem + '.txt'), 'w') as f:
            f.write('\n'.join(lines))

        if label != 'negative' and not boxes:
            stats['no_box'] += 1
        stats[label] += 1
        vis = img.copy()
        for (x1, y1, x2, y2), c in boxes:
            col = (0, 0, 255) if c == CLASS_DNE else (0, 255, 0)
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
        cv2.imwrite(os.path.join(a.out, 'review', stem + '.jpg'), vis)

    with open(os.path.join(a.out, 'data.yaml'), 'w') as f:
        f.write(f'path: {os.path.abspath(a.out)}\n'
                'train: images/train\nval: images/val\n'
                "names:\n  0: 'stop sign'\n  1: 'do not enter'\n")

    print(f'Labeled: {stats}')
    print(f'Frames with NO auto-box (review/fix these): {stats["no_box"]}')
    print(f'Review the drawn boxes in: {os.path.join(a.out, "review")}')
    print('Delete any bad frame\'s image + matching labels/*/<name>.txt, then:')
    print(f'  python3 train_signs.py --data {os.path.join(a.out, "data.yaml")}')


if __name__ == '__main__':
    main()
