#!/usr/bin/env python3
"""Run the YOLOv8 ONNX model on saved frames and report what it detects.

Run from the project tooling venv, never inside a ROS-sourced shell:

    .venv/bin/python scripts/check_yolo_on_frames.py [--input-dir data/sim_frames]

Mirrors the yolo_onnx_node pipeline from docs/architecture.txt section 7.1:
letterbox (keep aspect, pad grey 114) -> BGR to RGB -> float32 /255 -> NCHW
-> session.run -> decode ([1,84,8400], no objectness, max class score,
conf_threshold) -> NMS (IoU iou_threshold) -> undo letterbox.

Every *.png in --input-dir is processed; an annotated copy with the same
filename is written to --output-dir. A per-class summary (max confidence,
frame count) and the mean inference time are printed at the end.
"""

import argparse
import ast
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

REPO_ROOT = Path(__file__).resolve().parent.parent

PAD_VALUE = 114  # letterbox grey, per architecture 7.1
NUM_BOX_VALUES = 4  # cx, cy, w, h ahead of the class scores


def repo_path(p: str) -> Path:
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def letterbox(img: np.ndarray, size: int):
    """Resize keeping aspect into a size x size canvas, centred, padded grey.

    Returns (padded, scale, (pad_x, pad_y)) where an original pixel p maps to
    p * scale + pad in the padded image.
    """
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    padded = np.full((size, size, 3), PAD_VALUE, dtype=np.uint8)
    padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return padded, scale, (pad_x, pad_y)


def preprocess(padded_bgr: np.ndarray) -> np.ndarray:
    """BGR uint8 HWC -> RGB float32 /255 NCHW with batch 1."""
    rgb = cv2.cvtColor(padded_bgr, cv2.COLOR_BGR2RGB)
    chw = rgb.astype(np.float32).transpose(2, 0, 1) / 255.0
    return np.ascontiguousarray(chw[np.newaxis])


def decode(output: np.ndarray, conf_thr: float):
    """Decode a [1, 4+C, N] YOLOv8 output.

    Returns (boxes_xywh [K,4] in input pixels, top-left based; scores [K];
    class_ids [K]) for anchors whose best class score >= conf_thr.
    """
    preds = output[0].T  # [N, 4+C]
    class_scores = preds[:, NUM_BOX_VALUES:]
    class_ids = class_scores.argmax(axis=1)
    scores = class_scores[np.arange(len(class_ids)), class_ids]
    keep = scores >= conf_thr
    cxcywh = preds[keep, :NUM_BOX_VALUES]
    boxes = cxcywh.copy()
    boxes[:, 0] = cxcywh[:, 0] - cxcywh[:, 2] / 2.0
    boxes[:, 1] = cxcywh[:, 1] - cxcywh[:, 3] / 2.0
    return boxes, scores[keep], class_ids[keep]


def nms(boxes: np.ndarray, scores: np.ndarray, class_ids: np.ndarray,
        conf_thr: float, iou_thr: float) -> np.ndarray:
    """Per-class NMS using the batched-offset trick; returns kept indices.

    Each box is shifted by class_id * offset (offset larger than any
    coordinate), so boxes of different classes never overlap and a single
    cv2.dnn.NMSBoxes call only suppresses within a class.
    """
    if len(boxes) == 0:
        return np.empty(0, dtype=int)
    offset = float(boxes[:, :2].max() + boxes[:, 2:].max()) + 1.0
    shifted = boxes.copy()
    shifted[:, :2] += class_ids[:, np.newaxis] * offset
    idx = cv2.dnn.NMSBoxes(shifted.tolist(), scores.tolist(), conf_thr, iou_thr)
    return np.asarray(idx, dtype=int).reshape(-1)


def unletterbox_boxes(boxes: np.ndarray, scale: float, pad, img_w: int,
                      img_h: int) -> np.ndarray:
    """Map xywh boxes from letterboxed input pixels to original x1,y1,x2,y2,
    clipped to the image."""
    pad_x, pad_y = pad
    x1 = (boxes[:, 0] - pad_x) / scale
    y1 = (boxes[:, 1] - pad_y) / scale
    x2 = (boxes[:, 0] + boxes[:, 2] - pad_x) / scale
    y2 = (boxes[:, 1] + boxes[:, 3] - pad_y) / scale
    out = np.stack([x1, y1, x2, y2], axis=1)
    out[:, [0, 2]] = out[:, [0, 2]].clip(0, img_w - 1)
    out[:, [1, 3]] = out[:, [1, 3]].clip(0, img_h - 1)
    return out


def class_color(class_id: int):
    rng = np.random.default_rng(class_id)
    return tuple(int(c) for c in rng.integers(64, 256, size=3))


def draw(img: np.ndarray, dets) -> np.ndarray:
    out = img.copy()
    for (x1, y1, x2, y2), name, conf, cid in dets:
        color = class_color(cid)
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(out, p1, p2, color, 2)
        label = f"{name} {conf:.2f}"
        (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(p1[1], th + base)
        cv2.rectangle(out, (p1[0], ty - th - base), (p1[0] + tw, ty), color, -1)
        cv2.putText(out, label, (p1[0], ty - base), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", default="models/yolov8n.onnx",
                    help="ONNX model (relative to repo root)")
    ap.add_argument("--input-dir", default="data/sim_frames",
                    help="directory of *.png frames (relative to repo root)")
    ap.add_argument("--output-dir", default="data/yolo_check",
                    help="directory for annotated frames (relative to repo root)")
    ap.add_argument("--conf-threshold", type=float, default=0.35,
                    help="minimum class score to keep a detection")
    ap.add_argument("--iou-threshold", type=float, default=0.45, help="NMS IoU threshold")
    ap.add_argument("--input-size", type=int, default=None,
                    help="square network input size (default: read from the model)")
    ap.add_argument("--threads", type=int, default=4, help="ORT intra_op_num_threads")
    args = ap.parse_args()

    model = repo_path(args.model)
    if not model.is_file():
        print(f"error: model not found: {model}", file=sys.stderr)
        return 1
    in_dir = repo_path(args.input_dir)
    frames = sorted(in_dir.glob("*.png")) if in_dir.is_dir() else []
    if not frames:
        print(f"error: no *.png files found in {in_dir}", file=sys.stderr)
        return 1
    out_dir = repo_path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = args.threads
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(str(model), sess_options=opts,
                                   providers=["CPUExecutionProvider"])
    inp = session.get_inputs()[0]
    input_size = args.input_size or int(inp.shape[2])
    names = ast.literal_eval(session.get_modelmeta().custom_metadata_map["names"])
    print(f"model {model} input {inp.name} {inp.shape} -> size {input_size}, "
          f"{len(names)} classes, {len(frames)} frames")

    infer_ms = []
    stats = {}  # name -> [max_conf, frame_count]
    total = 0
    for path in frames:
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            print(f"{path.name}: unreadable, skipped")
            continue
        h, w = img.shape[:2]
        padded, scale, pad = letterbox(img, input_size)
        tensor = preprocess(padded)
        t0 = time.perf_counter()
        output = session.run(None, {inp.name: tensor})[0]
        infer_ms.append((time.perf_counter() - t0) * 1000.0)

        boxes, scores, cids = decode(output, args.conf_threshold)
        keep = nms(boxes, scores, cids, args.conf_threshold, args.iou_threshold)
        xyxy = unletterbox_boxes(boxes[keep], scale, pad, w, h)
        dets = [(tuple(b), names.get(int(c), str(int(c))), float(s), int(c))
                for b, s, c in zip(xyxy, scores[keep], cids[keep])]
        dets.sort(key=lambda d: -d[2])

        cv2.imwrite(str(out_dir / path.name), draw(img, dets))
        total += len(dets)
        print(f"{path.name} ({w}x{h}): {len(dets)} detections")
        for (x1, y1, x2, y2), name, conf, _ in dets:
            print(f"    {name} {conf:.3f} [{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")
        for name in {d[1] for d in dets}:
            best = max(d[2] for d in dets if d[1] == name)
            s = stats.setdefault(name, [0.0, 0])
            s[0] = max(s[0], best)
            s[1] += 1

    print()
    if total == 0:
        print("NO DETECTIONS in any frame")
    else:
        print(f"{'class':<16}{'max_conf':>10}{'frames':>8}")
        for name, (mx, n) in sorted(stats.items(), key=lambda kv: -kv[1][0]):
            print(f"{name:<16}{mx:>10.3f}{n:>8d}")
    if infer_ms:
        print(f"mean inference time: {np.mean(infer_ms):.1f} ms over {len(infer_ms)} frames")
    print(f"annotated frames written to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
