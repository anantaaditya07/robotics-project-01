#!/usr/bin/env python3
"""Export YOLOv8 weights (.pt) to ONNX for the SemNav C++ detector.

Run from the project tooling venv, never inside a ROS-sourced shell:

    .venv/bin/python scripts/export_yolo.py [--force]

Defaults reproduce models/yolov8n.onnx: static 1x3x640x640 input, opset 12,
simplified graph, no embedded NMS (output0 is 1x84x8400).

Ultralytics weights are licensed AGPL-3.0 (https://ultralytics.com/license).

Ultralytics always writes the .onnx next to the .pt it loads, so the export
runs on a temporary copy of the weights and the result is then moved to
--output-dir. This guarantees nothing next to the original weights is touched.
"""

import argparse
import shutil
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def repo_path(p: str) -> Path:
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def print_io(onnx_path: Path) -> None:
    import onnx

    model = onnx.load(str(onnx_path))

    def dims(v):
        return [d.dim_value or d.dim_param for d in v.type.tensor_type.shape.dim]

    print(f"opset: {[(o.domain or 'ai.onnx', o.version) for o in model.opset_import]}")
    for i in model.graph.input:
        print(f"input  {i.name}: {dims(i)}")
    for o in model.graph.output:
        print(f"output {o.name}: {dims(o)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--weights", default="models/yolov8n.pt",
                    help="path to .pt weights (relative to repo root)")
    ap.add_argument("--imgsz", type=int, default=640, help="square input size")
    ap.add_argument("--opset", type=int, default=12, help="ONNX opset")
    ap.add_argument("--simplify", action=argparse.BooleanOptionalAction, default=True,
                    help="simplify the exported graph")
    ap.add_argument("--output-dir", default="models",
                    help="directory for the .onnx (relative to repo root)")
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing output .onnx")
    args = ap.parse_args()

    weights = repo_path(args.weights)
    if not weights.is_file():
        print(f"error: weights not found: {weights}", file=sys.stderr)
        return 1
    out_dir = repo_path(args.output_dir)
    target = out_dir / f"{weights.stem}.onnx"
    if target.exists() and not args.force:
        print(f"error: {target} exists; pass --force to overwrite", file=sys.stderr)
        return 1

    from ultralytics import YOLO

    with tempfile.TemporaryDirectory() as tmp:
        tmp_weights = Path(tmp) / weights.name
        shutil.copy2(weights, tmp_weights)
        exported = YOLO(str(tmp_weights)).export(
            format="onnx", imgsz=args.imgsz, opset=args.opset, simplify=args.simplify)
        out_dir.mkdir(parents=True, exist_ok=True)
        shutil.move(str(exported), str(target))

    print(f"wrote {target}")
    print_io(target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
