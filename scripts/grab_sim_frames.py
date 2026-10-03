#!/usr/bin/env python3
"""Save camera frames from a running SemNav Gazebo sim as PNG files.

Run with the SYSTEM python in a ROS-sourced shell, never from the .venv:

    source /opt/ros/humble/setup.bash && /usr/bin/python3 scripts/grab_sim_frames.py

Subscribes to --topic (sensor_msgs/Image, sensor-data QoS to match the Gazebo
camera plugin), converts each kept frame to bgr8 with cv_bridge so colours are
correct for cv2.imwrite, and writes frame_000.png, frame_001.png, ... into
--output-dir. Exits 0 once --count frames are saved, 1 on --timeout.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

REPO_ROOT = Path(__file__).resolve().parent.parent


def repo_path(p: str) -> Path:
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


class FrameGrabber(Node):
    def __init__(self, topic: str, out_dir: Path, count: int, every: int):
        super().__init__("grab_sim_frames")
        self.out_dir = out_dir
        self.count = count
        self.every = every
        self.received = 0
        self.saved = 0
        self.error = None
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, topic, self.on_image, qos_profile_sensor_data)

    def done(self) -> bool:
        return self.saved >= self.count or self.error is not None

    def on_image(self, msg: Image) -> None:
        if self.done():
            return
        self.received += 1
        if (self.received - 1) % self.every != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # cv_bridge raises CvBridgeError or others
            self.error = f"cv_bridge conversion failed ({msg.encoding}): {e}"
            return
        path = self.out_dir / f"frame_{self.saved:03d}.png"
        if not cv2.imwrite(str(path), frame):
            self.error = f"cv2.imwrite failed for {path}"
            return
        stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        print(f"saved {path}  {msg.width}x{msg.height}  {msg.encoding}  stamp={stamp}",
              flush=True)
        self.saved += 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--topic", default="/camera/image_raw",
                    help="sensor_msgs/Image topic to subscribe to")
    ap.add_argument("--count", type=int, default=20, help="number of frames to save")
    ap.add_argument("--output-dir", default="data/sim_frames",
                    help="directory for the PNGs (relative to repo root)")
    ap.add_argument("--every", type=int, default=1,
                    help="save every Nth received frame")
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="total wall-clock seconds before giving up")
    args = ap.parse_args()

    if args.count < 1 or args.every < 1 or args.timeout <= 0:
        print("error: --count and --every must be >= 1, --timeout must be > 0",
              file=sys.stderr)
        return 1

    out_dir = repo_path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = None
    try:
        node = FrameGrabber(args.topic, out_dir, args.count, args.every)
        print(f"waiting for {args.count} frames on {args.topic} -> {out_dir}", flush=True)
        deadline = time.monotonic() + args.timeout
        while not node.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            rclpy.spin_once(node, timeout_sec=min(remaining, 0.5))

        if node.error is not None:
            print(f"error: {node.error} (saved {node.saved}/{args.count})", file=sys.stderr)
            return 1
        if node.saved < args.count:
            print(f"error: timeout after {args.timeout:g} s on {args.topic}: "
                  f"saved {node.saved}/{args.count} frames "
                  f"({node.received} received) to {out_dir}", file=sys.stderr)
            return 1
        print(f"done: saved {node.saved} frames to {out_dir}")
        return 0
    except KeyboardInterrupt:
        saved = node.saved if node is not None else 0
        print(f"error: interrupted, saved {saved}/{args.count} frames", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
