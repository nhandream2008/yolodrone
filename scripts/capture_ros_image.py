#!/usr/bin/env python3
"""Save one ROS 2 bgr8 image to verify the annotated YOLO stream."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/yolo/image_annotated")
    parser.add_argument("--output", default="output/annotated_frame.png")
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("timeout must be positive")

    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    result = {"image": None}

    class Capture(Node):
        def __init__(self):
            super().__init__("yolo_image_capture")
            self.subscription = self.create_subscription(Image, args.topic, self.receive, 1)

        def receive(self, message):
            if message.encoding != "bgr8" or message.step != message.width * 3:
                self.get_logger().error(f"Expected packed bgr8 image, received {message.encoding} step={message.step}")
                return
            result["image"] = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3).copy()

    rclpy.init()
    node = Capture()
    try:
        rclpy.spin_once(node, timeout_sec=args.timeout)
        if result["image"] is None:
            raise TimeoutError(f"No image received from {args.topic} within {args.timeout} seconds")
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output), result["image"]):
            raise RuntimeError(f"Could not write {output}")
        print(f"Saved {output} ({result['image'].shape[1]}x{result['image'].shape[0]})")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
