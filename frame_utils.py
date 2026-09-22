"""Decode Gazebo msgs10 image data, including row padding. No ROS dependency."""
import numpy as np


def image_to_bgr(msg):
    channels = {1: 1, 3: 3, 4: 4, 5: 4, 8: 3}.get(msg.pixel_format_type)
    if channels is None:
        raise ValueError(f"Unsupported Gazebo pixel format: {msg.pixel_format_type}")
    width, height = int(msg.width), int(msg.height)
    if width <= 0 or height <= 0:
        raise ValueError("Empty image dimensions")
    packed = width * channels
    step = int(msg.step) or packed
    if step < packed or len(msg.data) < step * height:
        raise ValueError("Invalid image stride or truncated image data")
    rows = np.frombuffer(msg.data, dtype=np.uint8, count=step * height).reshape(height, step)
    pixels = rows[:, :packed].reshape(height, width, channels)
    if msg.pixel_format_type == 1:
        pixels = np.repeat(pixels, 3, axis=2)
    elif msg.pixel_format_type in (3, 4):
        pixels = pixels[:, :, :3][:, :, ::-1]
    else:
        pixels = pixels[:, :, :3]
    return np.ascontiguousarray(pixels)
