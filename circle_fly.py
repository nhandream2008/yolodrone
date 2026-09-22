#!/usr/bin/env python3
"""Tiện ích telemetry dùng chung; bay tròn legacy đã bị vô hiệu hóa.

Không duy trì controller bay tròn riêng vì nó tạo một đường điều khiển Offboard
thứ hai ngoài bộ tránh vật cản LiDAR + PID. Dự án chỉ hỗ trợ `avoid` trong
PX4 SITL/Gazebo.
"""

import argparse
import asyncio


async def cho_den_khi(stream, dieu_kien, thoi_gian_cho):
    """Đọc stream telemetry đến khi điều kiện đúng rồi đóng stream an toàn."""
    async def doc_stream():
        async for gia_tri in stream:
            if dieu_kien(gia_tri):
                return gia_tri
        raise RuntimeError("Stream telemetry đã kết thúc")
    try:
        return await asyncio.wait_for(doc_stream(), thoi_gian_cho)
    finally:
        await stream.aclose()


# Alias tiếng Anh cho compatibility với controller hiện có.
wait_for = cho_den_khi


async def run(_args, _drone=None):
    """Từ chối controller legacy để không tạo pilot Offboard thứ hai."""
    raise RuntimeError(
        "Chế độ bay tròn đã bị vô hiệu hóa; dùng avoid trong PX4 SITL/Gazebo")


def parse_args():
    """Giữ parser tối thiểu để caller cũ nhận lỗi có chủ đích, không có lệnh bay."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    return argparse.Namespace()


if __name__ == "__main__":
    try:
        asyncio.run(run(parse_args()))
    except RuntimeError as loi:
        raise SystemExit(f"Dừng: {loi}") from loi
