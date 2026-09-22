"""Profile tốc độ có ràng buộc cho PX4 SITL/Gazebo.

Các profile không thay thế planner LiDAR/PID. Chúng chỉ đặt giới hạn đầu vào;
quãng phản ứng, quãng phanh, scan age và khả năng lách ngang vẫn có quyền hạ
lệnh vận tốc ở mỗi chu kỳ.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class HoSoTocDo:
    """Giới hạn đầu vào của một profile mô phỏng."""
    ten: str
    chi_duoc_mo_phong: bool
    van_toc_hanh_trinh: float
    gia_toc_lenh_toi_da: float
    van_toc_ngang_toi_da: float
    giam_toc_phanh: float
    van_toc_tien_toi_thieu: float
    nguong_tuoi_scan_fallback_ms: float
    mo_ta: str


CAC_HO_SO_TOC_DO = {
    "conservative": HoSoTocDo(
        ten="conservative",
        chi_duoc_mo_phong=False,
        van_toc_hanh_trinh=2.0,
        gia_toc_lenh_toi_da=0.8,
        van_toc_ngang_toi_da=1.4,
        giam_toc_phanh=1.2,
        van_toc_tien_toi_thieu=0.8,
        nguong_tuoi_scan_fallback_ms=150.0,
        mo_ta="Biên thận trọng cho kiểm tra cảm biến và fault injection.",
    ),
    "baseline": HoSoTocDo(
        ten="baseline",
        chi_duoc_mo_phong=False,
        van_toc_hanh_trinh=2.5,
        gia_toc_lenh_toi_da=1.0,
        van_toc_ngang_toi_da=1.8,
        giam_toc_phanh=1.5,
        van_toc_tien_toi_thieu=1.2,
        nguong_tuoi_scan_fallback_ms=150.0,
        mo_ta="Cấu hình chuẩn đã được dùng để đo baseline SITL.",
    ),
    "fast_sitl": HoSoTocDo(
        ten="fast_sitl",
        chi_duoc_mo_phong=True,
        van_toc_hanh_trinh=3.0,
        gia_toc_lenh_toi_da=1.5,
        van_toc_ngang_toi_da=1.8,
        giam_toc_phanh=1.7,
        van_toc_tien_toi_thieu=1.2,
        nguong_tuoi_scan_fallback_ms=100.0,
        mo_ta="Profile so sánh hiệu năng chỉ trong PX4 SITL/Gazebo.",
    ),
}


def lay_ho_so_toc_do(ten_ho_so):
    """Trả profile đã khai báo hoặc báo lỗi rõ ràng."""
    try:
        return CAC_HO_SO_TOC_DO[ten_ho_so]
    except KeyError as loi:
        hop_le = ", ".join(sorted(CAC_HO_SO_TOC_DO))
        raise ValueError(f"Profile tốc độ không hợp lệ: {ten_ho_so}; chọn: {hop_le}") from loi


def ap_dung_ho_so_toc_do(tham_so, ten_ho_so):
    """Đặt các giới hạn profile vào namespace nội bộ của controller."""
    ho_so = lay_ho_so_toc_do(ten_ho_so)
    tham_so.speed = ho_so.van_toc_hanh_trinh
    tham_so.max_accel = ho_so.gia_toc_lenh_toi_da
    tham_so.max_lateral_speed = ho_so.van_toc_ngang_toi_da
    tham_so.brake_accel = ho_so.giam_toc_phanh
    tham_so.min_forward_speed = ho_so.van_toc_tien_toi_thieu
    tham_so.profile = ho_so.ten
    return ho_so


def tinh_gioi_han_toc_do_ho_so(ho_so, tuoi_scan_ms):
    """Trả cap vận tốc và cờ fallback khi sensor chậm hơn ngưỡng profile.

    Chỉ `fast_sitl` hạ về cap baseline tự động. Scan mất/quá cũ vẫn do
    planner fail-closed xử lý bằng lệnh zero và hủy nhiệm vụ, không được che
    bởi fallback tốc độ.
    """
    tuoi_khong_hop_le = not isinstance(tuoi_scan_ms, (int, float)) or tuoi_scan_ms < 0
    if (ho_so.ten == "fast_sitl"
            and (tuoi_khong_hop_le or tuoi_scan_ms > ho_so.nguong_tuoi_scan_fallback_ms)):
        return CAC_HO_SO_TOC_DO["baseline"].van_toc_hanh_trinh, True
    return ho_so.van_toc_hanh_trinh, False
