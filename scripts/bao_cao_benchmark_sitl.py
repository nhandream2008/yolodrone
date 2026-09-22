#!/usr/bin/env python3
"""So sánh offline các log benchmark PX4 SITL/Gazebo; không điều khiển drone."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass


def gia_tri_huu_han(hang, cot):
    try:
        gia_tri = float(hang.get(cot, ""))
    except (TypeError, ValueError):
        return math.nan
    return gia_tri if math.isfinite(gia_tri) else math.nan


def mau_bay(duong_dan):
    with Path(duong_dan).open(newline="", encoding="utf-8") as luong:
        return [hang for hang in csv.DictReader(luong) if hang.get("event_type") == "SAMPLE"]


def tong_hop_mot_log(duong_dan):
    mau = mau_bay(duong_dan)
    if not mau:
        raise ValueError(f"Log không có mẫu bay: {duong_dan}")

    def cac_gia_tri(cot):
        return [gia_tri_huu_han(hang, cot) for hang in mau
                if math.isfinite(gia_tri_huu_han(hang, cot))]

    trang_thai = [hang.get("state", "") for hang in mau]
    van_toc_do = [math.hypot(gia_tri_huu_han(hang, "measured_north_m_s"),
                             gia_tri_huu_han(hang, "measured_east_m_s"))
                    for hang in mau
                    if math.isfinite(gia_tri_huu_han(hang, "measured_north_m_s"))
                    and math.isfinite(gia_tri_huu_han(hang, "measured_east_m_s"))]
    doi_ben = sum(truoc != sau and truoc.startswith("SLIDE_") and sau.startswith("SLIDE_")
                  for truoc, sau in zip(trang_thai, trang_thai[1:]))
    tong_hop = {
        "log": str(duong_dan),
        "profile": next((hang.get("speed_profile") for hang in mau if hang.get("speed_profile")), "legacy"),
        "elapsed_s": max(cac_gia_tri("elapsed_s")),
        "min_front_m": min(cac_gia_tri("front_m")),
        "min_nearest_m": min(cac_gia_tri("nearest_m")),
        "max_measured_speed_m_s": max(van_toc_do) if van_toc_do else math.nan,
        "max_pid_error_m": max(abs(gia_tri) for gia_tri in cac_gia_tri("pid_error_m")),
        "median_lidar_age_ms": statistics.median(cac_gia_tri("lidar_age_ms")),
        "mean_control_cycle_ms": (statistics.mean(cac_gia_tri("control_cycle_ms"))
                                  if cac_gia_tri("control_cycle_ms") else math.nan),
        "max_control_cycle_ms": (max(cac_gia_tri("control_cycle_ms"))
                                 if cac_gia_tri("control_cycle_ms") else math.nan),
        "final_home_m": cac_gia_tri("home_distance_m")[-1],
        "stop_samples": sum(trang_thai_mau in ("WAIT_SCAN", "BLOCKED", "BRAKE")
                            for trang_thai_mau in trang_thai),
        "side_switches": doi_ben,
    }
    sidecar = Path(duong_dan).with_suffix(".json")
    if sidecar.is_file():
        du_lieu_cuoi = json.loads(sidecar.read_text(encoding="utf-8"))
        tong_hop.update(
            result=du_lieu_cuoi.get("result", ""),
            landing_confirmed=bool(du_lieu_cuoi.get("landing_confirmed")),
            disarmed_confirmed=bool(du_lieu_cuoi.get("disarmed_confirmed")),
        )
    return tong_hop


def phan_vi(danh_sach, ty_le):
    """Nội suy percentile tuyến tính cho danh sách số hữu hạn."""
    gia_tri = sorted(gia_tri for gia_tri in danh_sach if math.isfinite(gia_tri))
    if not gia_tri:
        return None
    if len(gia_tri) == 1:
        return gia_tri[0]
    vi_tri = (len(gia_tri)-1)*ty_le
    thap = math.floor(vi_tri)
    cao = math.ceil(vi_tri)
    if thap == cao:
        return gia_tri[thap]
    return gia_tri[thap]+(gia_tri[cao]-gia_tri[thap])*(vi_tri-thap)


def trung_binh(danh_sach):
    if not danh_sach:
        raise ValueError("Cần ít nhất một log")
    truong_so = ("elapsed_s", "min_front_m", "min_nearest_m", "max_measured_speed_m_s",
                 "max_pid_error_m", "median_lidar_age_ms", "mean_control_cycle_ms",
                 "max_control_cycle_ms", "final_home_m", "stop_samples", "side_switches")
    ket_qua = {"runs": len(danh_sach), "profiles": sorted({mau["profile"] for mau in danh_sach})}
    for truong in truong_so:
        cac_gia_tri = [mau[truong] for mau in danh_sach
                        if isinstance(mau[truong], (int, float)) and math.isfinite(mau[truong])]
        ket_qua[truong] = statistics.mean(cac_gia_tri) if cac_gia_tri else None
        ket_qua[f"median_{truong}"] = statistics.median(cac_gia_tri) if cac_gia_tri else None
        ket_qua[f"p95_{truong}"] = phan_vi(cac_gia_tri, 0.95)
    ket_qua["worst_min_front_m"] = min(
        (mau["min_front_m"] for mau in danh_sach if math.isfinite(mau["min_front_m"])),
        default=None,
    )
    ket_qua["worst_control_cycle_ms"] = max(
        (mau["max_control_cycle_ms"] for mau in danh_sach if math.isfinite(mau["max_control_cycle_ms"])),
        default=None,
    )
    ket_qua["all_complete"] = all(mau.get("result") == "COMPLETE" for mau in danh_sach)
    ket_qua["all_landed_disarmed"] = all(mau.get("landing_confirmed") and mau.get("disarmed_confirmed")
                                          for mau in danh_sach)
    return ket_qua


def ty_le_thay_doi(moi, cu):
    if (not isinstance(moi, (int, float)) or not isinstance(cu, (int, float))
            or not math.isfinite(moi) or not math.isfinite(cu) or abs(cu) < 1e-9):
        return None
    return (moi-cu)/abs(cu)*100.0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", nargs="+", required=True, help="CSV baseline đã ghi")
    parser.add_argument("--fast-sitl", nargs="+", required=True, help="CSV fast_sitl đã ghi")
    parser.add_argument("--output", help="JSON tổng hợp tùy chọn")
    parser.add_argument("--min-runs", type=int, default=3,
                        help="số run hoàn tất tối thiểu cho mỗi profile (1..100)")
    args = parser.parse_args(argv)
    if not 1 <= args.min_runs <= 100:
        parser.error("--min-runs must be in 1..100")

    baseline = trung_binh([tong_hop_mot_log(duong_dan) for duong_dan in args.baseline])
    fast = trung_binh([tong_hop_mot_log(duong_dan) for duong_dan in args.fast_sitl])
    ket_qua = {
        "baseline": baseline,
        "fast_sitl": fast,
        "thay_doi_phan_tram": {
            "elapsed_s": ty_le_thay_doi(fast["elapsed_s"], baseline["elapsed_s"]),
            "min_front_m": ty_le_thay_doi(fast["min_front_m"], baseline["min_front_m"]),
            "max_measured_speed_m_s": ty_le_thay_doi(
                fast["max_measured_speed_m_s"], baseline["max_measured_speed_m_s"]),
            "max_pid_error_m": ty_le_thay_doi(fast["max_pid_error_m"], baseline["max_pid_error_m"]),
        },
        "acceptance_requirements": {
            "min_runs_per_profile": args.min_runs,
            "require_all_complete_landed_disarmed": True,
            "max_extra_stop_samples": 0,
            "max_worst_case_clearance_loss_m": 0.10,
        },
        "fast_sitl_accepted": (
            baseline["runs"] >= args.min_runs and fast["runs"] >= args.min_runs
            and fast["all_complete"] and fast["all_landed_disarmed"]
            and fast["stop_samples"] <= baseline["stop_samples"]
            and fast["worst_min_front_m"] is not None
            and baseline["worst_min_front_m"] is not None
            and fast["worst_min_front_m"] >= baseline["worst_min_front_m"] - 0.10
        ),
        "acceptance_note": (
            "Chỉ là tiêu chí benchmark SITL đa-run; không chứng nhận an toàn bay phần cứng thật."
        ),
    }
    print(json.dumps(ket_qua, ensure_ascii=False, indent=2, allow_nan=False))
    if args.output:
        Path(args.output).write_text(json.dumps(ket_qua, ensure_ascii=False, indent=2, allow_nan=False),
                                     encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
