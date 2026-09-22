"""Regression tests for constrained SITL speed profiles."""

from types import SimpleNamespace
import unittest

from cau_hinh_toc_do import (
    CAC_HO_SO_TOC_DO, ap_dung_ho_so_toc_do, lay_ho_so_toc_do,
    tinh_gioi_han_toc_do_ho_so,
)


class CauHinhTocDoTests(unittest.TestCase):
    def test_all_profiles_stay_inside_existing_hard_caps(self):
        for ten, ho_so in CAC_HO_SO_TOC_DO.items():
            with self.subTest(profile=ten):
                self.assertLessEqual(ho_so.van_toc_hanh_trinh, 3.0)
                self.assertLessEqual(ho_so.gia_toc_lenh_toi_da, 2.0)
                self.assertLessEqual(ho_so.van_toc_ngang_toi_da, 2.0)
                self.assertLessEqual(ho_so.giam_toc_phanh, 2.0)
                self.assertGreaterEqual(ho_so.van_toc_tien_toi_thieu, 0.5)
                self.assertLessEqual(ho_so.van_toc_tien_toi_thieu, ho_so.van_toc_hanh_trinh)

    def test_fast_sitl_is_explicitly_simulation_only(self):
        self.assertTrue(lay_ho_so_toc_do("fast_sitl").chi_duoc_mo_phong)
        self.assertFalse(lay_ho_so_toc_do("baseline").chi_duoc_mo_phong)

    def test_apply_profile_updates_only_constrained_limits(self):
        tham_so = SimpleNamespace()
        ho_so = ap_dung_ho_so_toc_do(tham_so, "fast_sitl")
        self.assertEqual(tham_so.profile, "fast_sitl")
        self.assertEqual(tham_so.speed, ho_so.van_toc_hanh_trinh)
        self.assertEqual(tham_so.max_accel, ho_so.gia_toc_lenh_toi_da)
        self.assertEqual(tham_so.max_lateral_speed, ho_so.van_toc_ngang_toi_da)
        self.assertEqual(tham_so.brake_accel, ho_so.giam_toc_phanh)
        self.assertEqual(tham_so.min_forward_speed, ho_so.van_toc_tien_toi_thieu)

    def test_fast_sitl_falls_back_when_valid_scan_latency_is_high(self):
        fast = lay_ho_so_toc_do("fast_sitl")
        cap_fresh, fallback_fresh = tinh_gioi_han_toc_do_ho_so(fast, 60.0)
        cap_delayed, fallback_delayed = tinh_gioi_han_toc_do_ho_so(fast, 120.0)
        cap_invalid, fallback_invalid = tinh_gioi_han_toc_do_ho_so(fast, -1.0)
        self.assertEqual(cap_fresh, 3.0)
        self.assertFalse(fallback_fresh)
        self.assertEqual(cap_delayed, 2.5)
        self.assertTrue(fallback_delayed)
        self.assertEqual(cap_invalid, 2.5)
        self.assertTrue(fallback_invalid)

    def test_baseline_never_uses_fast_profile_fallback(self):
        cap, fallback = tinh_gioi_han_toc_do_ho_so(lay_ho_so_toc_do("baseline"), 200.0)
        self.assertEqual(cap, 2.5)
        self.assertFalse(fallback)

    def test_unknown_profile_fails_with_clear_reason(self):
        with self.assertRaisesRegex(ValueError, "Profile tốc độ không hợp lệ"):
            lay_ho_so_toc_do("khong_co")


if __name__ == "__main__":
    unittest.main()
