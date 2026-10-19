#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_check_memory_layout.py -- check_memory_layout.py 单元测试

覆盖：
  - 2 KiB 向上对齐（align_up 边界）
  - 12/16 KiB Boot 的 C8 紧张窗口示例（APP_CAPACITY/A_max）
  - bin 刚好越界 1 B（边界值）与 bin 大于分区
  - map 有洞（多个 Load Region）
  - C8/CB 边界（元数据必须占 Flash 末尾两页、越界检测、页对齐）
  - VTOR 对齐（0x200 / 0x80）
  - memory_layout.h / uvprojx / map 解析与交叉校验
  - main() 端到端（退出码 + 计划 Expected output 关键行）

运行：python tools/test_check_memory_layout.py
退出码：0 全部通过，1 任一失败。
"""

import io
import os
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_memory_layout as cml


# ---------------------------------------------------------------------------
# 测试夹具：合法的 C8 临时布局头（与 CONFIG/memory_layout.h 同构）
# ---------------------------------------------------------------------------

LAYOUT_C8 = """
#ifndef __MEMORY_LAYOUT_H
#define __MEMORY_LAYOUT_H

/* 临时布局标记：1 = 分区未冻结（Task 16 才允许置 0） */
#define LAYOUT_PROVISIONAL   1

#define FLASH_BASE           0x08000000UL   /* 芯片事实 */
#define FLASH_END_EXCL       0x08010000UL
#define FLASH_PAGE_SIZE      0x00000800UL

#define META_A_BASE          0x0800F000UL   /* page 30 */
#define META_B_BASE          0x0800F800UL   /* page 31 */
#define APP_LIMIT_EXCL       0x0800F000UL

#define BOOT_PART_SIZE       0x00002000UL   /* align_up(2720 + 4096, 2048) */
#define APP_BASE             0x08002000UL
#define APP_CAPACITY         0x0000D000UL

#endif
"""


def layout_c8(**overrides):
    d = dict(cml.parse_layout_text(LAYOUT_C8))
    d.update(overrides)
    return d


def layout_cb(**overrides):
    """CB 迁移布局（设计 3.3：Flash 末端 0x08020000，元数据 0x0801F000/0x0801F800）。"""
    d = layout_c8(
        FLASH_END_EXCL=0x08020000,
        META_A_BASE=0x0801F000,
        META_B_BASE=0x0801F800,
        APP_LIMIT_EXCL=0x0801F000,
        APP_CAPACITY=0x0001D000,
    )
    d.update(overrides)
    return d


def layout_rc_dual(**overrides):
    """RCT6 双槽 A/B 布局（与 CONFIG/memory_layout.h 同构，256 KiB）。

    Boot 0x08000000+0x3800，SlotA 0x08003800+0x1DC00，
    SlotB 0x08021400+0x1DC00，元数据 page126/127。
    """
    d = layout_c8(
        LAYOUT_PROVISIONAL=0,
        FLASH_END_EXCL=0x08040000,
        META_A_BASE=0x0803F000,
        META_B_BASE=0x0803F800,
        APP_LIMIT_EXCL=0x0803F000,
        BOOT_PART_SIZE=0x00003800,
        APP_BASE=0x08003800,
        APP_CAPACITY=0x0001DC00,
        SLOT_CAPACITY=0x0001DC00,
        SLOT_A_BASE=0x08003800,
        SLOT_B_BASE=0x08021400,
    )
    d.update(overrides)
    return d


def uvprojx_text(target, irom_start, irom_size, define=""):
    return """<?xml version='1.0' encoding='UTF-8'?>
<Project>
  <Targets>
    <Target>
      <TargetName>{target}</TargetName>
      <TargetOption>
        <TargetCommonOption>
          <Cpu>IRAM(0x20000000,0xC000) IROM(0x{base:08X},0x{size:X}) CPUTYPE("Cortex-M3") CLOCK(12000000) ELITTLE</Cpu>
          <OnChipMemories>
            <IRAM>
              <StartAddress>0x20000000</StartAddress>
              <Size>0xC000</Size>
            </IRAM>
            <IROM>
              <StartAddress>0x{base:X}</StartAddress>
              <Size>0x{size:X}</Size>
            </IROM>
            <OCR_RVCT4>
              <StartAddress>0x{base:X}</StartAddress>
              <Size>0x{size:X}</Size>
            </OCR_RVCT4>
          </OnChipMemories>
        </TargetCommonOption>
        <TargetArmAds>
          <Cads>
            <VariousControls>
              <Define>{define}</Define>
            </VariousControls>
          </Cads>
        </TargetArmAds>
      </TargetOption>
    </Target>
  </Targets>
</Project>
""".format(target=target, base=irom_start, size=irom_size, define=define)


def map_text(lr_base, lr_size, total_rw, second_lr_base=None):
    lines = [
        "    Load Region LR_IROM1 (Base: 0x%08X, Size: 0x%08X, Max: 0x0000DC00, ABSOLUTE)"
        % (lr_base, lr_size),
        "    Execution Region ER_IROM1 (Exec base: 0x%08X, Load base: 0x%08X, "
        "Size: 0x0000BA4C, Max: 0x0000DC00, ABSOLUTE)" % (lr_base, lr_base),
        "    Execution Region RW_IRAM1 (Exec base: 0x20000000, Load base: 0x%08X, "
        "Size: 0x000034B8, Max: 0x00005000, ABSOLUTE)" % (lr_base + lr_size),
        "    Total RW  Size (RW Data + ZI Data)              %d (  13.18kB)" % total_rw,
    ]
    if second_lr_base is not None:
        lines.insert(
            1,
            "    Load Region LR_IROM2 (Base: 0x%08X, Size: 0x00003000, "
            "Max: 0x00003800, ABSOLUTE)" % second_lr_base,
        )
    return "\n".join(lines)


def bin_image(msp, pc, size):
    """带合法向量表头的 bin 夹具：word0=MSP，word1=Reset PC，余下补零。"""
    head = struct.pack("<II", msp, pc)
    assert size >= 8
    return head + b"\x00" * (size - 8)


# ---------------------------------------------------------------------------
# 2 KiB 向上对齐
# ---------------------------------------------------------------------------


class TestAlignUp(unittest.TestCase):
    def test_zero_stays_zero(self):
        self.assertEqual(cml.align_up(0, 1024), 0)

    def test_one_rounds_up_to_1k(self):
        self.assertEqual(cml.align_up(1, 1024), 1024)

    def test_exact_multiple_unchanged(self):
        self.assertEqual(cml.align_up(1024, 1024), 1024)
        self.assertEqual(cml.align_up(2048, 1024), 2048)

    def test_one_over_multiple_rounds_up(self):
        self.assertEqual(cml.align_up(1025, 1024), 2048)

    def test_b_pre_case(self):
        # Task 2 预审计 Boot B_pre = 2720 B，开发期临时余量 4 KiB：
        # align_up(2720 + 4096, 1024) = align_up(6816, 1024) = 7168 (0x1C00)
        self.assertEqual(cml.align_up(2720 + 4096, 1024), 7168)


# ---------------------------------------------------------------------------
# 12/16 KiB Boot 的 C8 紧张窗口
# ---------------------------------------------------------------------------


class TestTightWindow(unittest.TestCase):
    FLASH_BASE = 0x08000000
    APP_LIMIT = 0x0800F800

    def test_boot_12k(self):
        capacity, a_max = cml.tight_window(self.FLASH_BASE, self.APP_LIMIT, 0x3000)
        self.assertEqual(capacity, 51200)
        self.assertEqual(a_max, 49152)

    def test_boot_16k(self):
        capacity, a_max = cml.tight_window(self.FLASH_BASE, self.APP_LIMIT, 0x4000)
        self.assertEqual(capacity, 47104)
        self.assertEqual(a_max, 45056)

    def test_margin_is_parameter(self):
        capacity, a_max = cml.tight_window(self.FLASH_BASE, self.APP_LIMIT, 0x3000, 4096)
        self.assertEqual(a_max, 51200 - 4096)

    def test_cb_boundary(self):
        # CB 迁移后元数据上移到 0x0801F800，同样算法得到更大的窗口
        capacity, a_max = cml.tight_window(self.FLASH_BASE, 0x0801F800, 0x3000)
        self.assertEqual(capacity, 0x1C800)
        self.assertEqual(a_max, 0x1C800 - 2048)


# ---------------------------------------------------------------------------
# memory_layout.h 解析
# ---------------------------------------------------------------------------


class TestParseLayout(unittest.TestCase):
    def test_parse_all_constants(self):
        d = cml.parse_layout_text(LAYOUT_C8)
        expected = {
            "LAYOUT_PROVISIONAL": 1,
            "FLASH_BASE": 0x08000000,
            "FLASH_END_EXCL": 0x08010000,
            "FLASH_PAGE_SIZE": 0x800,
            "META_A_BASE": 0x0800F000,
            "META_B_BASE": 0x0800F800,
            "APP_LIMIT_EXCL": 0x0800F000,
            "BOOT_PART_SIZE": 0x2000,
            "APP_BASE": 0x08002000,
            "APP_CAPACITY": 0xD000,
        }
        for k, v in expected.items():
            self.assertEqual(d.get(k), v, "constant %s" % k)

    def test_expression_value_rejected(self):
        # 契约：头文件只允许字面量（表达式一律拒绝），由 checker 负责交叉校验
        text = "#define APP_BASE (FLASH_BASE + BOOT_PART_SIZE)\n"
        with self.assertRaises(cml.LayoutError):
            cml.parse_layout_text(text)

    def test_missing_constant_reported_by_validate(self):
        d = cml.parse_layout_text(LAYOUT_C8)
        del d["APP_CAPACITY"]
        errors = cml.validate_layout(d)
        self.assertTrue(any("APP_CAPACITY" in e for e in errors), errors)


# ---------------------------------------------------------------------------
# 布局内部校验：C8/CB 边界、页对齐、元数据占位、派生一致性
# ---------------------------------------------------------------------------


class TestValidateLayout(unittest.TestCase):
    def test_c8_provisional_valid(self):
        self.assertEqual(cml.validate_layout(layout_c8()), [])

    def test_cb_boundary_valid(self):
        self.assertEqual(cml.validate_layout(layout_cb()), [])

    def test_meta_not_page_aligned(self):
        errors = cml.validate_layout(layout_c8(META_A_BASE=0x0800F801))
        self.assertTrue(any("align" in e for e in errors), errors)

    def test_meta_beyond_c8_flash(self):
        # C8 边界（64 KiB）下使用 CB 的元数据地址 -> 越界
        errors = cml.validate_layout(layout_c8(META_A_BASE=0x0801F800, META_B_BASE=0x0801FC00))
        self.assertTrue(len(errors) > 0, "meta beyond flash must fail")

    def test_meta_must_occupy_last_two_pages(self):
        # 页对齐且不重叠，但不在 Flash 末尾两页 -> 失败
        errors = cml.validate_layout(
            layout_c8(
                META_A_BASE=0x0800E000,
                META_B_BASE=0x0800E800,
                APP_LIMIT_EXCL=0x0800E000,
                APP_CAPACITY=0xC000,
            )
        )
        self.assertTrue(any("last" in e or "末尾" in e for e in errors), errors)

    def test_meta_overlap(self):
        errors = cml.validate_layout(layout_c8(META_B_BASE=0x0800F000))
        self.assertTrue(any("overlap" in e or "重叠" in e for e in errors), errors)

    def test_boot_part_not_page_aligned(self):
        errors = cml.validate_layout(
            layout_c8(BOOT_PART_SIZE=0x2001, APP_BASE=0x08002001)
        )
        self.assertTrue(any("align" in e for e in errors), errors)

    def test_app_base_mismatch(self):
        errors = cml.validate_layout(layout_c8(APP_BASE=0x08002800))
        self.assertTrue(any("APP_BASE" in e for e in errors), errors)

    def test_app_capacity_mismatch(self):
        errors = cml.validate_layout(layout_c8(APP_CAPACITY=0xC000))
        self.assertTrue(any("APP_CAPACITY" in e for e in errors), errors)

    def test_app_limit_must_equal_meta_a(self):
        errors = cml.validate_layout(layout_c8(APP_LIMIT_EXCL=0x0800E000))
        self.assertTrue(any("APP_LIMIT_EXCL" in e for e in errors), errors)

    def test_flash_page_size_must_be_2k(self):
        # 0x400 是旧（中容量）错误值，必须被拒
        errors = cml.validate_layout(layout_c8(FLASH_PAGE_SIZE=0x400))
        self.assertTrue(any("FLASH_PAGE_SIZE" in e for e in errors), errors)

    def test_flash_span_not_page_multiple(self):
        errors = cml.validate_layout(layout_c8(FLASH_END_EXCL=0x08010001))
        self.assertTrue(len(errors) > 0)

    def test_layout_provisional_only_zero_or_one(self):
        self.assertEqual(cml.validate_layout(layout_c8(LAYOUT_PROVISIONAL=0)), [])
        errors = cml.validate_layout(layout_c8(LAYOUT_PROVISIONAL=2))
        self.assertTrue(any("LAYOUT_PROVISIONAL" in e for e in errors), errors)

    def test_boot_collides_with_meta(self):
        errors = cml.validate_layout(
            layout_c8(BOOT_PART_SIZE=0xF800, APP_BASE=0x0800F800, APP_CAPACITY=0)
        )
        self.assertTrue(len(errors) > 0)


# ---------------------------------------------------------------------------
# 双槽 A/B 布局校验（RCT6 256 KiB）
# ---------------------------------------------------------------------------


class TestValidateDualSlot(unittest.TestCase):
    def test_rc_dual_valid(self):
        self.assertEqual(cml.validate_layout(layout_rc_dual()), [])

    def test_slot_b_not_contiguous(self):
        errors = cml.validate_layout(layout_rc_dual(SLOT_B_BASE=0x08022000))
        self.assertTrue(any("SLOT_B_BASE" in e for e in errors), errors)

    def test_slot_b_end_must_equal_app_limit(self):
        # slot B 末尾必须精确等于 APP_LIMIT_EXCL
        errors = cml.validate_layout(
            layout_rc_dual(SLOT_B_BASE=0x08021000)
        )
        self.assertTrue(any("slot B end" in e or "SLOT_B_BASE" in e for e in errors), errors)

    def test_slot_capacity_must_match_app_capacity(self):
        errors = cml.validate_layout(
            layout_rc_dual(APP_CAPACITY=0x0001D000)
        )
        self.assertTrue(any("APP_CAPACITY" in e for e in errors), errors)

    def test_app_base_alias_must_equal_slot_a(self):
        errors = cml.validate_layout(layout_rc_dual(APP_BASE=0x08004000))
        self.assertTrue(any("APP_BASE" in e for e in errors), errors)

    def test_slot_capacity_need_not_be_page_multiple(self):
        # SLOT_CAPACITY 非擦除单元：合法布局的 0x1DC00 本身就不是 2 KiB 倍数
        #（0x1DC00 % 0x800 == 0x400），校验必须通过——旧规则会在此误报。
        d = layout_rc_dual()
        self.assertEqual(d["SLOT_CAPACITY"] % cml.FLASH_PAGE_SIZE_HI, 0x400)
        self.assertEqual(cml.validate_layout(d), [])

    def test_slot_capacity_keeps_slot_b_end_exact(self):
        # 但容量一动，slot B 末尾就不再精确等于 APP_LIMIT_EXCL -> 必须报错
        errors = cml.validate_layout(layout_rc_dual(SLOT_CAPACITY=0x0001E001))
        self.assertTrue(any("slot B end" in e for e in errors), errors)

    def test_slot_vtor_alignment(self):
        # Slot B 偏移 0x21400 是 0x200 倍数；改成非倍数必须失败
        errors = cml.validate_layout(
            layout_rc_dual(
                SLOT_B_BASE=0x08021900,
                APP_LIMIT_EXCL=0x0803F900,
                META_A_BASE=0x0803F900,
                META_B_BASE=0x0803FD00,
            )
        )
        # 至少 VTOR 对齐 / 页对齐 / 末页 之一必须报错
        self.assertTrue(len(errors) > 0)

    def test_partial_slot_constants_incomplete(self):
        d = layout_rc_dual()
        del d["SLOT_B_BASE"]
        errors = cml.validate_layout(d)
        self.assertTrue(any("dual-slot layout incomplete" in e for e in errors), errors)

    def test_app_vtor_allows_slot_b(self):
        # App Target 的 VECT_TAB_OFFSET 允许 SlotA(0x3800) 或 SlotB(0x21400)
        tgt = {"irom_base": 0x08003800, "irom_size": 0x1DC00,
               "iram_base": 0x20000000, "iram_size": 0xC000,
               "cpu_irom_base": 0x08003800, "cpu_irom_size": 0x1DC00,
               "defines": {"VECT_TAB_OFFSET": "0x00021400"}}
        self.assertEqual(cml.validate_target(layout_rc_dual(), tgt, "app"), [])


# ---------------------------------------------------------------------------
# VTOR 对齐
# ---------------------------------------------------------------------------


class TestVtorAlignment(unittest.TestCase):
    BASE = 0x08000000

    def test_provisional_offset_aligned(self):
        self.assertEqual(cml.check_vtor_alignment(0x08001C00, self.BASE), [])

    def test_offset_0x200_ok(self):
        self.assertEqual(cml.check_vtor_alignment(0x08000200, self.BASE), [])

    def test_offset_0x180_misaligned(self):
        self.assertNotEqual(cml.check_vtor_alignment(0x08000180, self.BASE), [])

    def test_offset_0x100_misaligned(self):
        self.assertNotEqual(cml.check_vtor_alignment(0x08000100, self.BASE), [])

    def test_offset_0x80_misaligned(self):
        # 0x80 只满足 VTOR 最低对齐的一半条件（不满足 0x200）
        self.assertNotEqual(cml.check_vtor_alignment(0x08000080, self.BASE), [])


# ---------------------------------------------------------------------------
# uvprojx 解析与 Target 交叉校验
# ---------------------------------------------------------------------------


class TestUvprojx(unittest.TestCase):
    def setUp(self):
        self.layout = layout_c8()

    def test_parse_app_target(self):
        t = cml.parse_uvprojx_text(
            uvprojx_text(
                "App_C8_Provisional",
                0x08002000,
                0xD000,
                "STM32F10X_MD,USE_STDPERIPH_DRIVER,VECT_TAB_OFFSET=0x00002000",
            )
        )
        self.assertEqual(t["target_name"], "App_C8_Provisional")
        self.assertEqual(t["irom_base"], 0x08002000)
        self.assertEqual(t["irom_size"], 0xD000)
        self.assertEqual(t["iram_base"], 0x20000000)
        self.assertEqual(t["iram_size"], 0xC000)
        self.assertEqual(t["defines"].get("VECT_TAB_OFFSET"), "0x00002000")

    def test_validate_app_target_ok(self):
        t = cml.parse_uvprojx_text(
            uvprojx_text(
                "App_C8_Provisional",
                0x08002000,
                0xD000,
                "STM32F10X_MD,USE_STDPERIPH_DRIVER,VECT_TAB_OFFSET=0x00002000",
            )
        )
        self.assertEqual(cml.validate_target(self.layout, t, "app"), [])

    def test_app_target_wrong_irom_size(self):
        t = cml.parse_uvprojx_text(
            uvprojx_text("App_C8_Provisional", 0x08002000, 0xC000,
                         "STM32F10X_MD,VECT_TAB_OFFSET=0x00002000")
        )
        errors = cml.validate_target(self.layout, t, "app")
        self.assertTrue(any("IROM" in e for e in errors), errors)

    def test_app_target_missing_vect_tab_offset(self):
        t = cml.parse_uvprojx_text(uvprojx_text("App_C8_Provisional", 0x08002000, 0xD000))
        errors = cml.validate_target(self.layout, t, "app")
        self.assertTrue(any("VECT_TAB_OFFSET" in e for e in errors), errors)

    def test_app_target_wrong_vect_tab_offset(self):
        t = cml.parse_uvprojx_text(
            uvprojx_text("App_C8_Provisional", 0x08002000, 0xD000, "VECT_TAB_OFFSET=0x0")
        )
        errors = cml.validate_target(self.layout, t, "app")
        self.assertTrue(any("VECT_TAB_OFFSET" in e for e in errors), errors)

    def test_boot_target_ok(self):
        t = cml.parse_uvprojx_text(uvprojx_text("Boot_C8_Provisional", 0x08000000, 0x2000))
        self.assertEqual(cml.validate_target(self.layout, t, "boot"), [])

    def test_boot_target_wrong_base(self):
        t = cml.parse_uvprojx_text(uvprojx_text("Boot_C8_Provisional", 0x08000800, 0x2000))
        errors = cml.validate_target(self.layout, t, "boot")
        self.assertTrue(any("IROM" in e for e in errors), errors)

    def test_boot_target_rejects_nonzero_vtor(self):
        t = cml.parse_uvprojx_text(
            uvprojx_text("Boot_C8_Provisional", 0x08000000, 0x2000, "VECT_TAB_OFFSET=0x2000")
        )
        errors = cml.validate_target(self.layout, t, "boot")
        self.assertTrue(any("VECT_TAB_OFFSET" in e for e in errors), errors)

    def test_cpu_string_disagrees_with_irom_block(self):
        text = uvprojx_text("Boot_C8_Provisional", 0x08000000, 0x2000)
        text = text.replace("IROM(0x08000000,0x2000)", "IROM(0x08000000,0x3000)")
        t = cml.parse_uvprojx_text(text)
        errors = cml.validate_target(self.layout, t, "boot")
        self.assertTrue(any("Cpu" in e or "IROM" in e for e in errors), errors)


# ---------------------------------------------------------------------------
# map 解析与校验（含“有洞”）
# ---------------------------------------------------------------------------


class TestMap(unittest.TestCase):
    def setUp(self):
        self.layout = layout_c8()

    def test_parse_single_load_region(self):
        m = cml.parse_map_text(map_text(0x08001C00, 0xBA90, 13496))
        self.assertEqual(m["lr_count"], 1)
        self.assertEqual(m["lr_base"], 0x08001C00)
        self.assertEqual(m["lr_size"], 0xBA90)
        self.assertEqual(m["er_load_base"], 0x08001C00)
        self.assertEqual(m["total_rw"], 13496)

    def test_map_with_hole_fails(self):
        # 两个 Load Region = 分区内有洞（IAP 流式写入无法覆盖）
        m = cml.parse_map_text(map_text(0x08001C00, 0x5000, 13496, second_lr_base=0x0800C000))
        self.assertEqual(m["lr_count"], 2)
        errors = cml.validate_map(self.layout, m, "app")
        self.assertTrue(any("load region" in e for e in errors), errors)

    def test_app_map_wrong_base(self):
        m = cml.parse_map_text(map_text(0x08000000, 0xBA90, 13496))
        errors = cml.validate_map(self.layout, m, "app")
        self.assertTrue(any("base" in e for e in errors), errors)

    def test_app_map_exceeds_partition(self):
        # 0x08002000 + 0xDE00 = 0x0800FE00 > APP_LIMIT_EXCL(0x0800F000)
        m = cml.parse_map_text(map_text(0x08002000, 0xDE00, 13496))
        errors = cml.validate_map(self.layout, m, "app")
        self.assertTrue(any("APP_LIMIT" in e or "exceed" in e or "越界" in e for e in errors), errors)

    def test_boot_map_ok(self):
        m = cml.parse_map_text(map_text(0x08000000, 0xAA0, 1792))
        self.assertEqual(cml.validate_map(self.layout, m, "boot"), [])

    def test_boot_map_exceeds_partition(self):
        # 0x08000000 + 0x2800 > FLASH_BASE + BOOT_PART_SIZE(0x2000)
        m = cml.parse_map_text(map_text(0x08000000, 0x2800, 1792))
        errors = cml.validate_map(self.layout, m, "boot")
        self.assertTrue(len(errors) > 0, errors)

    def test_missing_total_rw_fails_ram_check(self):
        m = cml.parse_map_text(map_text(0x08001C00, 0xBA90, 13496))
        del m["total_rw"]
        errors = cml.check_ram_budget("app", m.get("total_rw"), cml.APP_RAM_BUDGET)
        self.assertTrue(len(errors) > 0, errors)


# ---------------------------------------------------------------------------
# bin 边界：刚好越界 1 B / 大于分区
# ---------------------------------------------------------------------------


class TestBinFits(unittest.TestCase):
    CAPACITY = 0xDC00  # 56320

    def test_exact_fit_passes(self):
        self.assertEqual(cml.check_bin_fits("app", self.CAPACITY, self.CAPACITY), [])

    def test_one_byte_over_fails(self):
        # 计划要求：刚好越界 1 B 必须被检出
        errors = cml.check_bin_fits("app", self.CAPACITY + 1, self.CAPACITY)
        self.assertTrue(len(errors) > 0, errors)

    def test_bin_far_over_capacity(self):
        errors = cml.check_bin_fits("app", self.CAPACITY + 5000, self.CAPACITY)
        self.assertTrue(len(errors) > 0, errors)

    def test_empty_bin_fails(self):
        self.assertTrue(len(cml.check_bin_fits("app", 0, self.CAPACITY)) > 0)

    def test_boot_bin_boundary(self):
        # Boot 分区 0x1C00 = 7168：2720 通过，7169 失败
        self.assertEqual(cml.check_bin_fits("boot", 2720, 0x1C00), [])
        self.assertTrue(len(cml.check_bin_fits("boot", 0x1C00 + 1, 0x1C00)) > 0)


# ---------------------------------------------------------------------------
# RAM 预算（设计 §4：APP <= 18432 B，Boot <= 4096 B）
# ---------------------------------------------------------------------------


class TestRamBudget(unittest.TestCase):
    def test_app_budget_boundary(self):
        self.assertEqual(cml.check_ram_budget("app", 18432, cml.APP_RAM_BUDGET), [])
        self.assertTrue(len(cml.check_ram_budget("app", 18433, cml.APP_RAM_BUDGET)) > 0)

    def test_boot_budget_boundary(self):
        self.assertEqual(cml.check_ram_budget("boot", 4096, cml.BOOT_RAM_BUDGET), [])
        self.assertTrue(len(cml.check_ram_budget("boot", 4097, cml.BOOT_RAM_BUDGET)) > 0)


# ---------------------------------------------------------------------------
# 向量表 MSP/PC（Task 16 脚本核对：bin 前 8 B）
# ---------------------------------------------------------------------------


class TestVectorTable(unittest.TestCase):
    def test_valid_boot_vector(self):
        data = struct.pack("<II", 0x20005000, 0x08000131)
        self.assertEqual(cml.validate_vector_table(data, 0x08000000, 0x08001C00), [])

    def test_valid_app_vector(self):
        data = struct.pack("<II", 0x20005000, 0x08001D31)
        self.assertEqual(cml.validate_vector_table(data, 0x08001C00, 0x0800F800), [])

    def test_msp_outside_sram_fails(self):
        # 0x2000D000> 0x20000000 + 48 KiB，越界
        data = struct.pack("<II", 0x2000D000, 0x08000131)
        errors = cml.validate_vector_table(data, 0x08000000, 0x08001C00)
        self.assertTrue(any("MSP" in e and "SRAM" in e for e in errors), errors)

    def test_msp_not_word_aligned_fails(self):
        data = struct.pack("<II", 0x20005002, 0x08000131)
        errors = cml.validate_vector_table(data, 0x08000000, 0x08001C00)
        self.assertTrue(any("aligned" in e for e in errors), errors)

    def test_pc_missing_thumb_bit_fails(self):
        data = struct.pack("<II", 0x20005000, 0x08000130)
        errors = cml.validate_vector_table(data, 0x08000000, 0x08001C00)
        self.assertTrue(any("Thumb" in e for e in errors), errors)

    def test_pc_outside_image_fails(self):
        # Thumb 位正确，但入口落在 Boot 分区之外（越过 APP_BASE）
        data = struct.pack("<II", 0x20005000, 0x08002001)
        errors = cml.validate_vector_table(data, 0x08000000, 0x08001C00)
        self.assertTrue(any("outside image" in e for e in errors), errors)

    def test_short_data_fails(self):
        self.assertTrue(len(cml.validate_vector_table(b"\x00" * 4, 0x08000000, 0x1C00)) > 0)
        self.assertTrue(len(cml.validate_vector_table(None, 0x08000000, 0x1C00)) > 0)


# ---------------------------------------------------------------------------
# Flash 镜像区间：Boot/App 不重叠、都不侵入元数据两页（Task 16 脚本核对）
# ---------------------------------------------------------------------------


class TestFlashIntervals(unittest.TestCase):
    META = 0x0800F800
    FLASH_END = 0x08010000

    def test_disjoint_images_pass(self):
        self.assertEqual(
            cml.validate_flash_intervals(0x08000000, 0xAA0, 0x08001C00, 0xBA90,
                                         self.META, self.FLASH_END),
            [])

    def test_boot_app_overlap_fails(self):
        # Boot 镜像尾 0x08002000 > APP_BASE 0x08001C00
        errors = cml.validate_flash_intervals(0x08000000, 0x2000, 0x08001C00, 0xBA90,
                                              self.META, self.FLASH_END)
        self.assertTrue(any("overlaps app" in e for e in errors), errors)

    def test_app_touching_meta_start_passes(self):
        # 镜像尾恰好落在 META_A_BASE：[x, 0x0800F800) 与 [0x0800F800, end) 不相交
        self.assertEqual(
            cml.validate_flash_intervals(0x08000000, 0xAA0, 0x08001C00, 0xDC00,
                                         self.META, self.FLASH_END),
            [])

    def test_app_into_meta_pages_fails(self):
        errors = cml.validate_flash_intervals(0x08000000, 0xAA0, 0x08001C00, 0xDC01,
                                              self.META, self.FLASH_END)
        self.assertTrue(any("metadata" in e for e in errors), errors)

    def test_boot_into_meta_pages_fails(self):
        errors = cml.validate_flash_intervals(0x08000000, 0xF801, 0x08001C00, 0xBA90,
                                              self.META, self.FLASH_END)
        self.assertTrue(any("metadata" in e for e in errors), errors)


# ---------------------------------------------------------------------------
# bin 半字对齐（Boot 半字编程约束）
# ---------------------------------------------------------------------------


class TestHalfwordAlign(unittest.TestCase):
    def test_even_passes(self):
        self.assertEqual(cml.check_halfword_aligned("boot", 11464), [])
        self.assertEqual(cml.check_halfword_aligned("app", 54620), [])

    def test_odd_fails(self):
        self.assertTrue(len(cml.check_halfword_aligned("boot", 11465)) > 0)

    def test_missing_fails(self):
        self.assertTrue(len(cml.check_halfword_aligned("boot", None)) > 0)


# ---------------------------------------------------------------------------
# main() 端到端
# ---------------------------------------------------------------------------


class TestMain(unittest.TestCase):
    def setUp(self):
        self.layout = layout_c8()

    def _write_tree(self, tmp, app_bin_size=47760, app_audit_bin_size=47760,
                    provisional=1):
        paths = {}
        layout_text = LAYOUT_C8
        if provisional == 0:
            layout_text = layout_text.replace(
                "#define LAYOUT_PROVISIONAL   1",
                "#define LAYOUT_PROVISIONAL   0")
        specs = [
            ("layout.h", layout_text, "w"),
            (
                "boot.uvprojx",
                uvprojx_text("Boot_C8_Provisional", 0x08000000, 0x2000),
                "w",
            ),
            (
                "app.uvprojx",
                uvprojx_text(
                    "App_C8_Provisional",
                    0x08002000,
                    0xD000,
                    "STM32F10X_MD,USE_STDPERIPH_DRIVER,VECT_TAB_OFFSET=0x00002000",
                ),
                "w",
            ),
            ("boot.map", map_text(0x08000000, 0xAA0, 1792), "w"),
            ("app.map", map_text(0x08002000, 0xBA90, 13496), "w"),
            # 向量表：MSP=SRAM 顶 0x20005000；PC 带 Thumb 位且落在本镜像分区内
            ("boot.bin", bin_image(0x20005000, 0x08000131, 2720), "b"),
            ("app.bin", bin_image(0x20005000, 0x08002131, app_bin_size), "b"),
            ("app_audit.bin", b"\x00" * app_audit_bin_size, "b"),
        ]
        for name, content, mode in specs:
            path = os.path.join(tmp, name)
            if mode == "b":
                with open(path, "wb") as fh:
                    fh.write(content)
            else:
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write(content)
            paths[name] = path
        return paths

    def _argv(self, p, extra=()):
        return [
            "--layout", p["layout.h"],
            "--boot-proj", p["boot.uvprojx"],
            "--boot-map", p["boot.map"],
            "--boot-bin", p["boot.bin"],
            "--app-proj", p["app.uvprojx"],
            "--app-map", p["app.map"],
            "--app-bin", p["app.bin"],
            "--app-audit-bin", p["app_audit.bin"],
        ] + list(extra)

    def test_main_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(self._argv(p))
            out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("layout: PROVISIONAL", out)
        self.assertIn("flash_end_exclusive: 0x08010000", out)
        self.assertIn("layout_provisional: 1", out)
        self.assertIn("page aligned, non-overlapping", out)
        self.assertIn("current capacity warning: PASS", out)

    def test_main_bin_one_byte_over_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp, app_bin_size=0xD000 + 1)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(self._argv(p))
            out = buf.getvalue()
        self.assertEqual(rc, 1)
        self.assertIn("FAIL", out)

    def test_main_audit_bin_over_capacity_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp, app_audit_bin_size=0xD000 + 1)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(self._argv(p))
            out = buf.getvalue()
        self.assertEqual(rc, 1)
        self.assertIn("current capacity warning: FAIL", out)

    def test_main_missing_file_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp)
            argv = self._argv(p)
            argv[argv.index(p["app.map"])] = os.path.join(tmp, "nope.map")
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(argv)
        self.assertEqual(rc, 1)

    def test_main_reports_vector_and_interval_lines(self):
        # Task 16 脚本核对项在报告中有独立输出行
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(self._argv(p))
            out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("boot vector", out)
        self.assertIn("app vector", out)
        self.assertIn("flash intervals", out)
        self.assertIn("halfword aligned", out)

    def test_main_final_gate_fails_on_provisional(self):
        # --final 下 LAYOUT_PROVISIONAL=1 必须失败
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp, provisional=1)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(self._argv(p, ["--final"]))
            out = buf.getvalue()
        self.assertEqual(rc, 1)
        self.assertIn("LAYOUT_PROVISIONAL must be 0", out)

    def test_main_final_gate_passes_on_frozen(self):
        # --final 且 LAYOUT_PROVISIONAL=0：全绿
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp, provisional=0)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(self._argv(p, ["--final"]))
            out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("final gate", out)
        self.assertIn("layout: FINAL", out)

    def test_main_vector_check_catches_bad_msp(self):
        # MSP 写坏（0x08000000 不在 SRAM）必须整体失败
        with tempfile.TemporaryDirectory() as tmp:
            p = self._write_tree(tmp)
            bad = bin_image(0x08000000, 0x08000131, 2720)
            with open(p["boot.bin"], "wb") as fh:
                fh.write(bad)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = cml.main(self._argv(p))
            out = buf.getvalue()
        self.assertEqual(rc, 1)
        self.assertIn("FAIL", out)


if __name__ == "__main__":
    unittest.main()
