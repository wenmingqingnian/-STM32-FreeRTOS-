#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_memory_layout.py -- Flash 分区布局一致性校验

职责：
  1. 解析 CONFIG/memory_layout.h 的分区常量，并做内部一致性校验
（C8/CB 边界、页对齐、元数据双页必须占 Flash 末尾两页、
      BOOT_PART_SIZE 页对齐、派生关系 APP_BASE/APP_CAPACITY、VTOR 对齐）；
  2. 解析 Boot/App Target 的 uvprojx，校验 IROM/IRAM/VECT_TAB_OFFSET
     与 memory_layout.h 完全一致；
  3. 解析 Target 构建产物 map/bin，校验加载域基址、分区不越界、
     单一加载域（map 有洞即失败）、bin 长度不超容量、
     静态 RAM 预算（APP <= 18432 B，Boot <= 4096 B，设计 §4）；
  4. 用 App_SizeAudit.bin 做"当前容量预警"（只预警，不得据此认定最终通过）；
  5. Task 16 冻结期镜像级校验：bin 前 8 B 向量表（MSP 落 SRAM 且 4 B 对齐、
     PC 带 Thumb 位且落在本镜像分区内）、Boot/App 镜像 Flash 区间互不重叠
     且不侵入元数据两页、bin 长度半字对齐；
  6. --final 门禁：LAYOUT_PROVISIONAL 必须为 0（布局未冻结即失败）。

任何地址/长度不一致退出码为 1；全部通过退出码为 0。
单元测试：tools/test_check_memory_layout.py（python tools/test_check_memory_layout.py）

用法（仓库根目录，Task 16 冻结验证）：
  python tools/check_memory_layout.py ^
    --layout CONFIG/memory_layout.h ^
    --boot-proj USER/boot_cb_provisional.uvprojx ^
    --boot-map OBJ_BOOT_PROV/boot.map --boot-bin OBJ_BOOT_PROV/boot.bin ^
    --app-proj USER/app_cb_provisional.uvprojx ^
    --app-map OBJ_APP_PROV/range_hood_app.map ^
    --app-bin OBJ_APP_PROV/range_hood_app.bin ^
    --app-audit-bin OBJ_APP_AUDIT/range_hood_app.bin ^
    --final
"""

import argparse
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

# ---------------------------------------------------------------------------
# 设计约束常量（分区与对齐规则见 CONFIG/memory_layout.h）
# ---------------------------------------------------------------------------

FLASH_PAGE_SIZE_HI = 0x800          # 高密度 F1 页大小（RCT6 256 KiB；board_profile.h 冻结）
VTOR_ALIGN_SYS = 0x200              # system_stm32f10x.c 对 VECT_TAB_OFFSET 的要求
VTOR_ALIGN_MIN = 0x80               # Cortex-M3 SCB->VTOR 最低对齐
APP_RAM_BUDGET = 18432              # 设计 §4：APP Total RW <= 18 KiB
BOOT_RAM_BUDGET = 4096              # 设计 §4：Boot Total RW <= 4 KiB
SRAM_BASE = 0x20000000              # 两个镜像不同时执行，都可独占整片 SRAM
SRAM_SIZE = 0xC000                   # RCT6 = 48 KiB（board_profile.h BOARD_SRAM_SIZE_BYTES）
HARD_MARGIN = 2048                  # 冻结时的 2 KiB 硬余量（设计 §3.2）

REQUIRED_LAYOUT_CONSTANTS = (
    "LAYOUT_PROVISIONAL",
    "FLASH_BASE",
    "FLASH_END_EXCL",
    "FLASH_PAGE_SIZE",
    "META_A_BASE",
    "META_B_BASE",
    "APP_LIMIT_EXCL",
    "BOOT_PART_SIZE",
    "APP_BASE",
    "APP_CAPACITY",
)

# 双槽布局可选常量（存在即进入双槽校验模式；缺任一即按单槽模式校验）
DUAL_SLOT_CONSTANTS = (
    "SLOT_CAPACITY",
    "SLOT_A_BASE",
    "SLOT_B_BASE",
)

# 已知合法的 Flash 跨度（C8=64 KiB / CB=128 KiB / RC=256 KiB / RE=512 KiB）
KNOWN_FLASH_SPANS = (0x10000, 0x20000, 0x40000, 0x80000)


class LayoutError(Exception):
    """memory_layout.h 解析失败（缺常量、非法字面量等）。"""


# ---------------------------------------------------------------------------
# 纯逻辑：对齐与紧张窗口
# ---------------------------------------------------------------------------


def align_up(value, alignment):
    """向上取整（如 Boot 分区按 2 KiB 页对齐；alignment 必须为正）。"""
    return ((value + alignment - 1) // alignment) * alignment


def tight_window(flash_base, app_limit_excl, boot_part_size, hard_margin=HARD_MARGIN):
    """给定 Boot 分区大小，返回 (APP_CAPACITY, 扣硬余量后的 A_max)。：
      boot 12 KiB -> APP_CAPACITY 51200 B，A_max 49152 B
      boot 16 KiB -> APP_CAPACITY 47104 B，A_max 45056 B
    """
    capacity = app_limit_excl - (flash_base + boot_part_size)
    return capacity, capacity - hard_margin


# ---------------------------------------------------------------------------
# memory_layout.h 解析与校验
# ---------------------------------------------------------------------------

RE_DEFINE = re.compile(r"^\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)\s+(.+?)\s*$")
RE_C_COMMENT = re.compile(r"/\*.*?\*/|//.*$")


def _parse_literal(raw):
    """把宏值解析成 int；只允许字面量（UL 后缀/行内注释允许），否则抛 LayoutError。"""
    value = RE_C_COMMENT.sub("", raw).strip()
    if not value:
        raise LayoutError("empty macro value: %s" % raw)
    m = re.match(r"^(0[xX][0-9A-Fa-f]+|\d+)([uUlL]*)$", value)
    if not m:
        raise LayoutError(
            "non-literal macro value '%s' (memory_layout.h only allows integer "
            "literals; derived values are cross-checked by this tool)" % value
        )
    return int(m.group(1), 0)


def parse_layout_text(text):
    """从文本解析 #define 常量；非法字面量抛 LayoutError，缺常量由校验层报告。"""
    layout = {}
    for line in text.splitlines():
        m = RE_DEFINE.match(line)
        if m:
            layout[m.group(1)] = _parse_literal(m.group(2))
    return layout


def parse_layout_header(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return parse_layout_text(fh.read())


def check_vtor_alignment(app_base, flash_base):
    """VTOR 对齐：偏移必须是 0x200 的倍数（system 文件要求），且满足 0x80 最低对齐。"""
    errors = []
    offset = app_base - flash_base
    if offset % VTOR_ALIGN_SYS != 0:
        errors.append(
            "VTOR misaligned: APP_BASE-FLASH_BASE=0x%X is not a multiple of 0x%X"
            % (offset, VTOR_ALIGN_SYS)
        )
    if offset % VTOR_ALIGN_MIN != 0:
        errors.append(
            "VTOR misaligned: APP_BASE-FLASH_BASE=0x%X is not a multiple of 0x%X"
            % (offset, VTOR_ALIGN_MIN)
        )
    return errors


def validate_layout(layout):
    """布局头内部一致性校验；返回错误消息列表（空列表 = 通过）。"""
    errors = []

    for name in REQUIRED_LAYOUT_CONSTANTS:
        if name not in layout:
            errors.append("memory_layout.h missing constant: %s" % name)
    if errors:
        return errors

    fb, fe = layout["FLASH_BASE"], layout["FLASH_END_EXCL"]
    page = layout["FLASH_PAGE_SIZE"]
    meta_a, meta_b = layout["META_A_BASE"], layout["META_B_BASE"]

    if fb != 0x08000000:
        errors.append("FLASH_BASE must be 0x08000000 (got 0x%08X)" % fb)
    if page != FLASH_PAGE_SIZE_HI:
        errors.append("FLASH_PAGE_SIZE must be 0x%X (got 0x%X)" % (FLASH_PAGE_SIZE_HI, page))
    if fe <= fb:
        errors.append("FLASH_END_EXCL must be greater than FLASH_BASE")
    elif (fe - fb) % page != 0:
        errors.append("flash span 0x%X is not a multiple of page size" % (fe - fb))
    elif (fe - fb) not in KNOWN_FLASH_SPANS:
        errors.append(
            "flash span 0x%X is not a known C8/CB size (0x10000/0x20000)" % (fe - fb)
        )

    for name, addr in (("META_A_BASE", meta_a), ("META_B_BASE", meta_b)):
        if addr % page != 0:
            errors.append("%s not page aligned: 0x%08X" % (name, addr))
        if not (fb <= addr < fe):
            errors.append("%s beyond flash bounds: 0x%08X" % (name, addr))

    if meta_b + page != fe:
        errors.append(
            "META_B_BASE must be the last flash page (0x%08X + 0x%X != FLASH_END_EXCL 0x%08X)"
            % (meta_b, page, fe)
        )
    if meta_a + page != meta_b:
        errors.append(
            "meta pages overlap or non-adjacent: META_A_BASE+0x%X must equal META_B_BASE"
            " (0x%08X vs 0x%08X)" % (page, meta_a + page, meta_b)
        )

    if layout["APP_LIMIT_EXCL"] != meta_a:
        errors.append(
            "APP_LIMIT_EXCL must equal META_A_BASE (0x%08X vs 0x%08X)"
            % (layout["APP_LIMIT_EXCL"], meta_a)
        )

    boot_part = layout["BOOT_PART_SIZE"]
    if boot_part <= 0 or boot_part % FLASH_PAGE_SIZE_HI != 0:
        errors.append(
            "BOOT_PART_SIZE must be a positive multiple of 0x%X (got 0x%X)"
            % (FLASH_PAGE_SIZE_HI, boot_part)
        )
    if fb + boot_part > layout["APP_LIMIT_EXCL"]:
        errors.append(
            "BOOT partition collides with META pages: 0x%08X > APP_LIMIT_EXCL 0x%08X"
            % (fb + boot_part, layout["APP_LIMIT_EXCL"])
        )

    app_base, app_cap = layout["APP_BASE"], layout["APP_CAPACITY"]
    if app_base != fb + boot_part:
        errors.append(
            "APP_BASE mismatch: FLASH_BASE+BOOT_PART_SIZE=0x%08X but APP_BASE=0x%08X"
            % (fb + boot_part, app_base)
        )

    # 双槽模式：若三个 SLOT_* 常量齐全，则按"双槽 A/B"校验；
    # 否则回退旧"单 APP"校验（保证旧布局头仍可校验通过）。
    has_all_slot = all(name in layout for name in DUAL_SLOT_CONSTANTS)
    has_any_slot = any(name in layout for name in DUAL_SLOT_CONSTANTS)
    if has_any_slot and not has_all_slot:
        missing = [n for n in DUAL_SLOT_CONSTANTS if n not in layout]
        errors.append(
            "dual-slot layout incomplete: missing %s" % ", ".join(missing)
        )
    elif has_all_slot:
        errors.extend(_validate_dual_slot(layout, fb, page, meta_a))
    else:
        # 单槽：APP_CAPACITY == APP_LIMIT_EXCL - APP_BASE
        expected_cap = layout["APP_LIMIT_EXCL"] - app_base
        if app_cap != expected_cap:
            errors.append(
                "APP_CAPACITY mismatch: APP_LIMIT_EXCL-APP_BASE=%d (0x%X) but APP_CAPACITY=%d (0x%X)"
                % (expected_cap, expected_cap, app_cap, app_cap)
            )

    if app_cap <= 0:
        errors.append("BOOT partition leaves no room for APP (APP_CAPACITY=%d)" % app_cap)

    errors.extend(check_vtor_alignment(app_base, fb))

    if layout["LAYOUT_PROVISIONAL"] not in (0, 1):
        errors.append(
            "LAYOUT_PROVISIONAL must be 0 or 1 (got %d)" % layout["LAYOUT_PROVISIONAL"]
        )

    return errors


def _validate_dual_slot(layout, fb, page, meta_a):
    """双槽 A/B 布局内部一致性校验。

    约束：
      - SLOT_A_BASE == APP_BASE == FLASH_BASE + BOOT_PART_SIZE；
      - SLOT_B_BASE == SLOT_A_BASE + SLOT_CAPACITY（连续）；
      - SLOT_B_BASE + SLOT_CAPACITY == APP_LIMIT_EXCL（精确，无空洞）；
      - APP_CAPACITY == SLOT_CAPACITY（单槽容量，兼容别名）；
      - 两槽基址相对 FLASH_BASE 均为 0x200 的倍数（VTOR 要求）。

    注：SLOT_CAPACITY 与槽基址**不要求**是 2 KiB 页倍数。前者是镜像越界
    判定上界、后者是加载基址，二者均非擦除单元；Boot 擦写从 appBase 起
    按页推进，与槽容量是否页整数倍无关。要求页对齐的只有 META_A/META_B、
    BOOT_PART_SIZE 与 Flash 总跨度。
    """
    errors = []
    fb_local = fb
    slot_a = layout["SLOT_A_BASE"]
    slot_b = layout["SLOT_B_BASE"]
    slot_cap = layout["SLOT_CAPACITY"]
    app_limit = layout["APP_LIMIT_EXCL"]
    boot_part = layout["BOOT_PART_SIZE"]

    if slot_cap <= 0:
        errors.append("SLOT_CAPACITY must be positive (got 0x%X)" % slot_cap)
    if slot_a != fb_local + boot_part:
        errors.append(
            "SLOT_A_BASE must equal FLASH_BASE+BOOT_PART_SIZE (0x%08X vs 0x%08X)"
            % (fb_local + boot_part, slot_a)
        )
    if layout["APP_BASE"] != slot_a:
        errors.append(
            "APP_BASE (default alias) must equal SLOT_A_BASE (0x%08X vs 0x%08X)"
            % (layout["APP_BASE"], slot_a)
        )
    if layout["APP_CAPACITY"] != slot_cap:
        errors.append(
            "APP_CAPACITY (per-slot alias) must equal SLOT_CAPACITY (%d vs %d)"
            % (layout["APP_CAPACITY"], slot_cap)
        )
    if slot_b != slot_a + slot_cap:
        errors.append(
            "SLOT_B_BASE must equal SLOT_A_BASE+SLOT_CAPACITY (0x%08X vs 0x%08X)"
            % (slot_a + slot_cap, slot_b)
        )
    if slot_b + slot_cap != app_limit:
        errors.append(
            "slot B end must equal APP_LIMIT_EXCL exactly (0x%08X vs 0x%08X)"
            % (slot_b + slot_cap, app_limit)
        )
    for name, addr in (("SLOT_A_BASE", slot_a), ("SLOT_B_BASE", slot_b)):
        # 槽基址非擦除单元，只需 VTOR 0x200 对齐（下方统一校验）。
        offset = addr - fb_local
        if offset % VTOR_ALIGN_SYS != 0:
            errors.append(
                "%s VTOR offset 0x%X not a multiple of 0x%X"
                % (name, offset, VTOR_ALIGN_SYS)
            )
    return errors


# ---------------------------------------------------------------------------
# uvprojx 解析与 Target 校验
# ---------------------------------------------------------------------------

RE_CPU_IROM = re.compile(r"IROM\((0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\)")
RE_CPU_IRAM = re.compile(r"IRAM\((0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\)")


def parse_uvprojx_text(text):
    """解析单 Target uvprojx 的关键配置（Cpu 串、OnChipMemories、C 宏定义）。"""
    root = ET.fromstring(text)
    target = {"defines": {}}

    target["target_name"] = root.findtext(".//TargetName") or ""

    cpu = root.findtext(".//Cpu") or ""
    m_irom = RE_CPU_IROM.search(cpu)
    m_iram = RE_CPU_IRAM.search(cpu)
    if m_irom:
        target["cpu_irom_base"] = int(m_irom.group(1), 0)
        target["cpu_irom_size"] = int(m_irom.group(2), 0)
    if m_iram:
        target["cpu_iram_base"] = int(m_iram.group(1), 0)
        target["cpu_iram_size"] = int(m_iram.group(2), 0)

    for key, tag in (("irom", "IROM"), ("iram", "IRAM")):
        node = root.find(".//OnChipMemories/%s" % tag)
        if node is not None:
            start = node.findtext("StartAddress")
            size = node.findtext("Size")
            if start is not None and size is not None:
                target["%s_base" % key] = int(start, 0)
                target["%s_size" % key] = int(size, 0)

    define_text = root.findtext(".//Cads/VariousControls/Define") or ""
    for token in define_text.split(","):
        token = token.strip()
        if not token:
            continue
        if "=" in token:
            name, value = token.split("=", 1)
            target["defines"][name.strip()] = value.strip()
        else:
            target["defines"][token] = None

    return target


def parse_uvprojx(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return parse_uvprojx_text(fh.read())


def _vect_tab_offset(target):
    value = target.get("defines", {}).get("VECT_TAB_OFFSET")
    return None if value is None else int(value, 0)


def validate_target(layout, target, role):
    """校验 uvprojx Target 与 memory_layout.h 一致；role 为 'boot' 或 'app'。"""
    errors = []
    label = "boot" if role == "boot" else "app"

    if "irom_base" not in target or "irom_size" not in target:
        errors.append("%s uvprojx: OnChipMemories/IROM not found" % label)
        return errors

    if role == "boot":
        exp_base, exp_size = layout["FLASH_BASE"], layout["BOOT_PART_SIZE"]
    else:
        exp_base, exp_size = layout["APP_BASE"], layout["APP_CAPACITY"]

    if target["irom_base"] != exp_base or target["irom_size"] != exp_size:
        errors.append(
            "%s uvprojx IROM mismatch: got 0x%08X/0x%X, expected 0x%08X/0x%X"
            % (label, target["irom_base"], target["irom_size"], exp_base, exp_size)
        )
    if target.get("cpu_irom_base") != target["irom_base"] or target.get(
        "cpu_irom_size"
    ) != target["irom_size"]:
        errors.append(
            "%s uvprojx: Cpu string IROM (0x%08X/0x%X) disagrees with OnChipMemories IROM "
            "(0x%08X/0x%X)"
            % (
                label,
                target.get("cpu_irom_base", -1),
                target.get("cpu_irom_size", -1),
                target["irom_base"],
                target["irom_size"],
            )
        )

    if (
        target.get("iram_base") != SRAM_BASE
        or target.get("iram_size") != SRAM_SIZE
    ):
        errors.append(
            "%s uvprojx IRAM must be full SRAM 0x%08X/0x%X (got 0x%08X/0x%X)"
            % (
                label,
                SRAM_BASE,
                SRAM_SIZE,
                target.get("iram_base", -1),
                target.get("iram_size", -1),
            )
        )

    vtor = _vect_tab_offset(target)
    if role == "boot":
        if vtor not in (None, 0):
            errors.append(
                "boot uvprojx VECT_TAB_OFFSET must be 0 or absent (got 0x%X)" % vtor
            )
    else:
        # 双槽模式下允许 Slot A 或 Slot B 的偏移（两槽各编一份镜像）；
        # 单槽模式只允许 APP_BASE-FLASH_BASE。
        expected_offset = layout["APP_BASE"] - layout["FLASH_BASE"]
        if all(name in layout for name in ("SLOT_A_BASE", "SLOT_B_BASE")):
            allowed_offsets = (
                layout["SLOT_A_BASE"] - layout["FLASH_BASE"],
                layout["SLOT_B_BASE"] - layout["FLASH_BASE"],
            )
        else:
            allowed_offsets = (expected_offset,)
        if vtor is None:
            errors.append(
                "app uvprojx VECT_TAB_OFFSET missing (expected one of %s)"
                % ", ".join("0x%X" % o for o in allowed_offsets)
            )
        elif vtor not in allowed_offsets:
            errors.append(
                "app uvprojx VECT_TAB_OFFSET mismatch: got 0x%X, expected one of %s"
                % (vtor, ", ".join("0x%X" % o for o in allowed_offsets))
            )

    return errors


# ---------------------------------------------------------------------------
# map 解析与镜像校验
# ---------------------------------------------------------------------------

RE_LR_ANY = re.compile(r"Load Region LR_IROM\d+")
RE_LR_IROM1 = re.compile(
    r"Load Region LR_IROM1\s*"
    r"\(Base:\s*(0x[0-9A-Fa-f]+),\s*Size:\s*(0x[0-9A-Fa-f]+),\s*"
    r"Max:\s*(0x[0-9A-Fa-f]+)"
)
RE_ER_IROM1 = re.compile(
    r"Execution Region ER_IROM1\s*"
    r"\(Exec base:\s*(0x[0-9A-Fa-f]+),\s*Load base:\s*(0x[0-9A-Fa-f]+),\s*"
    r"Size:\s*(0x[0-9A-Fa-f]+)"
)
RE_TOTAL_RW = re.compile(r"Total RW\s+Size \(RW Data \+ ZI Data\)\s+(\d+)")


def parse_map_text(text):
    """解析 Keil map：加载域个数（多个 = 分区内有洞）、LR_IROM1、Total RW。"""
    fields = {"lr_count": len(RE_LR_ANY.findall(text))}

    m = RE_LR_IROM1.search(text)
    if m:
        fields["lr_base"] = int(m.group(1), 16)
        fields["lr_size"] = int(m.group(2), 16)

    m = RE_ER_IROM1.search(text)
    if m:
        fields["er_load_base"] = int(m.group(2), 16)

    m = RE_TOTAL_RW.search(text)
    if m:
        fields["total_rw"] = int(m.group(1))

    return fields


def parse_map(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return parse_map_text(fh.read())


def validate_map(layout, fields, role):
    """校验镜像落在分区内且单一加载域（map 有洞即失败）。"""
    errors = []
    label = "boot" if role == "boot" else "app"

    if fields.get("lr_count") != 1:
        errors.append(
            "%s map has %s load regions (hole inside partition): expected exactly 1"
            % (label, fields.get("lr_count", "?"))
        )
        return errors

    if "lr_base" not in fields or "lr_size" not in fields:
        errors.append("%s map: LR_IROM1 base/size not found" % label)
        return errors

    if role == "boot":
        exp_base = layout["FLASH_BASE"]
        limit = layout["FLASH_BASE"] + layout["BOOT_PART_SIZE"]
        limit_name = "BOOT partition end 0x%08X" % limit
    else:
        exp_base = layout["APP_BASE"]
        limit = layout["APP_LIMIT_EXCL"]
        limit_name = "APP_LIMIT_EXCL 0x%08X" % limit

    if fields["lr_base"] != exp_base:
        errors.append(
            "%s map LR_IROM1 base mismatch: got 0x%08X, expected 0x%08X"
            % (label, fields["lr_base"], exp_base)
        )

    lr_end = fields["lr_base"] + fields["lr_size"]
    if lr_end > limit:
        errors.append(
            "%s map LR_IROM1 end 0x%08X exceeds %s" % (label, lr_end, limit_name)
        )

    if fields.get("er_load_base") is not None and fields["er_load_base"] != fields["lr_base"]:
        errors.append(
            "%s map ER_IROM1 load base 0x%08X != LR_IROM1 base 0x%08X"
            % (label, fields["er_load_base"], fields["lr_base"])
        )

    return errors


def check_bin_fits(label, bin_size, capacity):
    """bin 长度必须为正且不超过分区容量（刚好越界 1 B 也必须失败）。"""
    errors = []
    if bin_size is None:
        errors.append("%s bin missing" % label)
    elif bin_size == 0:
        errors.append("%s bin is empty" % label)
    elif bin_size > capacity:
        errors.append(
            "%s bin %d B exceeds capacity %d B (by %d B)"
            % (label, bin_size, capacity, bin_size - capacity)
        )
    return errors


def check_ram_budget(label, total_rw, budget):
    """静态 RAM 预算（设计 §4：APP <= 18432 B，Boot <= 4096 B）。"""
    errors = []
    if total_rw is None:
        errors.append("%s map: Total RW not found" % label)
    elif total_rw > budget:
        errors.append(
            "%s total RW %d B exceeds budget %d B" % (label, total_rw, budget)
        )
    return errors


# ---------------------------------------------------------------------------
# Task 16 冻结期镜像级校验：向量表 / Flash 区间 / 半字对齐
# ---------------------------------------------------------------------------


def validate_vector_table(data, img_base, img_limit):
    """校验镜像前 8 B 向量表（Task 16 脚本核对项）。

    word0 = initial MSP：必须落在 SRAM (0x20000000, 0x20005000] 且 4 B 对齐；
    word1 = Reset PC：必须置 Thumb 位（bit0=1），且入口落在本镜像分区
    [img_base, img_limit) 内（Boot/APP 各自校验，防烧错镜像）。
    """
    errors = []
    if data is None or len(data) < 8:
        return [
            "vector table: need >= 8 bytes (MSP+PC), got %s"
            % ("none" if data is None else len(data))
        ]
    msp, pc = struct.unpack("<II", data[:8])
    if not (SRAM_BASE < msp <= SRAM_BASE + SRAM_SIZE):
        errors.append(
            "vector MSP 0x%08X outside SRAM (0x%08X,0x%08X]"
            % (msp, SRAM_BASE, SRAM_BASE + SRAM_SIZE)
        )
    if msp % 4 != 0:
        errors.append("vector MSP 0x%08X not 4-byte aligned" % msp)
    if pc & 1 != 1:
        errors.append("vector PC 0x%08X missing Thumb bit" % pc)
    entry = pc & ~1
    if not (img_base <= entry < img_limit):
        errors.append(
            "vector PC entry 0x%08X outside image [0x%08X,0x%08X)"
            % (entry, img_base, img_limit)
        )
    return errors


def intervals_overlap(a_start, a_end_excl, b_start, b_end_excl):
    """两个半开区间 [start, end) 是否相交。"""
    return a_start < b_end_excl and b_start < a_end_excl


def validate_flash_intervals(boot_base, boot_len, app_base, app_len,
                             meta_base, flash_end_excl):
    """Boot/App 实际镜像区间互不重叠，且都不侵入元数据两页。

    SRAM 允许重叠（两镜像不同时执行），Flash 不允许（Task 16 脚本核对项）。
    """
    errors = []
    boot_end = boot_base + boot_len
    app_end = app_base + app_len
    if intervals_overlap(boot_base, boot_end, app_base, app_end):
        errors.append(
            "boot image [0x%08X,0x%08X) overlaps app image [0x%08X,0x%08X)"
            % (boot_base, boot_end, app_base, app_end)
        )
    for label, start, end in (
        ("boot", boot_base, boot_end),
        ("app", app_base, app_end),
    ):
        if intervals_overlap(start, end, meta_base, flash_end_excl):
            errors.append(
                "%s image [0x%08X,0x%08X) reaches into metadata pages "
                "[0x%08X,0x%08X)" % (label, start, end, meta_base, flash_end_excl)
            )
    return errors


def check_halfword_aligned(label, size):
    """Boot 半字编程约束：bin 长度必须为偶数。"""
    if size is None:
        return ["%s bin missing" % label]
    if size % 2 != 0:
        return [
            "%s bin length %d is odd (halfword programming requires even)"
            % (label, size)
        ]
    return []


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def _report_lines(layout, boot_t, app_t, boot_m, app_m, boot_bin, app_bin,
                  app_audit_bin, final_gate=False):
    """生成报告文本与失败标记，供 main() 打印并决定退出码。"""
    failures = []
    lines = []

    prov = layout.get("LAYOUT_PROVISIONAL")
    lines.append("layout: %s" % ("PROVISIONAL" if prov == 1 else "FINAL"))
    lines.append("flash_end_exclusive: 0x%08X" % layout.get("FLASH_END_EXCL", 0))
    lines.append("layout_provisional: %s" % prov)

    if final_gate:
        if prov != 0:
            lines.append(
                "FAIL: LAYOUT_PROVISIONAL must be 0 when --final (layout not frozen)"
            )
            failures.append("LAYOUT_PROVISIONAL must be 0 with --final")
        else:
            lines.append("final gate: LAYOUT_PROVISIONAL == 0 -> PASS")

    layout_errors = validate_layout(layout)
    meta_errors = [e for e in layout_errors if "META" in e]
    if meta_errors:
        lines.append("meta_a/meta_b: FAIL (%s)" % meta_errors[0])
        failures.extend(meta_errors)
    else:
        lines.append("meta_a/meta_b: page aligned, non-overlapping")
    for e in layout_errors:
        if e not in meta_errors:
            lines.append("FAIL: layout: %s" % e)
            failures.append(e)

    cap = layout.get("APP_CAPACITY")
    if cap is None:
        lines.append("current capacity warning: FAIL (APP_CAPACITY unknown)")
        failures.append("APP_CAPACITY unknown")
    else:
        audit_size = None if app_audit_bin is None else os.path.getsize(app_audit_bin)
        if audit_size is not None and audit_size <= cap:
            lines.append(
                "current capacity warning: PASS (app audit bin %d B <= APP_CAPACITY %d B)"
                % (audit_size, cap)
            )
        elif audit_size is not None:
            lines.append(
                "current capacity warning: FAIL (app audit bin %d B > APP_CAPACITY %d B)"
                % (audit_size, cap)
            )
            failures.append("app audit bin exceeds APP_CAPACITY")

    # 双槽模式摘要
    if all(name in layout for name in DUAL_SLOT_CONSTANTS):
        lines.append(
            "dual-slot A/B: SLOT_A 0x%08X + 0x%X, SLOT_B 0x%08X + 0x%X, "
            "meta 0x%08X..0x%08X"
            % (
                layout["SLOT_A_BASE"],
                layout["SLOT_CAPACITY"],
                layout["SLOT_B_BASE"],
                layout["SLOT_CAPACITY"],
                layout["META_A_BASE"],
                layout["FLASH_END_EXCL"],
            )
        )

    for label, tgt, role in (
        ("boot", boot_t, "boot"),
        ("app", app_t, "app"),
    ):
        if tgt is None:
            lines.append("FAIL: %s uvprojx: not parsed" % label)
            failures.append("%s uvprojx not parsed" % label)
            continue
        errors = validate_target(layout, tgt, role)
        if errors:
            for e in errors:
                lines.append("FAIL: %s" % e)
                failures.append(e)
        else:
            lines.append(
                "%s target '%s': IROM 0x%08X/0x%X, IRAM 0x%08X/0x%X, "
                "VECT_TAB_OFFSET=%s -> PASS"
                % (
                    label,
                    tgt["target_name"],
                    tgt["irom_base"],
                    tgt["irom_size"],
                    tgt.get("iram_base", 0),
                    tgt.get("iram_size", 0),
                    "none" if _vect_tab_offset(tgt) is None else "0x%X" % _vect_tab_offset(tgt),
                )
            )

    for label, m, role, budget in (
        ("boot", boot_m, "boot", BOOT_RAM_BUDGET),
        ("app", app_m, "app", APP_RAM_BUDGET),
    ):
        if m is None:
            lines.append("FAIL: %s map: not parsed" % label)
            failures.append("%s map not parsed" % label)
            continue
        errors = validate_map(layout, m, role)
        errors += check_ram_budget(label, m.get("total_rw"), budget)
        if errors:
            for e in errors:
                lines.append("FAIL: %s" % e)
                failures.append(e)
        else:
            lines.append(
                "%s map: LR_IROM1 base 0x%08X size %d B, Total RW %d B <= %d B -> PASS"
                % (label, m["lr_base"], m["lr_size"], m["total_rw"], budget)
            )

    for label, path, capacity in (
        ("boot", boot_bin, layout.get("BOOT_PART_SIZE", 0)),
        ("app", app_bin, layout.get("APP_CAPACITY", 0)),
    ):
        size = None if path is None else os.path.getsize(path)
        errors = check_bin_fits(label, size, capacity)
        if errors:
            for e in errors:
                lines.append("FAIL: %s" % e)
                failures.append(e)
        else:
            lines.append(
                "%s bin %d B <= %s capacity %d B -> PASS"
                % (label, size, label, capacity)
            )

    # Task 16 镜像级校验：向量表 MSP/PC、Flash 区间不重叠、半字对齐
    fb0 = layout.get("FLASH_BASE", 0x08000000)
    boot_part = layout.get("BOOT_PART_SIZE", 0)
    app_base_l = layout.get("APP_BASE", 0)
    app_limit_l = layout.get("APP_LIMIT_EXCL", 0)
    meta_a_l = layout.get("META_A_BASE", 0)
    fe_l = layout.get("FLASH_END_EXCL", 0)

    boot_size = None if boot_bin is None else os.path.getsize(boot_bin)
    app_size = None if app_bin is None else os.path.getsize(app_bin)

    for label, path, base, limit in (
        ("boot", boot_bin, fb0, fb0 + boot_part),
        ("app", app_bin, app_base_l, app_limit_l),
    ):
        if path is None:
            continue
        with open(path, "rb") as fh:
            head = fh.read(8)
        errors = validate_vector_table(head, base, limit)
        if errors:
            for e in errors:
                lines.append("FAIL: %s vector: %s" % (label, e))
                failures.append(e)
        else:
            msp, pc = struct.unpack("<II", head)
            lines.append(
                "%s vector: MSP 0x%08X in SRAM, PC 0x%08X thumb in "
                "[0x%08X,0x%08X) -> PASS" % (label, msp, pc, base, limit)
            )

    if boot_size is not None and app_size is not None:
        # 双槽模式下：Boot 镜像 + 两个槽各自的 APP 副本都不得相交/越界
        intervals = [
            ("boot", fb0, fb0 + boot_size),
        ]
        if all(name in layout for name in DUAL_SLOT_CONSTANTS):
            sa = layout["SLOT_A_BASE"]
            sb = layout["SLOT_B_BASE"]
            intervals.append(("app(slotA)", sa, sa + app_size))
            intervals.append(("app(slotB)", sb, sb + app_size))
        else:
            intervals.append(("app", app_base_l, app_base_l + app_size))

        inter_errors = []
        for i in range(len(intervals)):
            for j in range(i + 1, len(intervals)):
                a = intervals[i]
                b = intervals[j]
                if intervals_overlap(a[1], a[2], b[1], b[2]):
                    inter_errors.append(
                        "%s image [0x%08X,0x%08X) overlaps %s image [0x%08X,0x%08X)"
                        % (a[0], a[1], a[2], b[0], b[1], b[2])
                    )
            label, start, end = intervals[i]
            if intervals_overlap(start, end, meta_a_l, fe_l):
                inter_errors.append(
                    "%s image [0x%08X,0x%08X) reaches into metadata pages "
                    "[0x%08X,0x%08X)" % (label, start, end, meta_a_l, fe_l)
                )
            if end > app_limit_l:
                inter_errors.append(
                    "%s image end 0x%08X exceeds APP_LIMIT_EXCL 0x%08X"
                    % (label, end, app_limit_l)
                )

        if inter_errors:
            for e in inter_errors:
                lines.append("FAIL: flash intervals: %s" % e)
                failures.extend(inter_errors)
        else:
            lines.append(
                "flash intervals: %s disjoint and clear of meta [0x%08X,0x%08X) -> PASS"
                % (
                    ", ".join("%s [0x%08X,0x%08X)" % iv for iv in intervals),
                    meta_a_l,
                    fe_l,
                )
            )

    for label, size in (("boot", boot_size), ("app", app_size)):
        errors = check_halfword_aligned(label, size)
        if errors:
            for e in errors:
                lines.append("FAIL: %s" % e)
                failures.extend(errors)
        else:
            lines.append("%s bin halfword aligned (%d B) -> PASS" % (label, size))

    fb = layout.get("FLASH_BASE", 0x08000000)
    app_limit = layout.get("APP_LIMIT_EXCL", 0)
    if app_limit:
        cap12, a12 = tight_window(fb, app_limit, 0x3000)
        cap16, a16 = tight_window(fb, app_limit, 0x4000)
        lines.append(
            "tight window (hard margin %d B): boot 12 KiB -> APP_CAPACITY %d B, "
            "A_max %d B; boot 16 KiB -> APP_CAPACITY %d B, A_max %d B"
            % (HARD_MARGIN, cap12, a12, cap16, a16)
        )

    return lines, failures


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Cross-check memory_layout.h, provisional uvprojx targets, "
        "map and bin artifacts for address/size consistency"
    )
    parser.add_argument("--layout", required=True, help="CONFIG/memory_layout.h")
    parser.add_argument("--boot-proj", required=True, help="Boot 临时 Target uvprojx")
    parser.add_argument("--boot-map", required=True, help="Boot 临时构建 map")
    parser.add_argument("--boot-bin", required=True, help="Boot 临时构建 bin")
    parser.add_argument("--app-proj", required=True, help="App 临时 Target uvprojx")
    parser.add_argument("--app-map", required=True, help="App 临时构建 map")
    parser.add_argument("--app-bin", required=True, help="App 临时构建 bin")
    parser.add_argument(
        "--app-audit-bin", required=True, help="App_SizeAudit.bin（当前容量预警用）"
    )
    parser.add_argument(
        "--final",
        action="store_true",
        help="Task 16 冻结门禁：要求 LAYOUT_PROVISIONAL == 0（未冻结即失败）",
    )
    args = parser.parse_args(argv)

    paths = {
        "layout": args.layout,
        "boot-proj": args.boot_proj,
        "boot-map": args.boot_map,
        "boot-bin": args.boot_bin,
        "app-proj": args.app_proj,
        "app-map": args.app_map,
        "app-bin": args.app_bin,
        "app-audit-bin": args.app_audit_bin,
    }
    for key, path in paths.items():
        if not os.path.isfile(path):
            print("FAIL: file not found (%s): %s" % (key, path))
            return 1

    try:
        layout = parse_layout_header(args.layout)
    except LayoutError as exc:
        print("FAIL: cannot parse %s: %s" % (args.layout, exc))
        return 1

    boot_t = parse_uvprojx(args.boot_proj)
    app_t = parse_uvprojx(args.app_proj)
    boot_m = parse_map(args.boot_map)
    app_m = parse_map(args.app_map)

    lines, failures = _report_lines(
        layout, boot_t, app_t, boot_m, app_m,
        args.boot_bin, args.app_bin, args.app_audit_bin,
        final_gate=args.final,
    )

    for line in lines:
        print(line)
    print("overall: %s" % ("PASS" if not failures else "FAIL"))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
