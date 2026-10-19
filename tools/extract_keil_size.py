#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_keil_size.py -- Keil 尺寸审计数据提取

从 Keil 生成的 map 文件与 fromelf 生成的 bin 文件中提取镜像尺寸证据：
  - LR_IROM1 Load Region base / size（完整落 Flash 的镜像大小，含 RW load copy）
  - ER_IROM1 Execution Region size（代码 + RO 数据）
  - Total RW Size（RW Data + ZI Data，静态 RAM 占用）
  - .bin 文件长度（fromelf --bin 输出，单一加载域时应等于 LR size）

失败策略（与计划一致）：
  - map / bin 文件不存在 -> 失败
  - 任一字段（LR base、LR size、ER size、Total RW、bin 长度）解析不到 -> 失败
  - 不从 Code/RO/RW 的 Program Size 数字推算任何字段

用法：
  python tools/extract_keil_size.py --map OBJ_BOOT_AUDIT/boot.map --bin OBJ_BOOT_AUDIT/boot.bin

退出码：0 成功，1 失败（字段缺失或文件不存在）。
"""

import argparse
import os
import re
import sys

# 本轮所有数字都是 PRELIMINARY：分区布局尚未冻结（Task 3 才建立临时布局）
CLASSIFICATION = "PRELIMINARY"

# Keil armlink map 行格式：
#   Load Region LR_IROM1 (Base: 0x08000000, Size: 0x00000aa0, Max: 0x00010000, ABSOLUTE)
RE_LOAD_REGION = re.compile(
    r"Load Region LR_IROM1\s*"
    r"\(Base:\s*(0x[0-9A-Fa-f]+),\s*Size:\s*(0x[0-9A-Fa-f]+),\s*"
    r"Max:\s*(0x[0-9A-Fa-f]+)"
)

#   Execution Region ER_IROM1 (Exec base: 0x08000000, Load base: 0x08000000, Size: 0x00000a84, Max: 0x00010000, ABSOLUTE)
RE_EXEC_REGION = re.compile(
    r"Execution Region ER_IROM1\s*"
    r"\(Exec base:\s*(0x[0-9A-Fa-f]+),\s*Load base:\s*(0x[0-9A-Fa-f]+),\s*"
    r"Size:\s*(0x[0-9A-Fa-f]+)"
)

#   Total RW  Size (RW Data + ZI Data)              1792 (   1.75kB)
RE_TOTAL_RW = re.compile(
    r"Total RW\s+Size \(RW Data \+ ZI Data\)\s+(\d+)"
)


def parse_map(map_path):
    """解析 map 文件，返回字段字典；任一关键字段缺失时由调用方判定失败。"""
    fields = {}
    with open(map_path, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()

    m_lr = RE_LOAD_REGION.search(text)
    if m_lr:
        fields["lr_base"] = int(m_lr.group(1), 16)
        fields["lr_size"] = int(m_lr.group(2), 16)
        fields["lr_max"] = int(m_lr.group(3), 16)

    m_er = RE_EXEC_REGION.search(text)
    if m_er:
        fields["er_exec_base"] = int(m_er.group(1), 16)
        fields["er_load_base"] = int(m_er.group(2), 16)
        fields["er_size"] = int(m_er.group(3), 16)

    m_rw = RE_TOTAL_RW.search(text)
    if m_rw:
        fields["total_rw"] = int(m_rw.group(1))

    return fields


def main():
    parser = argparse.ArgumentParser(
        description="Extract Boot/App size audit numbers from Keil map and bin files"
    )
    parser.add_argument("--map", required=True, help="Keil armlink 生成的 .map 文件")
    parser.add_argument("--bin", required=True, help="fromelf --bin 生成的 .bin 文件")
    parser.add_argument("--label", default=None, help="输出标签（默认取 bin 文件名）")
    args = parser.parse_args()

    label = args.label or os.path.splitext(os.path.basename(args.bin))[0]

    # 文件存在性检查：缺文件直接失败
    for path in (args.map, args.bin):
        if not os.path.isfile(path):
            sys.stderr.write("[extract_keil_size] FAIL: file not found: %s\n" % path)
            return 1

    fields = parse_map(args.map)
    bin_size = os.path.getsize(args.bin)

    # 字段完整性检查：计划要求“若找不到任一字段则失败，不从 Code 数字推算”
    required = ("lr_base", "lr_size", "er_size", "total_rw")
    missing = [k for k in required if k not in fields]
    if missing:
        sys.stderr.write(
            "[extract_keil_size] FAIL: map fields missing (%s) in %s\n"
            % (", ".join(missing), args.map)
        )
        return 1

    # 输出三组关键数字（十进制 + 十六进制）+ 支撑字段
    print("target: %s" % label)
    print("classification: %s" % CLASSIFICATION)
    print("map: %s" % args.map)
    print("bin: %s" % args.bin)
    print("lr_irom1_base: %d (0x%08X)" % (fields["lr_base"], fields["lr_base"]))
    print("load_region_size: %d (0x%X)" % (fields["lr_size"], fields["lr_size"]))
    print("er_irom1_size: %d (0x%X)" % (fields["er_size"], fields["er_size"]))
    print("total_rw: %d (0x%X)" % (fields["total_rw"], fields["total_rw"]))
    print("bin_size: %d (0x%X)" % (bin_size, bin_size))

    # 一致性检查（信息性输出，不改变退出码）：
    # 单一加载域时 fromelf --bin 的 bin 长度应等于 LR size；
    # 不一致通常意味着 map/bin 不是同一次构建的产物。
    if bin_size != fields["lr_size"]:
        print(
            "WARNING: bin_size != load_region_size (%d vs %d); "
            "check that map and bin come from the same build"
            % (bin_size, fields["lr_size"])
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
