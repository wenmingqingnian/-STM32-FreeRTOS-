#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
iap_stop_gates.py -- Task 16 冻结门禁集合（IAP+STOP 计划收尾）

三个子命令，全部由 tools/build-iap-stop.ps1 在 Task 16 流水线中调用：

  audit-gates  FINAL_AUDIT 尺寸门禁 + 冻结布局公式交叉校验：
                 - map/bin 同源（bin_size == LR_IROM1 Size）
                 - Boot Total RW <= 4 KiB、APP Total RW <= 18 KiB
                 - bin 长度为偶数（STM32 半字编程约束）
                 - BOOT_PART_SIZE == align_up(B_bin + 2048, 2048)
                 - APP_BASE == FLASH_BASE + BOOT_PART_SIZE
                 - APP_CAPACITY == APP_LIMIT_EXCL - APP_BASE
                 - A_bin <= APP_CAPACITY - 2048（2 KiB 硬余量）
                 - LAYOUT_PROVISIONAL == 0（冻结标记）
               末行输出机器可读的 LAYOUT_COMPUTED（供收敛比对逐字 diff）。

  roundtrip    .iap 帧流解包回原 .bin 的字节级 round-trip 验证：
                 - 恰好 1 个 START(seq=0)，manifest 与 bin/布局一致
（target_base/size/crc32 逐字段校验）；
                 - DATA(seq=1..N) 连续、offset 递增无洞、非尾包满 512 B；
                 - END(seq=N+1) offset == image_size、payload 为空；
                 - 重组字节与原 bin 完全一致。

  stale-scan   旧绝对地址散落扫描：在源码目录中查找旧 C8 时代地址
（0x0800F000/0x0800EC00/0x20004000）与旧缓冲符号
（FLASH_APP_MAX_SIZE/buff_size）。允许例外仅限
               memory_layout.h、startup/system 文件。

用法（由 build-iap-stop.ps1 调用；也可单独执行）：
  python tools/iap_stop_gates.py audit-gates --layout CONFIG/memory_layout.h `
      --boot-map OBJ_BOOT_PROV/boot.map --boot-bin OBJ_BOOT_PROV/boot.bin `
      --app-map OBJ_APP_PROV/range_hood_app.map `
      --app-bin OBJ_APP_PROV/range_hood_app.bin --label PROV_PASS1
  python tools/iap_stop_gates.py roundtrip `
      --iap OBJ_APP_PROV/range_hood_app.iap `
      --bin OBJ_APP_PROV/range_hood_app.bin --layout CONFIG/memory_layout.h
  python tools/iap_stop_gates.py stale-scan BSP APP_TASK SYSTEM USER CONFIG BOOT

退出码：0 全部门禁通过；1 任一门禁失败。
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_memory_layout as cml          # noqa: E402  布局解析/预算常量
import extract_keil_size as eks            # noqa: E402  map 字段提取
import iap_pack as ipk                     # noqa: E402  帧协议常量/解析


# ---------------------------------------------------------------------------
# audit-gates：FINAL_AUDIT 门禁 + 冻结公式交叉校验
# ---------------------------------------------------------------------------

def cmd_audit_gates(args):
    failures = []

    def check(cond, ok_msg, fail_msg):
        if cond:
            print("PASS: %s" % ok_msg)
        else:
            print("FAIL: %s" % fail_msg)
            failures.append(fail_msg)

    # ---- 布局头（冻结标记 + 全常量） ----
    try:
        layout = cml.parse_layout_header(args.layout)
    except cml.LayoutError as exc:
        print("FAIL: cannot parse layout: %s" % exc)
        return 1
    missing = [k for k in cml.REQUIRED_LAYOUT_CONSTANTS if k not in layout]
    if missing:
        print("FAIL: layout constants missing: %s" % ", ".join(missing))
        return 1

    check(layout["LAYOUT_PROVISIONAL"] == 0,
          "layout_provisional == 0 (frozen)",
          "LAYOUT_PROVISIONAL must be 0 (layout not frozen)")

    # ---- map/bin 提取 ----
    artifacts = {}
    for label, mp, bp in (("boot", args.boot_map, args.boot_bin),
                          ("app", args.app_map, args.app_bin)):
        for p in (mp, bp):
            if not os.path.isfile(p):
                print("FAIL: file not found (%s): %s" % (label, p))
                return 1
        fields = eks.parse_map(mp)
        # Total ROM Size（Code+RO+RW）= fromelf --bin 的真实 Flash 镜像长度
        #（RW 加载副本被 armlink 压缩时，LR_IROM1 Size 是未压缩视图）
        with open(mp, "r", encoding="utf-8", errors="replace") as fh:
            mtext = fh.read()
        m_rom = re.search(
            r"Total ROM\s+Size \(Code \+ RO Data \+ RW Data\)\s+(\d+)", mtext)
        if m_rom:
            fields["total_rom"] = int(m_rom.group(1))
        need = ("lr_base", "lr_size", "total_rw")
        miss = [k for k in need if k not in fields]
        if miss:
            print("FAIL: %s map fields missing (%s) in %s"
                  % (label, ", ".join(miss), mp))
            return 1
        artifacts[label] = {
            "map": fields,
            "bin_size": os.path.getsize(bp),
        }

    b_bin = artifacts["boot"]["bin_size"]
    a_bin = artifacts["app"]["bin_size"]
    b_rw = artifacts["boot"]["map"]["total_rw"]
    a_rw = artifacts["app"]["map"]["total_rw"]

    print("label: %s" % args.label)
    print("boot: bin_size=%d lr_size=%d total_rw=%d lr_base=0x%08X"
          % (b_bin, artifacts["boot"]["map"]["lr_size"], b_rw,
             artifacts["boot"]["map"]["lr_base"]))
    print("app:  bin_size=%d lr_size=%d total_rw=%d lr_base=0x%08X"
          % (a_bin, artifacts["app"]["map"]["lr_size"], a_rw,
             artifacts["app"]["map"]["lr_base"]))

    # ---- 门禁 1：map/bin 同源（同一次构建的产物） ----
    # armlink 可对 RW 加载副本压缩（map 标记 COMPRESSED[...]）：
    #   真实 Flash 镜像（fromelf --bin）== Total ROM Size（Code+RO+RW）
    #   且 <= LR_IROM1 Size（未压缩视图）。
    # bin 与 LR 直接相等会把压缩镜像误判为不同源；按，以 Total ROM 为同源判据。
    for label, info in (("boot", artifacts["boot"]), ("app", artifacts["app"])):
        rom = info["map"].get("total_rom")
        lr = info["map"]["lr_size"]
        size = info["bin_size"]
        if rom is None:
            check(False, "unused", "%s map missing Total ROM Size" % label)
            continue
        check(size == rom,
              "%s bin_size == Total ROM Size (%d)" % (label, size),
              "%s bin_size %d != Total ROM %d (map/bin not same build)"
              % (label, size, rom))
        check(size <= lr,
              "%s bin_size %d <= LR_IROM1 Size %d" % (label, size, lr),
              "%s bin_size %d > LR size %d (compression cannot grow)"
              % (label, size, lr))

    # ---- 门禁 2：RAM 预算 ----
    check(b_rw <= cml.BOOT_RAM_BUDGET,
          "boot Total RW %d <= %d" % (b_rw, cml.BOOT_RAM_BUDGET),
          "boot Total RW %d > budget %d"
          % (b_rw, cml.BOOT_RAM_BUDGET))
    check(a_rw <= cml.APP_RAM_BUDGET,
          "app Total RW %d <= %d" % (a_rw, cml.APP_RAM_BUDGET),
          "app Total RW %d > budget %d"
          % (a_rw, cml.APP_RAM_BUDGET))

    # ---- 门禁 3：半字对齐（STM32 Flash 编程粒度） ----
    check(b_bin % 2 == 0, "boot bin halfword aligned (%d B)" % b_bin,
          "boot bin length %d is odd" % b_bin)
    check(a_bin % 2 == 0, "app bin halfword aligned (%d B)" % a_bin,
          "app bin length %d is odd" % a_bin)

    # ---- 门禁 4：冻结公式交叉校验（实测尺寸 -> 布局常量） ----
    boot_part = cml.align_up(b_bin + cml.HARD_MARGIN,
                             cml.FLASH_PAGE_SIZE_HI)
    check(boot_part == layout["BOOT_PART_SIZE"],
          "BOOT_PART_SIZE 0x%X == align_up(%d + %d, 1024)"
          % (layout["BOOT_PART_SIZE"], b_bin, cml.HARD_MARGIN),
          "BOOT_PART_SIZE 0x%X != align_up(%d + %d, 1024) = 0x%X"
          % (layout["BOOT_PART_SIZE"], b_bin, cml.HARD_MARGIN, boot_part))

    app_base = layout["FLASH_BASE"] + layout["BOOT_PART_SIZE"]
    check(app_base == layout["APP_BASE"],
          "APP_BASE 0x%08X == FLASH_BASE + BOOT_PART_SIZE"
          % layout["APP_BASE"],
          "APP_BASE 0x%08X != FLASH_BASE + BOOT_PART_SIZE = 0x%08X"
          % (layout["APP_BASE"], app_base))

    app_cap = layout["APP_LIMIT_EXCL"] - layout["APP_BASE"]
    check(app_cap == layout["APP_CAPACITY"],
          "APP_CAPACITY 0x%X == APP_LIMIT_EXCL - APP_BASE"
          % layout["APP_CAPACITY"],
          "APP_CAPACITY 0x%X != APP_LIMIT_EXCL - APP_BASE = 0x%X"
          % (layout["APP_CAPACITY"], app_cap))

    # ---- 门禁 5：APP 容量硬余量 ----
    a_max = layout["APP_CAPACITY"] - cml.HARD_MARGIN
    check(a_bin <= a_max,
          "app bin %d <= APP_CAPACITY - %d = %d (margin %d B)"
          % (a_bin, cml.HARD_MARGIN, a_max, a_max - a_bin),
          "app bin %d > APP_CAPACITY - %d = %d"
          % (a_bin, cml.HARD_MARGIN, a_max))

    print("LAYOUT_COMPUTED boot_part=0x%X app_base=0x%08X "
          "app_capacity=0x%X boot_len=%d app_len=%d"
          % (boot_part, app_base, app_cap, b_bin, a_bin))
    print("audit-gates [%s]: %s" % (args.label,
                                    "PASS" if not failures else "FAIL"))
    return 0 if not failures else 1


# ---------------------------------------------------------------------------
# roundtrip：.iap 解包回 .bin 字节级验证
# ---------------------------------------------------------------------------

def cmd_roundtrip(args):
    for p in (args.iap, args.bin, args.layout):
        if not os.path.isfile(p):
            print("FAIL: file not found: %s" % p)
            return 1

    try:
        layout = ipk.parse_layout(args.layout)
    except ipk.LayoutError as exc:
        print("FAIL: cannot parse layout: %s" % exc)
        return 1

    with open(args.iap, "rb") as fh:
        blob = fh.read()
    with open(args.bin, "rb") as fh:
        image = fh.read()

    failures = []
    frames = ipk.parse_stream(blob)

    # ---- 帧序列结构：START(seq=0) + DATA(seq=1..N) + END(seq=N+1) ----
    starts = [f for f in frames if f["type"] == ipk.TYPE_START]
    datas = [f for f in frames if f["type"] == ipk.TYPE_DATA]
    ends = [f for f in frames if f["type"] == ipk.TYPE_END]

    if not (len(starts) == 1 and len(ends) == 1 and len(datas) >= 1):
        print("FAIL: frame structure: START=%d DATA=%d END=%d "
              "(expect 1/N/1)" % (len(starts), len(datas), len(ends)))
        return 1
    start, end = starts[0], ends[0]

    if start["seq"] != 0 or len(start["payload"]) != ipk.START_PAYLOAD_LEN:
        print("FAIL: START seq=%d payload=%d (expect 0/%d)"
              % (start["seq"], len(start["payload"]),
                 ipk.START_PAYLOAD_LEN))
        return 1

    # seq 连续 + offset 无洞 + 非尾包满 512
    expect_seq = 1
    expect_off = 0
    for i, f in enumerate(datas):
        if f["seq"] != expect_seq or f["offset"] != expect_off:
            print("FAIL: DATA[%d] seq=%d offset=%d (expect %d/%d)"
                  % (i, f["seq"], f["offset"], expect_seq, expect_off))
            return 1
        if i < len(datas) - 1 and len(f["payload"]) != ipk.MAX_PAYLOAD:
            print("FAIL: DATA[%d] non-tail payload %d != %d"
                  % (i, len(f["payload"]), ipk.MAX_PAYLOAD))
            return 1
        expect_seq += 1
        expect_off += len(f["payload"])

    if end["seq"] != expect_seq:
        print("FAIL: END seq=%d (expect %d)" % (end["seq"], expect_seq))
        return 1
    if end["payload"]:
        print("FAIL: END payload must be empty (%d B)"
              % len(end["payload"]))
        return 1

    # ---- manifest 字段校验 ----
    import struct
    target_base, image_size, image_crc, image_ver = struct.unpack(
        "<IIII", start["payload"])
    if target_base != layout["APP_BASE"]:
        print("FAIL: manifest target_base 0x%08X != APP_BASE 0x%08X"
              % (target_base, layout["APP_BASE"]))
        return 1
    if image_size != len(image):
        print("FAIL: manifest image_size %d != bin %d"
              % (image_size, len(image)))
        return 1
    if image_crc != ipk.crc32(image):
        print("FAIL: manifest image_crc32 0x%08X != crc32(bin) 0x%08X"
              % (image_crc, ipk.crc32(image)))
        return 1
    if end["offset"] != image_size:
        print("FAIL: END offset %d != image_size %d"
              % (end["offset"], image_size))
        return 1

    # ---- 字节级重组比对 ----
    rebuilt = b"".join(f["payload"] for f in datas)
    if rebuilt != image:
        print("FAIL: reassembled %d B != original %d B"
              % (len(rebuilt), len(image)))
        return 1

    print("iap: %s (%d B)" % (args.iap, len(blob)))
    print("bin: %s (%d B)" % (args.bin, len(image)))
    print("frames: START 1 + DATA %d + END 1" % len(datas))
    print("manifest: target_base=0x%08X image_size=%d "
          "image_crc32=0x%08X image_version=0x%08X"
          % (target_base, image_size, image_crc, image_ver))
    print("reassembled bytes == original bin (%d B) -> identical"
          % len(rebuilt))
    print("round-trip: PASS")
    return 0


# ---------------------------------------------------------------------------
# stale-scan：旧绝对地址散落扫描
# ---------------------------------------------------------------------------

STALE_PATTERNS = (
    # (pattern, description) —— hex 地址大小写不敏感
    (re.compile(r"0[xX]?0*0800[Ff]000"), "stale addr 0x0800F000"),
    (re.compile(r"0[xX]?0*0800[Ee][Cc]00"), "stale addr 0x0800EC00"),
    (re.compile(r"0[xX]?0*20004000"), "stale addr 0x20004000"),
    (re.compile(r"FLASH_APP_MAX_SIZE"), "stale symbol FLASH_APP_MAX_SIZE"),
    (re.compile(r"buff_size"), "stale symbol buff_size"),
)

STALE_EXTS = (".c", ".h", ".s", ".uvprojx", ".sct")

#：memory_layout.h、startup/system 文件、检查脚本
STALE_ALLOW_BASENAMES = ("memory_layout.h",)
STALE_ALLOW_PREFIXES = ("startup", "system")


def _allowed(rel_path):
    norm = rel_path.replace("\\", "/").lower()
    base = os.path.basename(norm)
    if base in STALE_ALLOW_BASENAMES:
        return True
    if base.startswith(STALE_ALLOW_PREFIXES):
        return True
    return False


def cmd_stale_scan(args):
    hits = []
    scanned = 0
    for root_dir in args.roots:
        if not os.path.isdir(root_dir):
            print("FAIL: root not found: %s" % root_dir)
            return 1
        for dirpath, dirnames, filenames in os.walk(root_dir):
            dirnames[:] = [d for d in dirnames
                           if d not in ("OBJ", "OBJ_BOOT_AUDIT",
                                        "OBJ_APP_AUDIT", "OBJ_BOOT_PROV",
                                        "OBJ_APP_PROV", "Listings",
                                        "Objects", "DebugConfig")]
            for name in filenames:
                if not name.lower().endswith(STALE_EXTS):
                    continue
                path = os.path.join(dirpath, name)
                rel = os.path.relpath(path)
                scanned += 1
                if _allowed(rel):
                    continue
                with open(path, "r", encoding="utf-8",
                          errors="replace") as fh:
                    for lineno, line in enumerate(fh, 1):
                        for pat, desc in STALE_PATTERNS:
                            if pat.search(line):
                                hits.append("%s:%d: %s" % (rel, lineno, desc))

    print("scanned %d source files under: %s"
          % (scanned, ", ".join(args.roots)))
    if hits:
        for h in hits:
            print("FAIL: %s" % h)
        print("stale-scan: FAIL (%d hit(s))" % len(hits))
        return 1
    print("stale-scan: PASS (no stale addresses/symbols outside allowlist)")
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Task 16 freeze gates: audit size gates, .iap "
                    "round-trip, stale address scan")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_audit = sub.add_parser(
        "audit-gates", help="FINAL_AUDIT size gates + layout formula check")
    p_audit.add_argument("--layout", required=True)
    p_audit.add_argument("--boot-map", required=True)
    p_audit.add_argument("--boot-bin", required=True)
    p_audit.add_argument("--app-map", required=True)
    p_audit.add_argument("--app-bin", required=True)
    p_audit.add_argument("--label", default="AUDIT")
    p_audit.set_defaults(func=cmd_audit_gates)

    p_rt = sub.add_parser(
        "roundtrip", help="unpack .iap frames and compare with original bin")
    p_rt.add_argument("--iap", required=True)
    p_rt.add_argument("--bin", required=True)
    p_rt.add_argument("--layout", required=True)
    p_rt.set_defaults(func=cmd_roundtrip)

    p_scan = sub.add_parser(
        "stale-scan", help="scan source roots for stale absolute addresses")
    p_scan.add_argument("roots", nargs="+")
    p_scan.set_defaults(func=cmd_stale_scan)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
