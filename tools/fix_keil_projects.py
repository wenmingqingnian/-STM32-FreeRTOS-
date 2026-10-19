"""
fix_keil_projects.py — Keil 工程配置批量修正

背景
----
芯片从 STM32F103CBT6（128 KiB Flash / 20 KiB SRAM，medium density）迁移到
STM32F103RCT6（256 KiB Flash / 48 KiB SRAM，high density）。所有 .uvprojx
里仍是旧器件与旧容量，必须同步，否则：
  1. 链接器按 128 KiB 布局，双槽的 Slot B（0x08021400 起）根本编不出来；
  2. 密度宏 STM32F10X_MD 与 HD 器件不匹配，HD 专属外设与向量表号不对；
  3. IROM 容量小于实际镜像时链接器报错或溢出到别的分区。

本脚本做三件事（幂等，可重复执行）：
  A. 全工程统一：Device -> STM32F103RC，STM32F10X_MD -> STM32F10X_HD，
     IRAM 0x5000 -> 0xC000（48 KiB），Flash 算法 128 -> 256 KiB；
  B. 按角色设定 IROM：
     - app_audit.uvprojx：改为双 Target（Slot A / Slot B），
       IROM(0x08003800,0x1DC00) 与 IROM(0x08021400,0x1DC00)，
       分别定义 VECT_TAB_OFFSET=0x3800 / 0x21400；
     - app_cb_provisional.uvprojx：Slot A 容量 0x1C000 -> 0x1DC00；
     - boot_*.uvprojx：保持 IROM(0x08000000,0x3800)；
     - project.uvprojx：单体演示工程，保持整片 0x40000。
  C. 输出校验报告。

所有布局常量都从 CONFIG/memory_layout.h 解析，不硬编码，避免与布局头
不一致（这是审查 F-02 的核心要求：工程配置必须与布局单一权威源对齐）。
"""
import re
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
USER = ROOT / "USER"

# ---- 从布局头解析（单一权威源） -------------------------------------------
LAYOUT = (ROOT / "CONFIG" / "memory_layout.h").read_text(encoding="utf-8")


def macro(name):
    m = re.search(r"#define\s+%s\s+(0x[0-9A-Fa-f]+)u?L?" % name, LAYOUT)
    if not m:
        raise SystemExit("memory_layout.h 缺少宏：%s" % name)
    return int(m.group(1), 16)


FLASH_SIZE = macro("BOARD_FLASH_SIZE_BYTES") if "BOARD_FLASH_SIZE_BYTES" in LAYOUT else 0x40000
SLOT_A = macro("SLOT_A_BASE")
SLOT_B = macro("SLOT_B_BASE")
SLOT_CAP = macro("SLOT_CAPACITY")
BOOT_SIZE = macro("BOOT_PART_SIZE")
FLASH_BASE = macro("FLASH_BASE")
SRAM_SIZE = 0xC000  # RCT6 48 KiB，见 CONFIG/board_profile.h BOARD_SRAM_SIZE_BYTES


def hexs(v):
    """Keil uvprojx 里StartAddress 用 0x + 大写 hex（无前导0）。"""
    return "0x%X" % v


def patch_common(s):
    """A. 与工程无关的全局修正（RCT6 / HD / 48 KiB SRAM / 256 KiB Flash）。"""
    s = s.replace("<Device>STM32F103CB</Device>",
                  "<Device>STM32F103RC</Device>")
    s = s.replace("<Device>STM32F103C8</Device>",
                  "<Device>STM32F103RC</Device>")
    # IRAM 容量：0x5000(20 KiB) -> 0xC000(48 KiB)
    s = s.replace("<Size>0x5000</Size>", "<Size>0xC000</Size>")
    s = s.replace("IRAM(0x20000000,0x5000)", "IRAM(0x20000000,0xC000)")
    # 密度宏：CB/C8 属 medium density 家族命名，但 RCT6(256K/48K) 是 high density
    s = s.replace("STM32F10X_MD", "STM32F10X_HD")
    # Flash 下载算法：128 -> 256 KiB
    s = s.replace("STM32F10x_128", "STM32F10x_256")
    s = s.replace("-FL020000", "-FL040000")
    # 启动文件：高密度向量表（76 个中断项 -> HD 版）
    s = s.replace("startup_stm32f10x_md.s", "startup_stm32f10x_hd.s")
    return s


def set_irom(s, base, size, tag="IROM"):
    """把 IROM 与 OCR_RVCT4 两个区块（Keil 实际生效的两处）设为 base/size。
    同时同步 <Cpu> 摘要串里的 IROM(...)，它是器件描述字符串，必须一致。
    已是目标值时原样返回（幂等），只有标签缺失才报错。"""
    pat = re.compile(r"<%s>.*?</%s>" % (tag, tag), re.S)
    if not pat.search(s):
        raise SystemExit("未找到 <%s> 区块" % tag)

    def repl(m):
        blk = m.group(0)
        blk = re.sub(r"<StartAddress>0x[0-9A-Fa-f]+</StartAddress>",
                     "<StartAddress>%s</StartAddress>" % hexs(base), blk)
        blk = re.sub(r"<Size>0x[0-9A-Fa-f]+</Size>",
                     "<Size>%s</Size>" % hexs(size), blk)
        return blk
    s = pat.sub(repl, s)

    # <Cpu> 摘要串：IROM(0x...,0x...)
    s = re.sub(r"IROM\(0x[0-9A-Fa-f]+,0x[0-9A-Fa-f]+\)",
               "IROM(0x%08X,0x%X)" % (base, size), s)
    return s


def set_define(s, define):
    """替换第一处 <Define>（Target 级宏定义）。若本来已是目标值则原样返回。"""
    if not re.search(r"<Define>", s):
        raise SystemExit("未找到 <Define>")
    def repl(m):
        return m.group(0).replace(m.group(1), define)
    return re.sub(r"<Define>([^<]*)</Define>", repl, s, count=1)


def extract_target(s, index=0):
    blocks = re.findall(r"[ \t]*<Target>.*?</Target>\n?", s, flags=re.S)
    return blocks[index]


# ---- 各工程的 IROM 设定 -----------------------------------------------------
def do_boot(path, boot_size):
    s = patch_common(path.read_text(encoding="utf-8"))
    s = set_irom(s, FLASH_BASE, boot_size, "IROM")
    s = set_irom(s, FLASH_BASE, boot_size, "OCR_RVCT4")
    path.write_text(s, encoding="utf-8")
    return "IROM(%s,0x%X) = Boot 分区" % (hexs(FLASH_BASE), boot_size)


def do_app_cb_provisional(path):
    s = patch_common(path.read_text(encoding="utf-8"))
    s = set_irom(s, SLOT_A, SLOT_CAP, "IROM")
    s = set_irom(s, SLOT_A, SLOT_CAP, "OCR_RVCT4")
    s = set_define(s, "STM32F10X_HD,USE_STDPERIPH_DRIVER,VECT_TAB_OFFSET=0x%08X"
                   % (SLOT_A - FLASH_BASE))
    path.write_text(s, encoding="utf-8")
    return ("IROM(%s,0x%X) + VECT_TAB_OFFSET=0x%08X（Slot A）"
            % (hexs(SLOT_A), SLOT_CAP, SLOT_A - FLASH_BASE))


def do_project_standalone(path):
    s = patch_common(path.read_text(encoding="utf-8"))
    s = set_irom(s, FLASH_BASE, FLASH_SIZE, "IROM")
    s = set_irom(s, FLASH_BASE, FLASH_SIZE, "OCR_RVCT4")
    path.write_text(s, encoding="utf-8")
    return "IROM(%s,0x%X) = 整片单体演示" % (hexs(FLASH_BASE), FLASH_SIZE)


def do_app_dual_target(path):
    """把单Target 的 APP 尺寸审计工程改成 Slot A / Slot B 双 Target。"""
    s = patch_common(path.read_text(encoding="utf-8"))

    # 先从原文切出 <Targets>...</Targets>，再基于原始块派生两个 Target，
    # 最后整体替换（不能拿改过的块去 partition 原文）。
    m = re.search(r"([ \t]*)<Targets>.*?</Targets>\n?", s, flags=re.S)
    if not m:
        raise SystemExit("app_audit: 未定位到 <Targets>")
    orig = m.group(0)
    a0 = extract_target(orig, 0)

    # 基线 Target（Slot A）
    a = a0.replace("<TargetName>App_SizeAudit</TargetName>",
                   "<TargetName>App_SlotA_SizeAudit</TargetName>")
    a = set_irom(a, SLOT_A, SLOT_CAP, "IROM")
    a = set_irom(a, SLOT_A, SLOT_CAP, "OCR_RVCT4")
    a = set_define(a, "STM32F10X_HD,USE_STDPERIPH_DRIVER,VECT_TAB_OFFSET=0x%08X"
                   % (SLOT_A - FLASH_BASE))
    a = re.sub(r"<OutputDirectory>.*?</OutputDirectory>",
               "<OutputDirectory>..\\\\OBJ_APP_SLOT_A\\\\</OutputDirectory>", a)
    a = re.sub(r"<ListingPath>.*?</ListingPath>",
               "<ListingPath>..\\\\OBJ_APP_SLOT_A\\\\</ListingPath>", a)
    a = re.sub(r"<OutputName>.*?</OutputName>",
               "<OutputName>range_hood_app_slot_a</OutputName>", a)

    # Slot B Target：同源复制，仅换 IROM / VECT_TAB_OFFSET / 输出目录与名字
    b = a
    b = b.replace("<TargetName>App_SlotA_SizeAudit</TargetName>",
                  "<TargetName>App_SlotB_SizeAudit</TargetName>")
    b = set_irom(b, SLOT_B, SLOT_CAP, "IROM")
    b = set_irom(b, SLOT_B, SLOT_CAP, "OCR_RVCT4")
    b = set_define(b, "STM32F10X_HD,USE_STDPERIPH_DRIVER,VECT_TAB_OFFSET=0x%08X"
                   % (SLOT_B - FLASH_BASE))
    b = b.replace("OBJ_APP_SLOT_A", "OBJ_APP_SLOT_B")
    b = re.sub(r"<OutputName>.*?</OutputName>",
               "<OutputName>range_hood_app_slot_b</OutputName>", b)

    path.write_text(s[:m.start()] + m.group(1) + "<Targets>\n"
                   + a + b + "  </Targets>\n" + s[m.end():], encoding="utf-8")
    return ("双 Target：Slot A IROM(%s,0x%X) / Slot B IROM(%s,0x%X)"
            % (hexs(SLOT_A), SLOT_CAP, hexs(SLOT_B), SLOT_CAP))


def main():
    plan = [
        ("app_audit.uvprojx", do_app_dual_target),
        ("app_cb_provisional.uvprojx", do_app_cb_provisional),
        ("boot_audit.uvprojx", lambda p: do_boot(p, BOOT_SIZE)),
        ("boot_cb_provisional.uvprojx", lambda p: do_boot(p, BOOT_SIZE)),
        ("project.uvprojx", do_project_standalone),
    ]
    print("布局来源 CONFIG/memory_layout.h：")
    print("  SLOT_A_BASE=0x%08X SLOT_B_BASE=0x%08X SLOT_CAPACITY=0x%X"
          % (SLOT_A, SLOT_B, SLOT_CAP))
    print("  BOOT_PART_SIZE=0x%X FLASH_SIZE=0x%X\n" % (BOOT_SIZE, FLASH_SIZE))
    for name, fn in plan:
        p = USER / name
        if not p.exists():
            print("  [skip] %s 不存在" % name)
            continue
        print("  [ok]   %-28s %s" % (name, fn(p)))
    print("\n请在 Keil 中重新打开工程并 Rebuild（工程 XML 已被改写，"
          "Keil 会提示工程已变更需重载）。")


if __name__ == "__main__":
    sys.exit(main())