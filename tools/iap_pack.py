# -*- coding: utf-8 -*-
"""
iap_pack.py -- IAP 固件打包器

职责（设计文档 §5.2 帧协议，与 BOOT/boot_protocol.c 逐字段对齐）：
  1. 从 CONFIG/memory_layout.h 解析 APP_BASE/APP_CAPACITY（只允许整数
     字面量，契约与 tools/check_memory_layout.py 一致，不手写地址）；
  2. 校验镜像尺寸 8 <= size <= APP_CAPACITY（与 Boot_ImageValidate 同界）；
  3. 生成 START/DATA/END 帧序列：
     - START(seq=0) 携带 16 B manifest（target_base/image_size/
       image_crc32/image_version，全部小端）；
     - DATA(seq=1..N) 满包 512 B 切分，offset 递增 2 字节对齐；
       只有尾包允许奇数长度（Boot 半字编程约束）；
     - END(seq=N+1) payload_len=0，offset=image_size；
     - frame_crc32 = IEEE CRC32（zlib.crc32 == C 端查表实现，
       Task 4 mock 证明；逐位参考实现交叉验证见 test_iap_tools.py），
       覆盖 [4,20) 头字段 + payload，不含 SOF 与 CRC 字段本身。

用法：
  python tools/iap_pack.py --input OBJ_APP/range_hood_app.bin `
    --output OBJ_APP/range_hood_app.iap --layout CONFIG/memory_layout.h

单元测试：tools/test_iap_tools.py（python -m unittest tools.test_iap_tools -v）
"""
import argparse
import re
import struct
import sys
import zlib

# ---------------------------------------------------------------------------
# 协议常量（boot_protocol.h 的 python 侧镜像，改动必须两处同步）
# ---------------------------------------------------------------------------

SOF = b"IAP1"
PROTOCOL_VERSION = 1
HEADER_LEN = 24
MAX_PAYLOAD = 512
START_PAYLOAD_LEN = 16

TYPE_START = 0x01
TYPE_DATA = 0x02
TYPE_END = 0x03
TYPE_ACK = 0x04     # 仅 Boot->主机方向
TYPE_NACK = 0x05     # 仅 Boot->主机方向

MIN_IMAGE_SIZE = 8  # 至少容纳 MSP+PC（设计 §5.5）


class LayoutError(Exception):
    """memory_layout.h 解析失败（缺常量/非字面量）。"""


class PackError(Exception):
    """镜像打包失败（尺寸越界等）。"""


def crc32(data):
    """IEEE CRC32（zlib.crc32 == C 端表驱动，见 crc32.c 表修正说明）"""
    return zlib.crc32(data) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# memory_layout.h 解析（只允许整数字面量，与 check_memory_layout.py 同契约）
# ---------------------------------------------------------------------------

_RE_DEFINE = re.compile(r"^\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)\s+(.+?)\s*$")
_RE_COMMENT = re.compile(r"/\*.*?\*/|//.*$")
_RE_LITERAL = re.compile(r"^(0[xX][0-9A-Fa-f]+|\d+)([uUlL]*)$")

_LAYOUT_REQUIRED = ("APP_BASE", "APP_CAPACITY")


def parse_layout(path):
    """提取 APP_BASE/APP_CAPACITY。任何缺失或非字面量值都抛 LayoutError。"""
    values = {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError as e:
        raise LayoutError("cannot read layout file %s: %s" % (path, e))

    for line in lines:
        m = _RE_DEFINE.match(line)
        if not m:
            continue
        name, raw = m.group(1), m.group(2)
        if name not in _LAYOUT_REQUIRED:
            continue
        value = _RE_COMMENT.sub("", raw).strip()
        lit = _RE_LITERAL.match(value)
        if not lit:
            raise LayoutError(
                "non-literal macro value for %s: '%s' "
                "(contract: integer literals only, see check_memory_layout.py)"
                % (name, raw))
        # 剥离 U/L 后缀（0x08003000UL -> 0x08003000）再取值；
        # 不用 int(x,0)：'0'前导十进制会被当非法八进制拒绝
        text = lit.group(1)
        base = 16 if text.lower().startswith("0x") else 10
        values[name] = int(text, base)

    for name in _LAYOUT_REQUIRED:
        if name not in values:
            raise LayoutError("layout file %s missing constant %s"
                              % (path, name))
    if values["APP_CAPACITY"] <= 0:
        raise LayoutError("APP_CAPACITY must be positive")
    return values


# ---------------------------------------------------------------------------
# 帧构造与解析
# ---------------------------------------------------------------------------

def build_frame(ftype, seq, offset, payload=b""):
    """构造一帧（主机方向 START/DATA/END 与 Boot 方向 ACK/NACK 同构）。"""
    if len(payload) > MAX_PAYLOAD:
        raise PackError("payload %d exceeds MAX_PAYLOAD %d"
                        % (len(payload), MAX_PAYLOAD))
    hdr = bytearray(HEADER_LEN)
    hdr[0:4] = SOF
    hdr[4] = PROTOCOL_VERSION
    hdr[5] = ftype
    struct.pack_into("<H", hdr, 6, HEADER_LEN)
    struct.pack_into("<I", hdr, 8, seq)
    struct.pack_into("<I", hdr, 12, offset)
    struct.pack_into("<H", hdr, 16, len(payload))
    struct.pack_into("<H", hdr, 18, 0)          # reserved 必须为 0
    # frame_crc32 覆盖 [4,20) + payload（不含 SOF 与 CRC 字段本身）
    struct.pack_into("<I", hdr, 20, crc32(bytes(hdr[4:20]) + payload))
    return bytes(hdr) + payload


def parse_stream(data):
    """从字节流解析全部完整帧（sender 接收路径）。

    - SOF 扫描同步：SOF 前的噪声字节静默丢弃；
    - 每帧校验 version==1、header_len==24、payload_len<=512、reserved==0、
      frame_crc32；任一不符该帧整体丢弃（含 payload），从下一字节继续
      找 SOF（与 Boot 解析器"坏帧重同步"语义一致）；
    - 流尾不足一帧时剩余字节丢弃（半帧由上层超时重传兜底）。
    """
    frames = []
    i = 0
    n = len(data)
    while True:
        pos = data.find(SOF, i)
        if pos < 0 or n - pos < HEADER_LEN:
            break
        version = data[pos + 4]
        ftype = data[pos + 5]
        hlen, = struct.unpack_from("<H", data, pos + 6)
        seq, offset = struct.unpack_from("<II", data, pos + 8)
        plen, reserved = struct.unpack_from("<HH", data, pos + 16)
        if (version != PROTOCOL_VERSION or hlen != HEADER_LEN
                or plen > MAX_PAYLOAD):
            i = pos + 1      # 坏头：从下一字节继续找 SOF
            continue
        end = pos + HEADER_LEN + plen
        if end > n:
            break            # 半帧
        payload = data[pos + HEADER_LEN:end]
        if reserved != 0:
            i = pos + 1
            continue
        want = struct.unpack_from("<I", data, pos + 20)[0]
        if crc32(data[pos + 4:pos + 20] + payload) != want:
            i = pos + 1      # CRC 失配：丢弃整帧（含 payload）
            continue
        frames.append({
            "type": ftype,
            "seq": seq,
            "offset": offset,
            "payload": payload,
        })
        i = end
    return frames


# ---------------------------------------------------------------------------
# 镜像打包
# ---------------------------------------------------------------------------

def chunk_image(image):
    """512 B 满包切分：非尾包必为偶数满 512，尾包可为奇数。"""
    if len(image) < MIN_IMAGE_SIZE:
        raise PackError("image %d B < minimum %d B (MSP+PC vectors)"
                        % (len(image), MIN_IMAGE_SIZE))
    return [image[i:i + MAX_PAYLOAD]
            for i in range(0, len(image), MAX_PAYLOAD)]


def pack_image(image, app_base, app_capacity, image_version=0x0100):
    """生成 START + DATA* + END 帧列表（bytes 列表，seq 0..N 连续）。

    image_size 必须满足 8 <= size <= app_capacity（与 Boot_ImageValidate
    和 BootProto START 校验同界，越界在打包期即拒绝）。
    """
    size = len(image)
    if size < MIN_IMAGE_SIZE:
        raise PackError("image %d B < minimum %d B" % (size, MIN_IMAGE_SIZE))
    if size > app_capacity:
        raise PackError("image %d B > APP_CAPACITY %d B" % (size, app_capacity))
    if app_base % 0x80 != 0:
        # VTOR 对齐要求由布局保证，此处防御（不应触发）
        raise PackError("APP_BASE 0x%08X not 0x80-aligned" % app_base)

    image_crc = crc32(image)
    manifest = struct.pack("<IIII", app_base, size, image_crc, image_version)
    frames = [build_frame(TYPE_START, 0, 0, manifest)]

    seq = 1
    offset = 0
    for chunk in chunk_image(image):
        frames.append(build_frame(TYPE_DATA, seq, offset, chunk))
        seq += 1
        offset += len(chunk)

    frames.append(build_frame(TYPE_END, seq, offset))
    return frames


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Pack APP binary into IAP START/DATA/END frame stream")
    parser.add_argument("--input", required=True, help="APP .bin 文件")
    parser.add_argument("--output", required=True, help="输出 .iap 帧流文件")
    parser.add_argument("--layout", required=True,
                        help="CONFIG/memory_layout.h（APP_BASE/CAPACITY 来源）")
    parser.add_argument("--version", type=lambda x: int(x, 0), default=0x0100,
                        help="image_version（仅记录，默认 0x0100）")
    args = parser.parse_args(argv)

    try:
        layout = parse_layout(args.layout)
    except LayoutError as e:
        print("error: %s" % e, file=sys.stderr)
        return 2

    try:
        with open(args.input, "rb") as f:
            image = f.read()
    except OSError as e:
        print("error: cannot read input: %s" % e, file=sys.stderr)
        return 2

    try:
        frames = pack_image(image, layout["APP_BASE"],
                            layout["APP_CAPACITY"], args.version)
    except PackError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1

    blob = b"".join(frames)
    try:
        with open(args.output, "wb") as f:
            f.write(blob)
    except OSError as e:
        print("error: cannot write output: %s" % e, file=sys.stderr)
        return 2

    n_data = len(frames) - 2
    print("APP_BASE      = 0x%08X" % layout["APP_BASE"])
    print("APP_CAPACITY  = %d B" % layout["APP_CAPACITY"])
    print("image_size    = %d B" % len(image))
    print("image_crc32   = 0x%08X" % crc32(image))
    print("image_version = 0x%08X" % args.version)
    print("frames        = %d (START 1 + DATA %d + END 1)"
          % (len(frames), n_data))
    print("output        = %s (%d B)" % (args.output, len(blob)))
    print("next          = python tools/iap_uart_sender.py --input %s"
          % args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
