# -*- coding: utf-8 -*-
"""
IAP PC 端工具单元测试

覆盖计划 "Test first" 三条：
  1. packer 对 APP bin 生成 START/DATA/END，CRC 与 C 端 golden vector 一致
     —— CRC32 用独立逐位反射参考实现（非 zlib/非查表）交叉验证，
        加 IEEE 标准测试向量 b"123456789"->0xCBF43926；
        C 端表驱动实现与 zlib 一致已由 Task 4 mock（test_crc32_stream.c
        85 检查）证明，本套件补齐 python 侧算法等价证据链；
  2. 8、511、512、513 B 和 APP_CAPACITY 边界分包正确
     —— 满包 512 切分、尾包奇数合法、非尾包必偶、容量边界接受/越界拒绝；
  3. sender 对 ACK、重复 ACK、NACK expected offset、超时重传和会话重启
     行为正确 —— FakeTransport 脚本化响应，不需要真实串口。

运行：仓库根目录
  python -m unittest tools.test_iap_tools -v
"""
import os
import struct
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import iap_pack
import iap_uart_sender as ius


# ---------------------------------------------------------------------------
# 辅助：独立 CRC32 参考实现（逐位反射，与 zlib/查表法互为交叉验证）
# ---------------------------------------------------------------------------

def crc32_reference(data):
    """IEEE CRC32 逐位实现：init 0xFFFFFFFF、反射多项式 0xEDB88320、终异或。"""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def make_ack(seq, offset):
    return iap_pack.build_frame(iap_pack.TYPE_ACK, seq, offset)


def make_nack(reason, expected_seq, expected_offset):
    return iap_pack.build_frame(iap_pack.TYPE_NACK, expected_seq,
                                expected_offset,
                                struct.pack("<I", reason))


NACK_NOT_ACTIVE = 1   # BootProto_NackReason_t（boot_protocol.h）


# ---------------------------------------------------------------------------
# packer 测试
# ---------------------------------------------------------------------------

class TestCrcGoldenVector(unittest.TestCase):
    """计划 Test first 1：CRC 与 C 端 golden vector 一致"""

    def test_ieee_standard_vector(self):
        # IEEE CRC32 官方测试向量：b"123456789" -> 0xCBF43926
        self.assertEqual(zlib.crc32(b"123456789") & 0xFFFFFFFF, 0xCBF43926)
        self.assertEqual(crc32_reference(b"123456789"), 0xCBF43926)
        self.assertEqual(iap_pack.crc32(b"123456789"), 0xCBF43926)

    def test_reference_matches_zlib_random_data(self):
        # 交叉验证参考实现与 zlib 在任意数据上一致（表驱动 C 端==zlib
        # 已由 Task 4 mock 证明，参考实现==zlib 在此证明）
        for n in (0, 1, 2, 7, 8, 511, 512, 513, 1000):
            data = bytes((i * 37 + 11) & 0xFF for i in range(n))
            self.assertEqual(crc32_reference(data),
                             zlib.crc32(data) & 0xFFFFFFFF,
                             "n=%d" % n)

    def test_frame_crc_matches_reference(self):
        # 帧内 frame_crc32 字段必须等于参考实现覆盖 [4,20)+payload 的值
        payload = bytes(range(20))
        frame = iap_pack.build_frame(iap_pack.TYPE_DATA, 7, 0x1234, payload)
        self.assertEqual(len(frame), 24 + len(payload))
        want = crc32_reference(frame[4:20] + payload)
        got = struct.unpack_from("<I", frame, 20)[0]
        self.assertEqual(got, want)


class TestLayoutParse(unittest.TestCase):
    """packer 从 memory_layout.h 读取 APP_BASE/CAPACITY，不手写地址"""

    def _write_layout(self, text):
        fd, path = tempfile.mkstemp(suffix=".h", text=True)
        with os.fdopen(fd, "w") as f:
            f.write(text)
        self.addCleanup(os.remove, path)
        return path

    def test_parse_app_base_capacity(self):
        path = self._write_layout(
            "#define FLASH_BASE 0x08000000UL\n"
            "#define APP_BASE 0x08003000UL\n"
            "#define APP_CAPACITY 0x0000C800UL\n")
        lay = iap_pack.parse_layout(path)
        self.assertEqual(lay["APP_BASE"], 0x08003000)
        self.assertEqual(lay["APP_CAPACITY"], 0x0000C800)

    def test_reject_expression_value(self):
        # 契约与 check_memory_layout.py 一致：只允许整数字面量
        path = self._write_layout("#define APP_BASE (FLASH_BASE + 0x3000UL)\n"
                                  "#define APP_CAPACITY 0xC800UL\n")
        with self.assertRaises(iap_pack.LayoutError):
            iap_pack.parse_layout(path)

    def test_reject_missing_constant(self):
        path = self._write_layout("#define APP_BASE 0x08003000UL\n")
        with self.assertRaises(iap_pack.LayoutError):
            iap_pack.parse_layout(path)


class TestPackImage(unittest.TestCase):
    """计划 Test first 2：分包边界"""

    APP_BASE = 0x08003000
    CAP = 0x200          # 测试用容量 512 B（> 最小镜像 8 B）

    def _pack(self, size, version=0x0100, cap=None):
        image = bytes((i * 31 + 7) & 0xFF for i in range(size))
        return image, iap_pack.pack_image(image, self.APP_BASE,
                                          cap or self.CAP,
                                          image_version=version)

    def _frames_of(self, frames):
        """解析出 [(type, seq, offset, payload)]，独立于构造路径"""
        out = []
        for raw in frames:
            self.assertTrue(raw.startswith(iap_pack.SOF))
            ftype = raw[5]
            seq, off = struct.unpack_from("<II", raw, 8)
            plen, = struct.unpack_from("<H", raw, 16)
            out.append((ftype, seq, off, raw[24:24 + plen]))
        return out

    def test_small_image_structure(self):
        # 10 B -> START + 1 DATA + END；manifest 字段与 zlib 全镜像 CRC 一致
        image, frames = self._pack(10)
        self.assertEqual(len(frames), 3)
        t0, t1, t2 = self._frames_of(frames)

        self.assertEqual(t0[0], iap_pack.TYPE_START)
        self.assertEqual(t0[1], 0)
        self.assertEqual(t0[2], 0)
        base, size, crc, ver = struct.unpack("<IIII", t0[3])
        self.assertEqual(base, self.APP_BASE)
        self.assertEqual(size, 10)
        self.assertEqual(crc, zlib.crc32(image) & 0xFFFFFFFF)
        self.assertEqual(ver, 0x0100)
        self.assertEqual(len(t0[3]), 16)

        self.assertEqual(t1[0], iap_pack.TYPE_DATA)
        self.assertEqual(t1[1], 1)
        self.assertEqual(t1[2], 0)
        self.assertEqual(t1[3], image)

        self.assertEqual(t2[0], iap_pack.TYPE_END)
        self.assertEqual(t2[1], 2)
        self.assertEqual(t2[2], 10)
        self.assertEqual(t2[3], b"")

    def test_min_image_8_bytes(self):
        _, frames = self._pack(8)
        self.assertEqual(len(frames), 3)

    def test_reject_image_below_8(self):
        with self.assertRaises(iap_pack.PackError):
            self._pack(7)

    def test_reject_image_above_capacity(self):
        with self.assertRaises(iap_pack.PackError):
            self._pack(self.CAP + 1)

    def test_accept_image_equal_capacity(self):
        _, frames = self._pack(self.CAP)
        # 512 B 容量 = 1 个满包 DATA
        parsed = self._frames_of(frames)
        self.assertEqual(len(parsed), 3)
        self.assertEqual(len(parsed[1][3]), 512)

    def test_chunk_boundaries(self):
        # 8、511、512、513、1024、1025 B 分包（协议最小镜像 8 B，故用 8
        # 代替 1；容量放宽到 2 KiB 以覆盖多包边界）：
        #   非尾包必为偶数满 512 包；尾包可为奇数
        cases = {
            8:    [(1, 8)],        # size -> [(包数, 尾包长)]
            511:  [(1, 511)],
            512:  [(1, 512)],
            513:  [(2, 1)],
            1024: [(2, 512)],
            1025: [(3, 1)],
        }
        for size, expect in cases.items():
            with self.subTest(size=size):
                _, frames = self._pack(size, cap=0x800)
                parsed = [p for p in self._frames_of(frames)
                          if p[0] == iap_pack.TYPE_DATA]
                self.assertEqual(len(parsed), expect[0][0],
                                 "DATA 包数（size=%d）" % size)
                self.assertEqual(len(parsed[-1][3]), expect[0][1],
                                 "尾包长度（size=%d）" % size)
                for i, (ftype, seq, off, payload) in enumerate(parsed):
                    self.assertEqual(seq, i + 1)          # seq 从 1 连续
                    self.assertEqual(off, i * 512)        # offset 累加
                    self.assertEqual(off % 2, 0)          # 2 字节对齐
                    if i < len(parsed) - 1:
                        self.assertEqual(len(payload), 512)  # 非尾包必满
                        self.assertEqual(len(payload) % 2, 0)  # 非尾包必偶

    def test_every_frame_crc_verified_by_reference(self):
        # 全部帧的 CRC 用逐位参考实现复算（golden vector 全覆盖）
        _, frames = self._pack(513, cap=0x800)
        for raw in frames:
            plen, = struct.unpack_from("<H", raw, 16)
            want = crc32_reference(raw[4:20] + raw[24:24 + plen])
            got, = struct.unpack_from("<I", raw, 20)
            self.assertEqual(got, want)


class TestFrameParse(unittest.TestCase):
    """parse_stream：SOF 同步跳噪声 + CRC 校验（sender 接收路径）"""

    def test_parse_ack(self):
        ack = make_ack(3, 0x200)
        frames = iap_pack.parse_stream(b"\x00\x11noise" + ack + b"tail")
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["type"], iap_pack.TYPE_ACK)
        self.assertEqual(frames[0]["seq"], 3)
        self.assertEqual(frames[0]["offset"], 0x200)
        self.assertEqual(frames[0]["payload"], b"")

    def test_parse_nack(self):
        nack = make_nack(NACK_NOT_ACTIVE, 0, 0)
        frames = iap_pack.parse_stream(nack)
        reason, = struct.unpack("<I", frames[0]["payload"])
        self.assertEqual(reason, NACK_NOT_ACTIVE)

    def test_parse_rejects_corrupted_crc(self):
        ack = bytearray(make_ack(3, 0x200))
        ack[8] ^= 0xFF   # 破坏 seq 字段 -> CRC 失配
        self.assertEqual(iap_pack.parse_stream(bytes(ack)), [])

    def test_parse_two_frames_in_one_stream(self):
        data = make_ack(0, 0) + make_ack(1, 512)
        frames = iap_pack.parse_stream(data)
        self.assertEqual([f["seq"] for f in frames], [0, 1])


# ---------------------------------------------------------------------------
# sender 测试（FakeTransport，无需真实串口）
# ---------------------------------------------------------------------------

class FakeTransport:
    """脚本化串口：write() 记录发送；read() 逐条弹出脚本项（None=超时）"""

    def __init__(self, script):
        self.script = list(script)
        self.written = []

    def write(self, data):
        self.written.append(bytes(data))

    def read(self, timeout):
        if not self.script:
            return b""
        item = self.script.pop(0)
        return item if item is not None else b""


def three_frame_session():
    """S(10B)+D(1)+E 的最小合法帧序列"""
    image = bytes(range(10))
    return iap_pack.pack_image(image, 0x08003000, 0xC800, image_version=1)


def frames_seq(frames):
    out = []
    for raw in frames:
        seq, = struct.unpack_from("<I", raw, 8)
        out.append(seq)
    return out


class TestSender(unittest.TestCase):
    """计划 Test first 3：sender 状态机五类行为"""

    def _run(self, script, frames=None, retries=3):
        frames = frames if frames is not None else three_frame_session()
        t = FakeTransport(script)
        sender = ius.IapSender(t, frames, timeout=0.01, retries=retries)
        stats = sender.run()
        return t, stats

    def test_all_acks_advance(self):
        # ACK：每帧确认一次，stop-and-wait 顺序推进
        t, stats = self._run([make_ack(0, 0), make_ack(1, 10), make_ack(2, 10)])
        self.assertEqual(frames_seq(t.written), [0, 1, 2])
        self.assertEqual(stats["frames_sent"], 3)
        self.assertEqual(stats["retransmits"], 0)
        self.assertTrue(stats["ok"])

    def test_duplicate_ack_is_idempotent(self):
        # 重复 ACK：seq 落后（上一帧的确认又来一次）不回退、不跳帧，
        # sender 重发当前帧直到收到它自己的确认为止
        t, stats = self._run([make_ack(0, 0),
                             make_ack(0, 0),      # 迟到的重复确认
                             make_ack(1, 10),
                             make_ack(2, 10)])
        # f1 收到旧 ACK -> 重发 f1 -> 收到 ACK1 -> 推进 f2
        self.assertEqual(frames_seq(t.written), [0, 1, 1, 2])
        self.assertEqual(stats["retransmits"], 1)
        self.assertTrue(stats["ok"])

    def test_nack_expected_offset_rolls_back(self):
        # NACK expected offset：Boot 回填期望 seq/offset -> 原地重发该帧
        t, stats = self._run([make_ack(0, 0),
                             make_nack(6, 1, 10),   # 期望 seq1/offset10
                             make_ack(1, 10),
                             make_ack(2, 10)])
        # f1 被拒（NACK 期望恰为 seq1）-> 原地重发 f1
        self.assertEqual(frames_seq(t.written), [0, 1, 1, 2])
        self.assertTrue(stats["ok"])

    def test_nack_lost_ack_reverts_to_expected_frame(self):
        # ACK 丢失：Boot 仍在等已发帧（expected seq 落后于已发）
        t, stats = self._run([make_ack(0, 0),
                             make_ack(1, 10),
                             make_nack(4, 1, 10),   # Boot 说在等 seq1（ACK1 丢了）
                             make_ack(1, 10),
                             make_ack(2, 10)])
        # f2 发出后收到 NACK(ES=1) -> 回退重发 f1 -> ACK -> 重发 f2
        self.assertEqual(frames_seq(t.written), [0, 1, 2, 1, 2])
        self.assertTrue(stats["ok"])

    def test_timeout_retransmit(self):
        # 超时重传：无响应（None）-> 重发当前帧，收到 ACK 后继续
        t, stats = self._run([None,
                             make_ack(0, 0),
                             make_ack(1, 10),
                             make_ack(2, 10)],
                            retries=3)
        self.assertEqual(frames_seq(t.written), [0, 0, 1, 2])
        self.assertEqual(stats["retransmits"], 1)
        self.assertTrue(stats["ok"])

    def test_retry_exhausted_write_count(self):
        # 重传预算用尽 -> SendError；初发 1 次 + 重传 retries 次
        frames = three_frame_session()
        t = FakeTransport([None, None, None])
        sender = ius.IapSender(t, frames, timeout=0.01, retries=2)
        with self.assertRaises(ius.SendError):
            sender.run()
        self.assertEqual(frames_seq(t.written), [0, 0, 0])

    def test_session_restart_from_start(self):
        # 会话重启：设备中途复位（会话丢失）-> NACK(NOT_ACTIVE, ES=0)
        # -> 从 START 重新开始整个会话
        t, stats = self._run([make_ack(0, 0),
                             make_ack(1, 10),
                             make_nack(NACK_NOT_ACTIVE, 0, 0),
                             make_ack(0, 0),
                             make_ack(1, 10),
                             make_ack(2, 10)])
        # f2 后 NACK(ES=0) -> 回到 f0 -> 全部重发
        self.assertEqual(frames_seq(t.written), [0, 1, 2, 0, 1, 2])
        self.assertTrue(stats["ok"])

    def test_restart_budget(self):
        # 会话重启也受重传预算约束，防止无限循环
        frames = three_frame_session()
        t = FakeTransport([make_nack(NACK_NOT_ACTIVE, 0, 0)] * 10)
        sender = ius.IapSender(t, frames, timeout=0.01, retries=2)
        with self.assertRaises(ius.SendError):
            sender.run()

    def test_end_ack_completes(self):
        # END 的 ACK 到达即会话成功（Boot 随后复位属预期）
        t, stats = self._run([make_ack(0, 0), make_ack(1, 10), make_ack(2, 10)])
        self.assertTrue(stats["ok"])
        self.assertEqual(stats["bytes_payload"], 10)


if __name__ == "__main__":
    unittest.main()
