# -*- coding: utf-8 -*-
"""
iap_uart_sender.py -- IAP 串口发送器

stop-and-wait 状态机（设计文档 §5.2：主机收到 ACK 前不得发送下一包）：

  发送 frames[i] -> 等响应 -> 按响应转移：
    ACK(seq == frames[i].seq)   帧确认 -> i+=1，重传预算重置
    ACK(seq <  frames[i].seq)   重复/迟到确认 -> 忽略，重发当前帧（幂等，
                                不回退不跳帧）
    ACK(seq >  frames[i].seq)   Boot 跳号 -> 协议违例，报错
    NACK(expectedSeq=ES)        ES == i：当前帧被拒 -> 原地重发；
                                ES <  i：ACK 丢失，Boot 在等 ES 帧 ->
                                回退 i=ES 重发（会话重启时 ES=0，
                                即从 START 重新开始整个会话）
    超时（无响应）              重发当前帧，消耗一次重传预算

  重传预算：per-frame（默认 3），帧被正确 ACK 推进时重置；预算用尽抛
  SendError。NACK 回退/原地重发同样消耗预算，防止与故障设备无限循环。

传输层抽象：Transport（write/read），测试注入 FakeTransport（见
test_iap_tools.py），CLI 用 pyserial SerialTransport。OTA 不在本 Task。

用法：
  python tools/iap_uart_sender.py --input OBJ_APP/range_hood_app.iap `
    --port COM3 [--baud 115200] [--timeout 1.0] [--retries 3] [--kick]

  --kick：先发送文本命令 "IAP\\r\\n" 让运行中的 APP 复位进 Boot
（APP 侧 app_tasks.c 命令轮询），等待 2 s 后再开始 START 握手。
"""
import argparse
import struct
import sys
import time

import iap_pack


class SendError(Exception):
    """发送失败（重传预算用尽/协议违例）。"""


class Transport(object):
    """传输层接口：write(bytes)/read(timeout)->bytes（空=本轮无数据）"""

    def write(self, data):
        raise NotImplementedError

    def read(self, timeout):
        raise NotImplementedError


class SerialTransport(Transport):
    """pyserial 包装（延迟导入：单元测试环境无需安装 pyserial）"""

    def __init__(self, port, baud):
        try:
            import serial   # pyserial
        except ImportError:
            raise SendError("pyserial not installed: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0)

    def write(self, data):
        self.ser.write(data)
        self.ser.flush()

    def read(self, timeout):
        self.ser.timeout = timeout
        data = self.ser.read(4096)
        return bytes(data)

    def close(self):
        self.ser.close()


class IapSender(object):
    """stop-and-wait 发送器：frames 为 iap_pack.pack_image 的输出"""

    def __init__(self, transport, frames, timeout=1.0, retries=3):
        if not frames:
            raise SendError("no frames to send")
        self.transport = transport
        self.frames = frames
        self.timeout = timeout
        self.retries = retries
        # 帧序号 <-> 帧体（pack_image 保证 seq 0..N-1 与列表索引一致；
        # 防御性映射：按帧头实际 seq 建索引，不依赖调用方顺序）
        self._seq_of = []
        for raw in frames:
            seq, = struct.unpack_from("<I", raw, 8)
            self._seq_of.append(seq)
        self._index_of_seq = {s: i for i, s in enumerate(self._seq_of)}

    def _frame_seq(self, i):
        return self._seq_of[i]

    def run(self):
        """执行整个会话，返回统计 dict；失败抛 SendError。"""
        i = 0
        n = len(self.frames)
        retries_left = self.retries
        written = set()           # 已发送过的帧索引（重复写=重传）
        stats = {
            "ok": False,
            "frames_sent": 0,       # 被确认推进的帧数
            "writes": 0,            # 总 write 次数（含重传）
            "retransmits": 0,       # 重传次数（含 NACK 回退重发）
            "duplicate_acks": 0,
            "bytes_payload": 0,     # DATA 负载累计字节
        }

        while i < n:
            if i in written:
                stats["retransmits"] += 1   # 同一帧再次写=重传
            written.add(i)
            self.transport.write(self.frames[i])
            stats["writes"] += 1

            rx = self.transport.read(self.timeout)
            responses = iap_pack.parse_stream(rx)  # 半帧不缓存：整帧重传兜底
            prev_i = i

            for resp in responses:
                rtype = resp["type"]
                if rtype == iap_pack.TYPE_ACK:
                    cur = self._frame_seq(i)
                    if resp["seq"] == cur:
                        # 当前帧确认：推进（DATA 帧累计负载字节）
                        if self.frames[i][5] == iap_pack.TYPE_DATA:
                            plen, = struct.unpack_from("<H", self.frames[i], 16)
                            stats["bytes_payload"] += plen
                        stats["frames_sent"] += 1
                        i += 1
                        retries_left = self.retries
                    elif resp["seq"] < cur:
                        # 重复/迟到确认：不回退不跳帧，下轮重发当前帧
                        stats["duplicate_acks"] += 1
                    else:
                        raise SendError(
                            "protocol violation: ACK seq %d jumps ahead of %d"
                            % (resp["seq"], cur))
                elif rtype == iap_pack.TYPE_NACK:
                    es = resp["seq"]           # NACK 携带期望 seq/offset
                    if es not in self._index_of_seq or es > self._frame_seq(i):
                        raise SendError(
                            "protocol violation: NACK expected seq %d invalid"
                            % es)
                    i = self._index_of_seq[es]   # 回退（ES==i 时原地重发）
                # 其它类型响应：忽略

            if i == prev_i:
                # 无推进（超时/重复确认/被忽略响应/NACK 原地）：
                # 重发当前帧，消耗一次重传预算（防故障设备无限循环）
                retries_left -= 1
                if retries_left < 0:
                    raise SendError(
                        "no progress on seq %d after %d retries"
                        % (self._frame_seq(prev_i), self.retries))

        stats["ok"] = True
        return stats


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Send .iap frame stream over UART (stop-and-wait)")
    parser.add_argument("--input", required=True, help="iap_pack.py 输出的 .iap")
    parser.add_argument("--port", required=True, help="串口名，如 COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="单帧 ACK 超时秒数")
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--kick", action="store_true",
                        help="先发 'IAP\\r\\n' 让 APP 复位进 Boot（等 2 s）")
    args = parser.parse_args(argv)

    try:
        with open(args.input, "rb") as f:
            blob = f.read()
    except OSError as e:
        print("error: cannot read input: %s" % e, file=sys.stderr)
        return 2

    frames = []
    pos = 0
    while pos < len(blob):
        plen, = struct.unpack_from("<H", blob, pos + 16)
        frames.append(blob[pos:pos + iap_pack.HEADER_LEN + plen])
        pos += iap_pack.HEADER_LEN + plen
    if not frames:
        print("error: empty .iap file", file=sys.stderr)
        return 2

    try:
        t = SerialTransport(args.port, args.baud)
    except SendError as e:
        print("error: %s" % e, file=sys.stderr)
        return 2

    try:
        if args.kick:
            print("kick: sending 'IAP\\r\\n' to running APP ...")
            t.write(b"IAP\r\n")
            time.sleep(2.0)
        print("sending %d frames to %s @ %d baud (stop-and-wait) ..."
              % (len(frames), args.port, args.baud))
        sender = IapSender(t, frames, timeout=args.timeout,
                           retries=args.retries)
        stats = sender.run()
    except SendError as e:
        print("FAILED: %s" % e, file=sys.stderr)
        return 1
    finally:
        t.close()

    print("OK: frames_sent=%d writes=%d retransmits=%d duplicate_acks=%d "
          "payload=%d B"
          % (stats["frames_sent"], stats["writes"], stats["retransmits"],
             stats["duplicate_acks"], stats["bytes_payload"]))
    print("device now reboots into the new image (Boot re-validates it)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
