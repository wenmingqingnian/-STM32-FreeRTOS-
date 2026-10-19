# -*- coding: utf-8 -*-
"""
[LEGACY] 固件CRC32预处理工具 —— 已被 iap_pack.py 取代

本工具属旧"单体 APP + 尾部 4B CRC"升级格式：把 CRC32 追加到 bin 末尾，
配套旧版 APP 内自擦写 IAP。新版独立 Bootloader 使用 IAP 帧协议
（START/DATA/END，帧内 CRC + manifest 整镜像 CRC），两种格式不兼容，
混用会导致升级失败。

新工具链（bin -> .iap 帧流 -> stop-and-wait 串口发送）：
  python tools/iap_pack.py --input OBJ_APP/range_hood_app.bin `
    --output OBJ_APP/range_hood_app.iap --layout CONFIG/memory_layout.h
  python tools/iap_uart_sender.py --input OBJ_APP/range_hood_app.iap `
    --port COM3 --kick

默认本工具拒绝产出旧格式文件（防止两套 CRC 格式混用）；确需旧行为
（如回滚验证旧 APP 链路）时显式加 --legacy 参数。

功能：读取原始bin文件，计算CRC32并附加到文件末尾
使用方法：python add_crc32.py <输入文件.bin> [输出文件.bin] --legacy

"""

import sys
import zlib
import struct
import os

def calculate_crc32(data):
    """计算数据的CRC32值（与STM32端一致）"""
    return zlib.crc32(data) & 0xFFFFFFFF

def add_crc_to_firmware(input_file, output_file=None):
    """
    为固件文件添加CRC32校验值
    
    参数:
        input_file: 原始bin文件路径
        output_file: 输出文件路径（可选，默认为 原文件名_crc.bin）
    """
    # 检查输入文件是否存在
    if not os.path.exists(input_file):
        print(f"错误：找不到输入文件 '{input_file}'")
        return False
    
    # 生成默认输出文件名
    if output_file is None:
        base_name = os.path.splitext(input_file)[0]
        output_file = f"{base_name}_crc.bin"
    
    try:
        # 读取原始固件数据
        with open(input_file, 'rb') as f:
            firmware_data = f.read()
        
        original_size = len(firmware_data)
        print(f"原始固件大小: {original_size} 字节")
        
        # 计算CRC32
        crc_value = calculate_crc32(firmware_data)
        print(f"CRC32校验值: 0x{crc_value:08X}")
        
        # 将CRC32转换为4字节小端序
        crc_bytes = struct.pack('<I', crc_value)
        
        # 拼接固件数据和CRC32
        output_data = firmware_data + crc_bytes
        
        # 写入输出文件
        with open(output_file, 'wb') as f:
            f.write(output_data)
        
        print(f"输出文件大小: {len(output_data)} 字节")
        print(f"成功！输出文件: '{output_file}'")
        print(f"\n请使用串口助手发送 '{output_file}' 文件进行固件升级。")
        return True
        
    except Exception as e:
        print(f"错误：{e}")
        return False

def main():
    print("=" * 50)
    print("[LEGACY] 固件CRC32预处理工具")
    print("=" * 50)

    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    legacy_confirmed = "--legacy" in sys.argv

    if not legacy_confirmed:
        print("\n本工具产出的\"尾部 4B CRC\"格式属于旧单体 APP 升级链路，")
        print("与新版独立 Bootloader 的 IAP 帧协议不兼容。")
        print("请改用新工具链：")
        print("  python tools/iap_pack.py --input <app.bin> "
              "--output <app.iap> --layout CONFIG/memory_layout.h")
        print("  python tools/iap_uart_sender.py --input <app.iap> --port COM3 --kick")
        print("\n确需旧格式（回滚验证旧链路）请显式加 --legacy 参数。")
        sys.exit(2)

    if len(args) < 1:
        print("\n使用方法:")
        print("  python add_crc32.py <输入文件.bin> [输出文件.bin] --legacy")
        print("\n示例:")
        print("  python add_crc32.py app.bin --legacy")
        print("  python add_crc32.py app.bin app_with_crc.bin --legacy")
        print("\n说明:")
        print("  - 输入文件：原始的APP固件bin文件")
        print("  - 输出文件：附加CRC32后的bin文件（用于发送到STM32）")
        print("  - 如不指定输出文件名，默认为 '原文件名_crc.bin'")
        return

    input_file = args[0]
    output_file = args[1] if len(args) > 1 else None

    print(f"\n输入文件: {input_file}")
    add_crc_to_firmware(input_file, output_file)

if __name__ == "__main__":
    main()
