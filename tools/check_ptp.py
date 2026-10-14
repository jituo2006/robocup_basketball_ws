#!/usr/bin/env python3
"""Passively inspect MID360 PTP timestamps; never bind/steal the driver UDP ports."""
import argparse
from pathlib import Path
import socket
import struct
import subprocess
import time


def decode(frame, source_ip, ports):
    if len(frame) < 14 or frame[12:14] != b'\x08\x00':
        return None
    ip = frame[14:]
    if len(ip) < 20 or ip[0] >> 4 != 4 or ip[9] != 17:
        return None
    ihl = (ip[0] & 15) * 4
    total = int.from_bytes(ip[2:4], 'big')
    if ihl < 20 or total > len(ip) or total < ihl + 8:
        return None
    if int.from_bytes(ip[6:8], 'big') & 0x3fff:  # no fragmented datagrams
        return None
    if socket.inet_ntoa(ip[12:16]) != source_ip:
        return None
    udp = ip[ihl:total]
    src, dest, length, _ = struct.unpack_from('!HHHH', udp)
    if dest not in ports or length < 44 or length > len(udp):
        return None
    payload = udp[8:length]
    if payload[0] != 0 or int.from_bytes(payload[1:3], 'little') != len(payload):
        return None
    # Official MID360 header: data_type byte 10, time_type byte 11, timestamp 28.
    data_type, sync_type = payload[10:12]
    expected = 'imu' if dest == ports[1] else 'lidar'
    if (expected == 'imu' and data_type != 0) or (expected == 'lidar' and data_type not in (1, 2, 3)):
        return None
    stamp = int.from_bytes(payload[28:36], 'little') / 1e9
    return expected, sync_type, stamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    try:
        interface = Path('/etc/robocup/ptp_interface').read_text().strip()
    except OSError:
        interface = 'enp114s0'
    parser.add_argument('--interface', default=interface)
    parser.add_argument('--lidar-ip', default='192.168.1.138')
    parser.add_argument('--lidar-port', type=int, default=56301)
    parser.add_argument('--imu-port', type=int, default=56401)
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('seconds must be positive')
    for service in ('robocup-ptp4l', 'robocup-phc2sys'):
        result = subprocess.run(['systemctl', 'is-active', f'{service}@{args.interface}.service'],
                                capture_output=True, text=True)
        print(f'{service}: {result.stdout.strip()}')
        if result.returncode:
            return 1
    try:
        linked = Path('/sys/class/net', args.interface, 'carrier').read_text().strip() == '1'
    except OSError:
        linked = False
    if not linked:
        print('雷达网口无链路；PTP 服务等待连接，尚未验证雷达同步。')
        return 2
    try:
        offset = int(Path('/etc/robocup/livox_ptp_utc_offset').read_text().strip())
        if not 0 <= offset <= 100:
            raise ValueError('invalid UTC offset')
        raw = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
        raw.bind((args.interface, 0))
    except (OSError, ValueError) as exc:
        print(f'检查失败（需要 sudo 权限）：{exc}')
        return 1
    raw.settimeout(.2)
    samples = {'lidar': [], 'imu': []}
    deadline = time.monotonic() + args.seconds
    with raw:
        while time.monotonic() < deadline:
            try:
                frame, address = raw.recvfrom(65535)
            except socket.timeout:
                continue
            if address[2] == socket.PACKET_OUTGOING:
                continue
            result = decode(frame, args.lidar_ip, (args.lidar_port, args.imu_port))
            if result:
                name, mode, stamp = result
                samples[name].append((mode, time.time() - (stamp-offset)))
    success = True
    for name, data in samples.items():
        if not data:
            print(f'{name}: 未收到数据（检查雷达电源和 Livox 驱动）')
            success = False
            continue
        modes = sorted({mode for mode, age in data})
        ages = [age for mode, age in data if mode == 1]
        valid = len(ages) >= 5 and modes == [1] and all(-.05 <= age <= .5 for age in ages)
        print(f'{name}: packets={len(data)}, timestamp_type={modes}, PTP={valid}')
        if ages:
            print(f'  转为 UTC 后的收包时间差：{min(ages)*1000:.3f}..{max(ages)*1000:.3f} ms')
        success &= valid
    print('该时间差包含采样/网络/收包延迟，并非雷达伺服同步精度。')
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
