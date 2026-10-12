"""Verify real clock conversion and packet validation without robot hardware."""
import importlib.util
from pathlib import Path
import shutil
import os
import socket
import struct
import subprocess

import pytest

ROOT = Path(__file__).parents[3]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tai_to_utc_and_unsynchronised_packets(tmp_path):
    compiler = shutil.which('c++')
    assert compiler, 'C++ compiler needed to test the deployed Livox conversion'
    source = tmp_path / 'clock.cpp'
    source.write_text('''#include "livox_ptp_clock.hpp"
#include <cassert>
int main(int argc, char** argv) {
  auto absent = rb_ptp::ReadConfig(std::string(argv[1]) + ".absent");
  assert(!absent.enabled);
  auto config = rb_ptp::ReadConfig(argv[1]);
  assert(config.enabled && config.offset_ns == 37000000000ULL);
  const uint64_t utc = 1791859200000000000ULL;
  assert(rb_ptp::ToUtc(1, utc+config.offset_ns, config) == utc);
  assert(rb_ptp::ToUtc(0, utc, config) == 0);
  assert(rb_ptp::ToUtc(2, utc, config) == 0);
  assert(rb_ptp::ToUtc(1, 1000, config) == 0);
  bool rejected = false;
  try { rb_ptp::ReadConfig(argv[2]); } catch (const std::runtime_error&) { rejected=true; }
  assert(rejected);
}''')
    good, bad = tmp_path / 'offset', tmp_path / 'bad'
    good.write_text('37\n'); bad.write_text('37 garbage\n')
    executable = tmp_path / 'clock'
    subprocess.run([compiler, '-std=c++14', '-I', str(ROOT/'tools/ptp'), str(source),
                    '-o', str(executable)], check=True, capture_output=True)
    subprocess.run([str(executable), str(good), str(bad)], check=True)


def test_patch_is_repeatable_and_preserves_unrelated_changes():
    patch = load('ptp_patch', 'tools/ptp/patch_livox.py')
    source = '\n'.join((patch.INCLUDE, '// custom mounting unchanged', patch.CALLBACK,
                        patch.FUNCTION))
    result = patch.patched(source)
    assert patch.patched(result) == result
    assert '// custom mounting unchanged' in result
    with pytest.raises(ValueError):
        patch.patched(source.replace('return time.stamp;', 'return 123;'))


def packet(mode=1, data_type=1, port=56301, source='192.168.1.138'):
    payload = bytearray(60)
    payload[1:3] = struct.pack('<H', len(payload))
    payload[10], payload[11] = data_type, mode
    payload[28:36] = struct.pack('<Q', 1791859237000000000)
    udp = struct.pack('!HHHH', 56300, port, len(payload)+8, 0) + payload
    ip = bytearray(20)
    ip[0], ip[9] = 0x45, 17
    ip[2:4] = struct.pack('!H', len(ip)+len(udp))
    ip[12:16] = socket.inet_aton(source)
    return b'\x00'*12 + b'\x08\x00' + ip + udp


def test_passive_check_reads_device_mode_not_arrival_fallback():
    check = load('ptp_check', 'tools/check_ptp.py')
    args = ('192.168.1.138', (56301, 56401))
    assert check.decode(packet(), *args) == ('lidar', 1, 1791859237.)
    assert check.decode(packet(mode=0), *args)[1] == 0
    assert check.decode(packet(data_type=0, port=56401), *args)[0] == 'imu'
    assert check.decode(packet(source='192.168.1.139'), *args) is None
    assert check.decode(packet(port=319), *args) is None
    assert check.decode(packet(data_type=1, port=56401), *args) is None
    assert check.decode(packet()[:-1], *args) is None
    assert check.decode(b'', *args) is None


@pytest.mark.parametrize('can_start', [True, False])
def test_ptp_boot_offline_without_ntp_wait(tmp_path, can_start):
    # Execute just the real boot preamble, before any process cleanup/hardware work.
    boot = (ROOT/'tools/boot.sh').read_text().split('WS=/home/user/robocup_basketball_ws')[0]
    config = tmp_path/'etc'; config.mkdir()
    (config/'livox_ptp_utc_offset').write_text('37\n')
    (config/'ptp_interface').write_text('enp114s0\n')
    boot = boot.replace('/etc/robocup', str(config))
    bin_dir = tmp_path/'bin'; bin_dir.mkdir()
    commands = {
        'timedatectl': 'echo no',
        'systemctl': 'exit 1',
        'sudo': 'exit '+('0' if can_start else '1'),
        'sleep': 'echo unexpected-NTP-wait; exit 99',
    }
    for name, body in commands.items():
        target = bin_dir/name
        target.write_text('#!/bin/sh\n'+body+'\n'); target.chmod(0o755)
    result = subprocess.run(['bash', '-c', boot], capture_output=True, text=True,
                            env={**os.environ, 'PATH': str(bin_dir)+':'+os.environ['PATH']})
    assert result.returncode == (0 if can_start else 1)
    assert 'unexpected-NTP-wait' not in result.stdout
    if can_start:
        assert '本机时钟' in result.stdout
    else:
        assert 'PTP 服务启动失败' in result.stderr
