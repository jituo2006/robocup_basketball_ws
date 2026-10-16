#!/usr/bin/env python3
"""Configure bounded CAN queues and kernel bus-off recovery before node startup."""
import argparse
import json
import subprocess


def configure(interface='can0', bitrate=1000000, queue=16, restart_ms=100, run=subprocess.run):
    if bitrate <= 0 or queue < 4 or restart_ms <= 0:
        raise ValueError('bitrate/restart_ms must be positive; queue must be at least 4')

    def read(*args):
        result = run(list(args), capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or 'command failed: '+str(args))
        return json.loads(result.stdout)

    def change(*args):
        result = run(list(args), capture_output=True, text=True)
        if result.returncode:
            result = run(['sudo', '-n', *args], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or 'CAN configuration needs ip/tc permission')

    links = read('ip', '-details', '-json', 'link', 'show', 'dev', interface)
    if len(links) != 1:
        raise RuntimeError('CAN interface not found: '+interface)
    link = links[0]
    info = link.get('linkinfo', {})
    if info.get('info_kind') != 'can':
        raise RuntimeError(interface+' is not a physical CAN interface')
    can = info.get('info_data', {})
    reconfigure = ('UP' not in link.get('flags', []) or
                   can.get('bittiming', {}).get('bitrate') != bitrate or
                   can.get('restart_ms', 0) != restart_ms)
    if reconfigure:
        change('ip', 'link', 'set', 'dev', interface, 'down')
        change('ip', 'link', 'set', 'dev', interface, 'type', 'can',
               'bitrate', str(bitrate), 'restart-ms', str(restart_ms))
    if reconfigure or link.get('txqlen') != queue:
        change('ip', 'link', 'set', 'dev', interface, 'txqueuelen', str(queue))
    if reconfigure:
        change('ip', 'link', 'set', 'dev', interface, 'up')
    elif can.get('state') == 'BUS-OFF':
        change('ip', 'link', 'set', 'dev', interface, 'type', 'can', 'restart')
    qdiscs = read('tc', '-json', 'qdisc', 'show', 'dev', interface)
    if not any(q.get('root') and q.get('kind') == 'pfifo_head_drop'
               and q.get('options', {}).get('limit') == queue for q in qdiscs):
        change('tc', 'qdisc', 'replace', 'dev', interface, 'root',
               'pfifo_head_drop', 'limit', str(queue))
    print(f'{interface}: bitrate={bitrate}, restart-ms={restart_ms}, '
          f'txqueuelen={queue}, pfifo_head_drop limit={queue}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='can0')
    args = parser.parse_args()
    try:
        configure(args.interface)
    except (RuntimeError, ValueError, OSError) as exc:
        parser.exit(1, 'CAN配置失败：'+str(exc)+'\n')


if __name__ == '__main__':
    main()
