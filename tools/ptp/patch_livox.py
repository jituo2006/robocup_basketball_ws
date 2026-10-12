#!/usr/bin/env python3
"""Apply a narrowly scoped, repeatable PTP patch to the external Livox driver."""
import argparse
from pathlib import Path
import shutil


INCLUDE = '#include "pub_handler.h"'
CALLBACK = '  if (data->time_type != kTimestampTypeNoSync) {'
FUNCTION = '''  if (timestamp_type == kTimestampTypeGptpOrPtp ||
      timestamp_type == kTimestampTypeGps) {
    return time.stamp;
  }'''
PTP_CALLBACK = '''  // RB_PTP: wait for device PTP; do not queue uptime/arrival-time samples.
  if (rb_ptp::ClockConfig().enabled &&
      data->time_type != kTimestampTypeGptpOrPtp) {
    return;
  }

'''
PTP_FUNCTION = '''  // RB_PTP: standard PTP carries TAI, while ROS expects UTC.
  if (rb_ptp::ClockConfig().enabled) {
    return rb_ptp::ToUtc(timestamp_type, time.stamp, rb_ptp::ClockConfig());
  }

'''


def patched(source):
    include = INCLUDE + '\n#include "livox_ptp_clock.hpp"'
    callback = PTP_CALLBACK + CALLBACK
    function = PTP_FUNCTION + FUNCTION
    if 'RB_PTP:' in source:
        if all(part in source for part in (include, callback, function)):
            return source
        raise ValueError('Existing PTP patch differs; inspect driver before updating')
    for part in (INCLUDE, CALLBACK, FUNCTION):
        if source.count(part) != 1:
            raise ValueError('Livox source differs from expected patch location')
    return source.replace(INCLUDE, include).replace(CALLBACK, callback).replace(FUNCTION, function)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver-root', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    source = args.driver_root / 'src/comm/pub_handler.cpp'
    header = source.with_name('livox_ptp_clock.hpp')
    desired_header = Path(__file__).with_name('livox_ptp_clock.hpp').read_bytes()
    before = source.read_text()
    after = patched(before)
    if args.check:
        if before != after or not header.exists() or header.read_bytes() != desired_header:
            parser.exit(1, 'Livox PTP patch/header missing or different\n')
        print('Livox PTP patch verified')
        return
    if before != after:
        backup = source.with_suffix('.cpp.before_robocup_ptp')
        if not backup.exists():
            shutil.copy2(source, backup)
        source.write_text(after)
    header.write_bytes(desired_header)
    print('Livox PTP patch applied; rebuild livox_ros_driver2 before enabling PTP')


if __name__ == '__main__':
    main()
