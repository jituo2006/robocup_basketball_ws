#!/usr/bin/env python3
"""Patch external Livox SDK/driver shutdown while preserving existing PTP changes."""
import argparse
from pathlib import Path
import shutil


def replace_once(source, before, after):
    if after in source:
        return source
    if source.count(before) != 1:
        raise ValueError('Unexpected Livox source; inspect before patching')
    return source.replace(before, after)


def sdk_cmake(source):
    old = 'set(LIVOX_PRIVATE_INCLUDE_DIR \n'
    new = '# RB_SHUTDOWN: use the same compiled spdlog ABI as ROS.\nfind_package(spdlog REQUIRED CONFIG)\ntarget_link_libraries(${SDK_LIBRARY_STATIC} PUBLIC spdlog::spdlog)\ntarget_link_libraries(${SDK_LIBRARY_SHARED} PUBLIC spdlog::spdlog)\n\n' + old
    source = replace_once(source, old, new)
    return source.replace('        ../3rdparty/spdlog \n', '')


def sdk_logging(source):
    source = source.replace('"console"', '"livox_sdk"')
    return replace_once(source, '  spdlog::drop_all();',
                        '  // RB_SHUTDOWN: remove only our logger; keep the ROS registry alive.\n'
                        '  spdlog::drop("livox_sdk");\n  logger.reset();')


def driver_node(source):
    before = '''DriverNode::~DriverNode() {
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}'''
    after = '''DriverNode::~DriverNode() {
  // RB_SHUTDOWN: wake both queues even if no sensor packet will arrive.
  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->RequestExit();
    lddc_ptr_->lds_->pcd_semaphore_.Signal();
    lddc_ptr_->lds_->imu_semaphore_.Signal();
  }
  exit_signal_.set_value();
  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
  // SDK teardown runs while this ROS node and its library still exist.
  lddc_ptr_.reset();
}'''
    return replace_once(source, before, after)


def driver_source(source):
    return replace_once(source, '    LivoxLidarSdkUninit();',
                        '    // RB_SHUTDOWN: stop callbacks/processing before SDK destruction.\n'
                        '    pub_handler().Uninit();\n    LivoxLidarSdkUninit();\n'
                        '    is_initialized_ = false;')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root', type=Path, required=True)
    parser.add_argument('--driver-root', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    changes = []
    for path, patch in ((args.sdk_root/'sdk_core/CMakeLists.txt', sdk_cmake),
                        (args.sdk_root/'sdk_core/base/logging.cpp', sdk_logging),
                        (args.driver_root/'src/driver_node.cpp', driver_node),
                        (args.driver_root/'src/lds_lidar.cpp', driver_source)):
        before = path.read_text()
        after = patch(before)
        changes.append((path, before, after))
    # Validate every source first; a mismatched checkout is left untouched.
    if args.check:
        if any(before != after for _, before, after in changes):
            parser.exit(1, 'Livox shutdown patch missing; rebuild after applying\n')
        print('Livox shutdown source patch verified')
        return
    for path, before, after in changes:
        if before != after:
            backup = path.with_name(path.name+'.before_rb_shutdown')
            if not backup.exists():
                shutil.copy2(path, backup)
            path.write_text(after)
    print('Livox shutdown patch applied; rebuild SDK and driver before use')


if __name__ == '__main__':
    main()
