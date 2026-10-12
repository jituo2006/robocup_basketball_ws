#!/usr/bin/env bash
# Install as the normal ROS workspace user; sudo is used only for system files.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
iface=enp114s0
driver_root=/home/user/lidar_360/ws_livox/src/livox_ros_driver2
utc_offset=37
while (($#)); do
  case "$1" in
    --interface) iface=${2:?}; shift 2 ;;
    --driver-root) driver_root=${2:?}; shift 2 ;;
    --utc-offset) utc_offset=${2:?}; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
[[ $iface =~ ^[a-zA-Z0-9_.:-]+$ && -d /sys/class/net/$iface ]]
[[ $utc_offset =~ ^[0-9]+$ && $utc_offset -le 100 ]]
sudo -n true
if ! command -v ptp4l >/dev/null || ! command -v ethtool >/dev/null; then
  sudo -n env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends linuxptp ethtool
fi
caps=$(ethtool -T "$iface")
for capability in hardware-transmit hardware-receive hardware-raw-clock; do
  if ! grep -q "$capability" <<< "$caps"; then
    echo "$iface does not support $capability; hardware PTP installation stopped" >&2
    exit 1
  fi
done
if pgrep -f '[l]ivox_ros_driver2_node' >/dev/null; then
  echo 'Stop the Livox driver before installing/rebuilding its clock conversion.' >&2
  exit 1
fi
# Do not run a second master or a second servo on this PHC.
for pid in $(pgrep -x 'ptp4l|phc2sys' || true); do
  if ! grep -q 'robocup-.*@' "/proc/$pid/cgroup"; then
    echo "Existing unmanaged PTP process $pid; inspect it before installing" >&2
    exit 1
  fi
done
python3 "$root/tools/ptp/patch_livox.py" --driver-root "$driver_root"
# ROS setup scripts need not have every optional variable defined.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$(dirname "$(dirname "$driver_root")")"
colcon build --packages-select livox_ros_driver2 --executor sequential --cmake-args -DROS_EDITION=ROS2
python3 "$root/tools/ptp/patch_livox.py" --driver-root "$driver_root" --check

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
sed "s/@UTC_OFFSET@/$utc_offset/g" "$root/tools/ptp/ptp4l.conf.in" > "$stage/ptp4l.conf"
echo "$utc_offset" > "$stage/livox_ptp_utc_offset"
echo "$iface" > "$stage/ptp_interface"
sudo -n install -d /etc/robocup /usr/local/libexec/robocup
# Retain the first pre-installation configuration for a possible rollback.
for name in ptp4l.conf livox_ptp_utc_offset ptp_interface; do
  if [[ -f /etc/robocup/$name && ! -f /etc/robocup/$name.before_robocup_ptp ]]; then
    sudo -n cp -p "/etc/robocup/$name" "/etc/robocup/$name.before_robocup_ptp"
  fi
done
sudo -n install -m 644 "$stage/ptp4l.conf" /etc/robocup/ptp4l.conf
sudo -n install -m 644 "$stage/ptp_interface" /etc/robocup/ptp_interface
sudo -n install -m 755 "$root/tools/ptp/run_ptp4l.sh" /usr/local/libexec/robocup/run_ptp4l.sh
sudo -n install -m 644 "$root/tools/ptp/robocup-ptp4l@.service" "$root/tools/ptp/robocup-phc2sys@.service" /etc/systemd/system/
sudo -n systemctl daemon-reload
sudo -n systemctl enable "robocup-ptp4l@$iface.service" "robocup-phc2sys@$iface.service"
sudo -n systemctl restart "robocup-ptp4l@$iface.service" "robocup-phc2sys@$iface.service"
# Activation is last: the rebuilt driver and both system services are present.
sudo -n install -m 644 "$stage/livox_ptp_utc_offset" /etc/robocup/livox_ptp_utc_offset
echo "PTP installed on $iface; offset=$utc_offset seconds. ROS/vehicle processes were not started."
echo "Once LiDAR and driver are running: sudo python3 $root/tools/check_ptp.py --interface $iface"
