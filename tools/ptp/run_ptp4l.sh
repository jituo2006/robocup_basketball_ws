#!/usr/bin/env bash
set -euo pipefail
iface=${1:?network interface required}
# NetworkManager assigns the static address after the powered LiDAR has link.
# Wait without bringing the interface up or changing its network configuration.
while [[ ! -r /sys/class/net/$iface/carrier ]] ||
      [[ $(cat "/sys/class/net/$iface/carrier" 2>/dev/null || true) != 1 ]] ||
      ! ip -4 -o addr show dev "$iface" | grep -q ' inet '; do
  sleep 1
done
exec /usr/sbin/ptp4l -i "$iface" -f /etc/robocup/ptp4l.conf -m -q
