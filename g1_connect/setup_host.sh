#!/usr/bin/env bash
# Tune kernel IP-fragment reassembly for the lossy VM<->G1 link.
#   ./setup_host.sh            apply now (lost on reboot)
#   ./setup_host.sh --persist  also write /etc/sysctl.d/90-g1.conf
set -e
conf='net.ipv4.ipfrag_high_thresh = 67108864
net.ipv4.ipfrag_low_thresh = 50331648
net.ipv4.ipfrag_time = 3
net.ipv4.ipfrag_max_dist = 0'
echo "$conf" | sudo sysctl -p -
if [ "$1" = "--persist" ]; then
  echo "$conf" | sudo tee /etc/sysctl.d/90-g1.conf >/dev/null
  echo "persisted to /etc/sysctl.d/90-g1.conf"
fi
