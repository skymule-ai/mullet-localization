#!/usr/bin/env bash
# Pulls the tags in .env, (re)starts the COMPOSE_PROFILES stack and reports each topic's rate.
# Run on the Orin from the repo root. Restarts the localization containers: never in flight.
set -euo pipefail

docker compose pull

# Fast DDS sizes its UDP sockets from this default; at JetPack's 208 KB half of every 400 KB LiDAR
# scan is dropped and FAST-LIO's output lags ~2 s (README, MID-360 setup).
[ "$(sysctl -n net.core.rmem_default)" -ge 8388608 ] \
  || echo "WARNING: net.core.rmem_default < 8 MiB: /livox/lidar loses datagrams (README, Socket buffers)"
docker compose up -d
sleep 10  # driver and estimator start-up

# The driver ignores a config value of the wrong type with only a WARN (emitter_enabled: false
# left the IR projector on for a whole day of recordings).
docker compose logs --since 60s realsense 2>&1 | grep "Could not set param" \
  && echo "WARNING: the camera driver rejected the setting(s) above: check config/realsense/d435i.yaml" || true

# Expected: infra 30 Hz, imu 200 Hz, lidar 10 Hz, livox imu 200 Hz, OpenVINS odometry ~ imu
# rate, FAST-LIO odometry 10 Hz. A topic of a profile that is not running prints NO DATA.
for topic in /uav0/d435i/infra1/image_rect_raw /uav0/d435i/infra2/image_rect_raw \
             /uav0/d435i/imu /livox/lidar /livox/imu \
             /uav0/openvins/odomimu /uav0/fast_lio/odometry /localization/vehicle_odometry; do
  rate=$(docker compose run --rm --no-deps -T ev_bridge bash -c \
    "source /opt/mullet_ws/install/setup.bash && timeout 6 ros2 topic hz $topic" 2>/dev/null \
    | awk '/average rate/ {r=$3} END {print r}' || true)
  echo "$topic ${rate:-NO DATA}"
done

# The MID-360's own report of its gPTP sync (time_sync_type 1, offset in us); exits 1 when unsynced.
if docker compose ps --services --status running | grep -qx livox; then
  docker compose exec -T livox livox-status "$(sed -n 's/^LIVOX_IFACE=//p' .env)" || echo "WARNING: the MID-360 is not synced to livox-ptp"
fi
