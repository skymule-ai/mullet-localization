#!/usr/bin/env bash
# Pulls the tags in .env, (re)starts the COMPOSE_PROFILES stack and reports each topic's rate.
# Run on the Orin from the repo root. Restarts the localization containers: never in flight.
set -euo pipefail

docker compose pull
docker compose up -d
sleep 10  # driver and estimator start-up

# The driver ignores a config value of the wrong type with only a WARN (emitter_enabled: false
# left the IR projector on for a whole day of recordings).
docker compose logs --since 60s realsense 2>&1 | grep "Could not set param" \
  && echo "WARNING: the camera driver rejected the setting(s) above: check config/realsense/d435i.yaml" || true

# Expected: infra 30 Hz, imu 200 Hz, lidar 10 Hz, livox imu 200 Hz, odometry ~ imu rate.
for topic in /uav0/d435i/infra1/image_rect_raw /uav0/d435i/infra2/image_rect_raw \
             /uav0/d435i/imu /livox/lidar /livox/imu \
             "$(sed -n 's/^ODOMETRY_TOPIC=//p' .env)" /localization/vehicle_odometry; do
  rate=$(docker compose run --rm --no-deps -T ev_bridge bash -c \
    "source /opt/mullet_ws/install/setup.bash && timeout 6 ros2 topic hz $topic" 2>/dev/null \
    | awk '/average rate/ {r=$3} END {print r}' || true)
  echo "$topic ${rate:-NO DATA}"
done
