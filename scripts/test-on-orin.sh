#!/usr/bin/env bash
# Pulls <tag>, (re)starts the sensors service with it and reports each sensor topic's rate.
# Run on the Orin from the repo root, with .env in place. Restarts mullet-sensors: never in flight.
set -euo pipefail

export IMAGE_TAG=${1:?usage: scripts/test-on-orin.sh <image-tag>}

docker compose pull sensors
docker compose up -d sensors
sleep 5  # driver start-up

# Expected: infra1/infra2 30 Hz, imu 200 Hz (D435i); points 10 Hz, imu 200 Hz (Mid-360).
for topic in /sensors/d435i/infra1/image_rect_raw /sensors/d435i/infra2/image_rect_raw \
             /sensors/d435i/imu /sensors/mid360/points /sensors/mid360/imu; do
  rate=$(docker compose exec -T sensors bash -c \
    "source /opt/mullet_ws/install/setup.bash && timeout 6 ros2 topic hz $topic" 2>/dev/null \
    | awk '/average rate/ {r=$3} END {print r}' || true)
  echo "$topic ${rate:-NO DATA}"
done
