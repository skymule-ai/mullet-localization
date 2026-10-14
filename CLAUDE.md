# CLAUDE.md

Guidance for AI coding sessions working in this repository. User-facing docs: `README.md`.

## What this is

Onboard localization for Mullet drones (Orin Nano, ROS 2 Jazzy, PX4 1.17): sensor drivers
(D435i, Mid-360), one estimator backend at a time, and `px4_ev_bridge`, which turns the backend's
`localization/odometry` into PX4 external vision. Images are built in CI and pushed to GHCR. The
drone pulls immutable tags and never builds.

Sibling repos: `mullet_ros_base` (the base image), `mullet-core` (the supervisor; its
`docker/compose.yaml` is the pattern copied here), `mullet-paddock` (ground station), `mullet-wiki`.

## Rules

- **Every backend remaps its output to `localization/odometry`** (nav_msgs/Odometry, REP 103:
  z-up world, FLU body, twist in the child frame). Nothing downstream may know which backend runs.
  A backend that breaks the convention (VINS-Fusion publishes its twist in the world frame) gets a
  relay in its launch file, not a branch in the bridge.
- **`px4_ev_bridge` is the only `/fmu/in` writer here**, and only for
  `/fmu/in/vehicle_visual_odometry`. It defaults to shadow mode (`publish_to_px4: false`,
  `localization/vehicle_odometry`). Do not change that default.
- The frame math lives in `mullet_localization/frames.py` with no ROS imports, covered by
  `test/test_frames.py`. Any change there needs a test that fails without it.
- The sensors run on their own (`sensors` service) so that switching or crashing a backend never
  restarts a camera. The drivers move to a `mullet-sensors` repo once perception or obstacle
  avoidance needs them (README, "Later").

## Layout

- `Dockerfile`: stage `common` (librealsense RSUSB, Livox-SDK2, `repos/drivers.repos`,
  foxglove_bridge, this package; tests run at build). A backend = a new stage `FROM common`, plus
  `repos/<backend>.repos`, `config/<backend>/`, a launch file and a CI matrix entry.
- `compose.yaml`: `sensors` (default), `viz`, `record`, `localization` profiles. `config/` is
  mounted at `/config`.
- Version pairs that must move together: realsense-ros ↔ `LIBREALSENSE_VERSION`
  (4.59 ↔ 2.59), livox_ros_driver2 ↔ `LIVOX_SDK2_VERSION`.

## Build & test

```bash
cd mullet_localization && python3 -m pytest test/      # frame math, needs numpy + pytest only
docker compose build sensors                           # arm64 only: the base image is arm64/CUDA
```

On an x86 laptop, build the Dockerfile against an amd64 stand-in base (`osrf/ros:jazzy-*` plus
px4_msgs release/1.17 in `/opt/ros2_base_ws` and `/usr/local/share/mullet-ros/setup.bash`) with
`--build-arg BASE_IMAGE=...` to catch driver build breaks before CI.

On the drone: `scripts/test-on-orin.sh <tag>` restarts `mullet-sensors` and prints each sensor topic's
rate. Ask before running anything on the drone: experiments run there in parallel.
