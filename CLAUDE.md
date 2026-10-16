# CLAUDE.md

Guidance for AI coding sessions working in this repository. User-facing docs: `README.md`.

## What this is

The localization deployment for a Mullet drone (Orin Nano, ROS 2 Jazzy, PX4 1.17): a Compose
project that pins images from `mullet-drivers` (sensors), `mullet-vio` and `mullet-lio`
(estimators), holds the drone's sensor configs and calibration in `config/`, and builds one image
of its own with `px4_ev_bridge` and `ros2 bag`. All images come from CI on GHCR; the drone pulls
and never builds.

Driver or estimator code and image builds belong in their repos, not here. A new estimator is an
image there plus, here, a compose service, a profile on `ev_bridge`, and its output topic as
`ODOMETRY_TOPIC`.

## Rules

- **The bridge reads standard odometry** (nav_msgs/Odometry, REP 103: z-up world, FLU body, twist
  in the child frame) on `localization/odometry`, remapped from the estimator's topic. An estimator
  that breaks the convention (VINS-Fusion publishes its twist in the world frame) gets a relay,
  not a branch in the bridge.
- **`px4_ev_bridge` is the only `/fmu/in` writer here**, and only for
  `/fmu/in/vehicle_visual_odometry`. It defaults to shadow mode (`publish_to_px4: false`,
  `/localization/vehicle_odometry`). Do not change that default.
- The frame math lives in `mullet_localization/frames.py` with no ROS imports, covered by
  `test/test_frames.py`. Any change there needs a test that fails without it.
- Camera parameters use the type the driver declares (`depth_module.emitter_enabled: 0`, not `false`):
  a wrong type is dropped with only a WARN. After changing `config/realsense/d435i.yaml`, check the
  driver log (`scripts/test-on-orin.sh` prints rejected settings).
- PX4 parameters and the bridge mode (`PUBLISH_TO_PX4`) belong to a profile in
  [mullet-px4](https://github.com/skymule-ai/mullet-px4): change them there, not in `.env` or QGC.
- `config/realsense/d435i.yaml` and `config/openvins/kalibr_imucam_chain.yaml` must
  agree on the infra resolution (848x480): the calibration is only valid at that size.

## Build & test

```bash
cd mullet_localization && python3 -m pytest test/   # frame math, needs numpy + pytest only
```

The image is arm64 (CUDA base). On an x86 laptop, build against an amd64 stand-in base
(`osrf/ros:jazzy-*` plus px4_msgs release/1.17 in `/opt/ros2_base_ws` and
`/usr/local/share/mullet-ros/setup.bash`) with `--build-arg BASE_IMAGE=...`.

On the drone, `scripts/test-on-orin.sh` (re)starts the stack in `.env` and prints each topic's
rate. Ask before running anything on the drone: experiments run there in parallel.
