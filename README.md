# mullet-localization

Localization on a Mullet drone, as one Compose project. The drivers and estimators are
built and versioned in their own repos; this one pins which images a drone runs, holds that
drone's sensor configs and calibration, and adds the bridge that feeds the estimate to PX4's
EKF2 as external vision.

| Repo | Images | Role |
| --- | --- | --- |
| [mullet-drivers](https://github.com/skymule-ai/mullet-drivers) | `mullet-drivers-{realsense,livox,viz}` | D435i, Mid-360S, RViz / foxglove_bridge |
| [mullet-vio](https://github.com/skymule-ai/mullet-vio) | `mullet-vio-openvins` (VINS-Fusion, Isaac VSLAM later) | visual-inertial odometry |
| [mullet-lio](https://github.com/skymule-ai/mullet-lio) | `mullet-lio-fast-lio` (FAST-LIVO2 later) | lidar-inertial odometry |
| this repo | `mullet-localization` | `px4_ev_bridge`, `ros2 bag` |

All of them build `FROM` [mullet-base](https://github.com/skymule-ai/mullet-base)
`v0.5.1` (ROS 2 Jazzy, px4_msgs 1.17), like mullet-core.

```text
realsense ──► /uav0/d435i/...                  livox ──► /livox/lidar (CustomMsg), /livox/imu
     │                                           │        ◄── livox-ptp (PTP master)
openvins ──► /uav0/openvins/odomimu            fast_lio ──► /uav0/fast_lio/odometry
     └──────────────┬────────────────────────────┘
ev_bridge (ODOMETRY_TOPIC: one of them) ──► /localization/vehicle_odometry    shadow mode (default)
                                        └─► /fmu/in/vehicle_visual_odometry  PUBLISH_TO_PX4=true
```

The bridge is the only `/fmu/in` writer outside mullet_core, and it writes only that topic.
Mission frameworks still never publish `/fmu/in/*`.

## On the drone

```bash
git clone git@github.com:skymule-ai/mullet-localization.git && cd mullet-localization
cp .env.example .env     # ROS_DOMAIN_ID, COMPOSE_PROFILES, image tags (immutable on aircraft)
docker compose pull && docker compose up -d
scripts/test-on-orin.sh  # topic rates of the running stack
docker compose run --rm record                 # bag into ~/flights/<UTC>_localization
docker compose run -d --rm --name loc-record record  # same, detached: survives SSH drops
docker kill -s INT loc-record                  # stops it, closes the bag; the container removes itself
```

`COMPOSE_PROFILES` picks the stack: `openvins` (D435i + OpenVINS + bridge), `fast-lio` (MID-360 +
PTP master + FAST-LIO2 + bridge), both (`openvins,fast-lio`) for comparison flights, plus `foxglove`
for the ground-station view (Lichtblick or Foxglove at `ws://<drone-ip>:8765`). Switching the
estimator the bridge forwards means `ODOMETRY_TOPIC` and `EV_BRIDGE_CONFIG` (its extrinsic); the
recorder and PX4 do not change. Merging `feat/meta` waits on fused flights of both estimators,
outdoors and indoors ([#4](https://github.com/skymule-ai/mullet-localization/issues/4)).

`config/` is mounted, so tuning needs no rebuild: `docker compose restart <service>`. Never during
a flight. Pulling the drone's images needs a GHCR login with `read:packages`.

## This drone's configuration

| File | What |
| --- | --- |
| `config/realsense/d435i.yaml` | infra stereo 848x480@30, emitter off (`emitter_enabled: 0`, an integer), IMU 200 Hz united |
| `config/livox/MID360s_config.json` | MID-360(S) at 192.168.1.104, host 192.168.1.50 (USB-Ethernet adapter `LIVOX_IFACE`). The block name must match the model: see mullet-drivers `config/livox/README.md` |
| `config/openvins/` | OpenVINS estimator and Kalibr cam-IMU calibration (848x480) |
| `config/fast-lio/mid360.yaml` | FAST-LIO2: factory LiDAR-IMU extrinsic held fixed, no map publishing or PCD saving |
| `config/ev_bridge.yaml` | pose of base_link in OpenVINS's body frame (the D435i IMU) |
| `config/ev_bridge_fast_lio.yaml` | pose of base_link in FAST-LIO's body frame (the MID-360 IMU); identity until measured |

The defaults in mullet-drivers and mullet-vio are examples. These files are what this drone flies
with: change them here.

## D435i infrared projector

Be careful with this one: it silently ruins VIO.

- **What it does:** projects a fixed infrared dot pattern so stereo depth works on blank surfaces. Both
  infrared cameras see the dots.
- **Why it matters for VIO:** the dots move with the camera, and OpenVINS tracks them as if they were
  fixed to the scene. Indoors, near walls and on stairs this means drift and lost tracking (the
  2026-10-16 office walk drifted 4.5% with it on, 1.4% off). Outdoors, sunlight mostly washes it out.
- **Off** whenever VIO uses the infrared images. That is our default.
- **On** only for depth or point clouds on texture-poor surfaces, without VIO on the same images.
  Alternating frames (`depth_module.emitter_on_off`) needs VIO fed only the projector-off frames.
- **Setting:** `depth_module.emitter_enabled: 0` in `config/realsense/d435i.yaml` (0 off, 1 on, 2 auto).
  It must be an **integer**: `false` is rejected with only a WARN, and the projector stays on.
- **Check:**
  - `ros2 param get /uav0/d435i depth_module.emitter_enabled` says `Integer value is: 0`;
  - `scripts/test-on-orin.sh` prints any setting the driver rejected;
  - the infrared image shows no regular grid of white dots on nearby surfaces.

## MID-360

- **Link.** The Orin's USB Ethernet adapter is 192.168.1.50/24 (NetworkManager profile `livox`,
  no default route); the LiDAR is 192.168.1.104.
- **CustomMsg.** `/livox/lidar` is `livox_ros_driver2/msg/CustomMsg`: FAST-LIO needs its per-point
  times. Foxglove can't draw it; view `/uav0/fast_lio/cloud_registered_body`. This image carries the
  message type (copied from the driver image), so `record` and `test-on-orin.sh` can read it.
- **Time.** `livox-ptp` is the PTP master on `LIVOX_IFACE`. Without it the driver stamps each packet
  on arrival at the Orin: ROS time, but a few ms late and jittered. With it, at measurement.
  It is gPTP: the MID-360 does not sync to plain PTP over UDP. `scripts/test-on-orin.sh` runs
  `livox-status`, which prints the sync state the LiDAR itself reports (`time_sync_type 1`, offset in µs).
- **Start-up.** Keep the drone still while FAST-LIO starts: it initialises gravity and the IMU
  biases from the first samples. Its world frame is the IMU frame at that moment.

## PX4 external vision

The bridge sends base_link's pose in world FRD (`POSE_FRAME_FRD`, arbitrary heading), its
velocity in the same frame and covariance-derived variances. The extrinsic in
`config/ev_bridge.yaml` is identity until measured (Kalibr + CAD); it also applies the lever arm.

The PX4 parameters for vision and the bridge mode (`PUBLISH_TO_PX4`) are set together by a profile in
[mullet-px4](https://github.com/skymule-ai/mullet-px4) (`scripts/set-profile.py outdoor|outdoor-vio|indoor`).
Change them there, not by hand. For reference:

| Param | Value |
| --- | --- |
| `EKF2_EV_CTRL` | `0` in shadow mode (`outdoor`); `5` position + velocity (`outdoor-vio`); `13` + yaw (`indoor`) |
| `EKF2_EV_DELAY` | measured: the estimator's latency against the IMU |
| `EKF2_EV_NOISE_MD` | `0`: reported variances, parameters as the lower bound |
| `EKF2_EV_POS_X/Y/Z` | `0`: the bridge already applies the lever arm |
| `EKF2_HGT_REF` | stays on baro/GPS until EV is trusted |
| `UXRCE_DDS_SYNCT` | `1` (default): PX4 converts the bridge's ROS timestamps |

Order: shadow flight (recorded, not fused) → fused with GPS on → GPS denied. Which estimator wins
is decided offline: replay the same bags through each, score against PX4's GPS estimate (`evo`
ATE/RPE), plus CPU/GPU load and resets.

## Build

CI (`.github/workflows/build-and-push.yml`) builds `linux/arm64` and pushes
`ghcr.io/skymule-ai/mullet-localization:{sha-<commit>,<branch>,<tag>}`. The image build runs
`colcon test`; the frame-conversion tests alone:

```bash
cd mullet_localization && python3 -m pytest test/
```
