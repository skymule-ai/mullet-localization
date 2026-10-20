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

## MID-360 and FAST-LIO2

### Setup (once per drone)

- **Link.** The Orin's USB Ethernet adapter (`LIVOX_IFACE`) is 192.168.1.50/24: NetworkManager
  profile `livox`, no default route. It needs sudo once:
  `sudo nmcli con add type ethernet ifname <iface> con-name livox ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes ipv6.method disabled`.
  The LiDAR is 192.168.1.104 (`config/livox/MID360s_config.json`; the block name must match the model, see
  mullet-drivers `config/livox/README.md`).
- **Extrinsic.** `config/ev_bridge_fast_lio.yaml` is the pose of base_link in the MID-360's IMU frame.
  It is **identity until measured**. Measure it before any fused flight, and before trusting a shadow
  comparison: the MID-360's position relative to the CoM (tape, ~1 cm) and which way its x axis (cable
  connector at the back) faces. Its tilt doesn't matter: FAST-LIO's world is gravity-aligned at start-up.

### Run and check

```bash
# .env: COMPOSE_PROFILES=fast-lio (or openvins,fast-lio for a comparison flight)
#       ODOMETRY_TOPIC=/uav0/fast_lio/odometry  EV_BRIDGE_CONFIG=ev_bridge_fast_lio.yaml  (bridge on LIO)
docker compose pull && docker compose up -d     # drone standing still for ~5 s
scripts/test-on-orin.sh                         # rates + the MID-360's own sync report
```

Expect `/livox/lidar` 10 Hz, `/livox/imu` 200 Hz, `/uav0/fast_lio/odometry` 10 Hz, and
`time_sync_type 1: PTP/gPTP` with an offset of a few µs. Measured on mullet1 standing still for 60 s
(2026-10-20): 3.4 cm drift, world "up" within 0.2° of gravity, FAST-LIO at ~33% of one core and 180 MB, and
the driver at 15%.

- **Start still.** FAST-LIO takes gravity and the gyro bias from its first IMU samples. Started while moving,
  it diverges within seconds: `docker compose restart fast_lio` with the drone still. The first few scans
  wobble by a few cm/s while it settles.
- **Never restart `fast_lio` in flight.** Its world restarts at the new pose. The bridge does not signal
  resets yet (`reset_counter` stays 0), so PX4 would see a jump.
- **CustomMsg.** `/livox/lidar` is `livox_ros_driver2/msg/CustomMsg`: FAST-LIO needs its per-point times.
  Foxglove can't draw it; view `/uav0/fast_lio/cloud_registered_body`. This image carries the message type
  (copied from the driver image), so `record` and `test-on-orin.sh` can read it.
- **Time.** `livox-ptp` is the gPTP master on `LIVOX_IFACE` (the MID-360 doesn't sync to plain PTP over
  UDP). Without it the driver stamps each packet on arrival: ROS time, but a few ms late and jittered.
- **Warnings.** A few `No point` / `No IMU sample in this scan, skip it` at start-up are harmless. If they
  are steady, FAST-LIO is falling behind: check `docker stats`.
- **Where it fails.** Geometry that leaves one direction unconstrained: corridors, open fields with only
  the ground, foliage seen from far, anything inside 0.5 m (`blind`). mullet-lio `docs/fast-lio.md` has the
  details.

### Record a comparison flight

`COMPOSE_PROFILES=openvins,fast-lio` runs both estimators. `docker compose run -d --rm --name loc-record
record` records the raw sensors and both odometries (`/uav0/openvins/*`, `/uav0/fast_lio/odometry|path`),
the bridge output and PX4's estimate. Both can then be scored on the same flight, and FAST-LIO can be
replayed from the raw `/livox/*`.

### Fuse FAST-LIO in PX4

The same three steps as for OpenVINS. Don't skip one:

1. **Shadow** (`outdoor` profile): the bridge forwards FAST-LIO (`ODOMETRY_TOPIC`, `EV_BRIDGE_CONFIG`
   as above) to `/localization/vehicle_odometry` only. From the bag, check:
   - the track against PX4's GPS estimate (`evo`), and no jumps;
   - the latency, `timestamp - timestamp_sample` of `/localization/vehicle_odometry`: it must stay well
     under `EKF2_DELAY_MAX` (200 ms on mullet1), the oldest measurement the EKF can still use;
   - that rotating the drone in place moves the bridge's position only by noise, which checks the
     extrinsic.
2. **Fused with GPS on** (`outdoor-vio`, `EKF2_EV_CTRL 5`: position + velocity).
3. **Without GPS** (`indoor`: + heading, compass off). Bench/walk only, as for OpenVINS.

Merging `feat/meta` waits on fused flights of both estimators, outdoors (woods) and indoors
([#4](https://github.com/skymule-ai/mullet-localization/issues/4)).

## PX4 external vision

The bridge sends base_link's pose in world FRD (`POSE_FRAME_FRD`, arbitrary heading), its
velocity in the same frame and covariance-derived variances. Its extrinsic is per estimator
(`EV_BRIDGE_CONFIG`): `config/ev_bridge.yaml` for OpenVINS (D435i IMU), `config/ev_bridge_fast_lio.yaml`
for FAST-LIO (MID-360 IMU, identity until measured). It also applies the lever arm.
The mullet-px4 profiles set the same PX4 parameters whichever estimator runs.

The PX4 parameters for vision and the bridge mode (`PUBLISH_TO_PX4`) are set together by a profile in
[mullet-px4](https://github.com/skymule-ai/mullet-px4) (`scripts/set-profile.py outdoor|outdoor-vio|indoor`).
Change them there, not by hand. For reference:

| Param | Value |
| --- | --- |
| `EKF2_EV_CTRL` | `0` in shadow mode (`outdoor`); `5` position + velocity (`outdoor-vio`); `13` + yaw (`indoor`) |
| `EKF2_EV_DELAY` | `0` while the stamps are right: the bridge sends the estimator's measurement time as `timestamp_sample` (FAST-LIO: the scan end, on the gPTP clock), so the EKF needs no extra delay. Revisit only if the shadow bag shows a lag. |
| `EKF2_EV_NOISE_MD` | `0`: reported variances, parameters as the lower bound |
| `EKF2_EVP_NOISE`, `EKF2_EVV_NOISE`, `EKF2_EVA_NOISE` | `0.1` (m, m/s, rad), the floors. FAST-LIO reports σ ≈ 3 mm and OpenVINS is also over-confident, so these floors decide the weight PX4 gives vision. Tune them in mullet-px4, from the shadow bags. |
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
