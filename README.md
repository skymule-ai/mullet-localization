# mullet-localization

Onboard localization for Mullet drones: the sensor drivers, one estimator backend at a time, and
the bridge that feeds the estimate to PX4's EKF2 as external vision. Docker images built on
[mullet_ros_base](https://github.com/skymule-ai/mullet_ros_base) (ROS 2 Jazzy, PX4 1.17, Orin Nano).

```text
sensors ──► /sensors/d435i/...  /sensors/mid360/...        always on, whatever backend runs
   │
backend ──► localization/odometry (nav_msgs/Odometry)      exactly one; every backend remaps here
   │
px4_ev_bridge ──► localization/vehicle_odometry            shadow mode (default)
              └─► /fmu/in/vehicle_visual_odometry          PUBLISH_TO_PX4=true
```

The bridge is the only `/fmu/in` writer outside mullet_core, and it writes only that topic. Mission
frameworks still never publish `/fmu/in/*`.

## Backends

Which one flies is decided on recorded flights: every backend replays the same bags and is scored
against PX4's GPS estimate (`evo` ATE/RPE), plus CPU/GPU load and resets.

| Backend | Sensors | Status |
| --- | --- | --- |
| OpenVINS | D435i infra stereo + IMU | next (phase 3) |
| Isaac ROS Visual SLAM | D435i infra stereo + IMU, GPU | after a compatibility spike: Jazzy on Orin JetPack 6.2 |
| VINS-Fusion | D435i infra stereo + IMU | ROS 2 fork to pick and pin |
| FAST-LIO2 | Mid-360 + its IMU | placeholder |
| FAST-LIVO2 | Mid-360 + D435i | placeholder, upstream is ROS 1 |

`repos/<backend>.repos` holds each one's sources; each becomes a Dockerfile stage `FROM common`.

## Sensors

| Sensor | Link | Topics |
| --- | --- | --- |
| RealSense D435i | USB 3 | `/sensors/d435i/{infra1,infra2}/image_rect_raw` 30 Hz, `/sensors/d435i/imu` 200 Hz |
| Livox Mid-360 | USB-Ethernet adapter | `/sensors/mid360/points` 10 Hz, `/sensors/mid360/imu` 200 Hz |

Settings live in `config/sensors/`. The D435i runs infra stereo with the IR emitter off (its dots
break feature tracking) and one IMU topic interpolated to the gyro rate. librealsense uses the
RSUSB backend, so the Jetson kernel needs no patches.

The Mid-360 is on its own subnet, separate from the onboard port (bench SSH). One-time host setup
on the Orin, with the adapter's interface name from `ip -br link`:

```bash
sudo nmcli con add type ethernet ifname <usb-eth-if> con-name mid360 ipv4.method manual ipv4.addresses 192.168.1.5/24
```

The lidar's own IP is `192.168.1.1XX`, where XX are the last two digits of its serial number. Put
it in `config/sensors/MID360_config.json` (`lidar_configs[0].ip`). A wrong lidar IP means no data
and no error. `bind failed` / `Init lds lidar fail!` in the log means no interface holds the host
IP (`192.168.1.5`).

## On the drone

```bash
git clone git@github.com:skymule-ai/mullet-localization.git && cd mullet-localization
cp .env.example .env              # IMAGE_TAG (immutable on aircraft), ROS_DOMAIN_ID, sensors present
docker compose pull && docker compose up -d                   # sensors, on boot
scripts/test-on-orin.sh sha-<commit>                          # sensor topic rates
docker compose --profile viz up -d                            # foxglove_bridge on :8765
docker compose run --rm record                                # bag, Ctrl-C to stop
docker compose --profile localization up -d                   # px4_ev_bridge (+ backend)
```

`config/` is mounted: edit it, then `docker compose restart <service>`. Never restart during a
flight.

**Visualization.** Open [Lichtblick](https://github.com/lichtblick-suite/lichtblick) (or Foxglove)
on the laptop and connect to `ws://<drone-ip>:8765`. The bridge sends only the topics a panel
subscribes to. A raw Mid-360 cloud is about 5 MB/s over WiFi.

**Recording.** `~/flights/<UTC>_sensors/` (mcap): sensors, `localization/*`, PX4 odometry, local
position, GPS, IMU, TF. About 20 MB/s with both sensors, so record to NVMe.

## PX4 external vision

The bridge sends base_link's pose in world FRD (`POSE_FRAME_FRD`, arbitrary heading), its velocity
in the same frame and covariance-derived variances. The estimator-to-base_link extrinsic is in
`config/ev_bridge.yaml`: identity until calibration (Kalibr + CAD). It also applies the lever arm.

| Param | Value |
| --- | --- |
| `EKF2_EV_CTRL` | `0` in shadow mode; then horizontal + vertical position + velocity (+ yaw) |
| `EKF2_EV_DELAY` | measured: the estimator's latency against the IMU |
| `EKF2_EV_NOISE_MD` | `0`: reported variances, parameters as the lower bound |
| `EKF2_EV_POS_X/Y/Z` | `0`: the bridge already applies the lever arm |
| `EKF2_HGT_REF` | stays on baro/GPS until EV is trusted |
| `UXRCE_DDS_SYNCT` | `1` (default): PX4 converts the bridge's ROS timestamps |

Order: shadow flight (recorded, not fused) → fused with GPS on → GPS denied. Without GPS, the
paddock map has no global position unless the EKF origin is set.

## Build

CI (`.github/workflows/build-and-push.yml`) builds `linux/arm64` and pushes
`ghcr.io/skymule-ai/mullet-localization:{sha-<commit>,<branch>,<tag>}`, the same flow as
mullet_ros_base. The image build runs `colcon test`. The frame conversion tests alone:

```bash
cd mullet_localization && python3 -m pytest test/
```

## Later

The drivers live here until a second consumer appears. Once obstacle avoidance or perception needs
the D435i or the Mid-360 voxel map, move the `sensors` service, `repos/drivers.repos` and
`config/sensors/` to a `mullet-sensors` repo and build this image `FROM` it.
