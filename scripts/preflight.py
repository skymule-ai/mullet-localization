"""Pre-recording check: every item green, or exit 1. Runs as the `preflight` compose service (see README):

    docker compose run --rm preflight && docker compose run -d --rm --name loc-record record
    docker compose run --rm preflight --no-gps       # indoor-only: GPS items are skipped, not failed

Listens for WINDOW seconds, then checks the host (clock, disk, UDP buffer), PX4 (alive, EKF aligned, GPS), the
estimators COMPOSE_PROFILES runs, and the bridge in the mode PUBLISH_TO_PX4 sets.
"""
import os
import shutil
import sys
import time
from collections import defaultdict

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import EstimatorStatusFlags, SensorGps, VehicleOdometry
from rclpy.qos import qos_profile_sensor_data

WINDOW = 6.0  # s
NO_GPS = "--no-gps" in sys.argv[1:]
PROFILES = set(os.environ.get("COMPOSE_PROFILES", "").split(","))
FUSED = os.environ.get("PUBLISH_TO_PX4", "false").lower() == "true"
EV_TOPIC = "/fmu/in/vehicle_visual_odometry" if FUSED else "/localization/vehicle_odometry"
FLIGHTS_MIN_GB = 20  # full `record` is ~1.6 GB/min
RMEM_MIN = 8 * 1024 * 1024  # below this /livox/lidar loses datagrams and FAST-LIO lags ~2 s (README)
EV_LAG_MAX_MS = 100  # EKF2_DELAY_MAX is 200 ms

rclpy.init()
node = rclpy.create_node("preflight")
msgs = defaultdict(list)  # topic -> [(receive time, msg)]
subs = [("/fmu/out/estimator_status_flags", EstimatorStatusFlags), ("/fmu/out/vehicle_gps_position", SensorGps),
        (EV_TOPIC, VehicleOdometry), ("/uav0/fast_lio/odometry", Odometry), ("/uav0/openvins/odomimu", Odometry)]
for topic, cls in subs:
    node.create_subscription(cls, topic, lambda m, t=topic: msgs[t].append((time.monotonic(), m)),
                             qos_profile_sensor_data)
end = time.monotonic() + WINDOW
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

results = []  # (status, name, detail): status PASS / FAIL / SKIP


def check(name, ok, detail):
    results.append(("PASS" if ok else "FAIL", name, detail))


def skip(name, why):
    results.append(("SKIP", name, why))


def rate(topic):
    return len(msgs[topic]) / WINDOW


# Host
synced = os.path.exists("/run/timesync/synchronized")  # systemd-timesyncd's flag, host /run/systemd/timesync
check("clock synchronised", synced,
      "NTP synced" if synced else "not synced: bag names and times would be wrong (no RTC battery); get on mullnet")
free_gb = shutil.disk_usage("/flights").free / 1e9
check("disk space", free_gb >= FLIGHTS_MIN_GB, f"{free_gb:.0f} GB free in ~/flights (~{free_gb / 1.6:.0f} min of full record)")
rmem = int(open("/proc/sys/net/core/rmem_default").read())
check("UDP receive buffer", rmem >= RMEM_MIN, f"net.core.rmem_default {rmem} (need {RMEM_MIN})")

# PX4
flags = msgs["/fmu/out/estimator_status_flags"]
check("PX4 publishing", bool(flags), f"estimator_status_flags {rate('/fmu/out/estimator_status_flags'):.1f} Hz"
      if flags else "nothing from PX4: battery, Pixhawk USB/serial or the XRCE agent")
f = flags[-1][1] if flags else None
if f:
    check("EKF aligned", f.cs_tilt_align and f.cs_yaw_align, f"tilt {f.cs_tilt_align}, yaw {f.cs_yaw_align}")
gps = msgs["/fmu/out/vehicle_gps_position"]
if NO_GPS:
    skip("GPS fix", "--no-gps")
    skip("GPS fused", "--no-gps")
else:
    g = gps[-1][1] if gps else None
    check("GPS fix", g is not None and g.fix_type >= 3 and g.satellites_used >= 6,
          f"fix_type {g.fix_type}, {g.satellites_used} satellites, eph {g.eph:.1f} m" if g else "no GPS messages")
    check("GPS fused", bool(f and f.cs_gnss_pos), f"cs_gnss_pos {f.cs_gnss_pos}" if f else "no EKF flags")

# Estimators
for prof, topic, min_hz in (("fast-lio", "/uav0/fast_lio/odometry", 8), ("openvins", "/uav0/openvins/odomimu", 50)):
    if prof in PROFILES:
        check(f"{prof} odometry", rate(topic) >= min_hz, f"{topic} {rate(topic):.1f} Hz (need {min_hz})")
    else:
        skip(f"{prof} odometry", "not in COMPOSE_PROFILES")

# Bridge
ev = [m for _, m in msgs[EV_TOPIC]]
mode = "fused" if FUSED else "shadow"
check(f"bridge ({mode})", rate(EV_TOPIC) >= 8, f"{EV_TOPIC} {rate(EV_TOPIC):.1f} Hz")
if ev:
    lag = sorted((m.timestamp - m.timestamp_sample) / 1e3 for m in ev)
    check("vision lag", lag[len(lag) // 2] <= EV_LAG_MAX_MS,
          f"median {lag[len(lag) // 2]:.0f} ms, max {lag[-1]:.0f} ms (EKF2_DELAY_MAX 200 ms)")
if FUSED:
    check("PX4 fusing vision", bool(f and f.cs_ev_pos and f.cs_ev_vel),
          f"cs_ev_pos {f.cs_ev_pos}, cs_ev_vel {f.cs_ev_vel}" if f else "no EKF flags")

node.destroy_node()
rclpy.shutdown()
width = max(len(n) for _, n, _ in results)
for status, name, detail in results:
    mark = {"PASS": "\033[32m PASS \033[0m", "FAIL": "\033[31m FAIL \033[0m", "SKIP": "\033[33m SKIP \033[0m"}[status]
    print(f"[{mark}] {name:<{width}}  {detail}")
failed = [n for s, n, _ in results if s == "FAIL"]
print("\nALL GREEN: record." if not failed else f"\nNOT READY: {', '.join(failed)}.")
sys.exit(1 if failed else 0)
