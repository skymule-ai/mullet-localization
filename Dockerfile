# px4_ev_bridge (and ros2 bag for the record service) on mullet-ros2-base (ROS 2 Jazzy,
# px4_msgs release/1.17). Drivers and estimators are built by mullet-drivers, mullet-vio
# and mullet-lio; this repo only composes their images. Tests run at build time, so a red
# test means no image.
ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-ros2-base:v0.5.0
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /opt/mullet_ws

COPY mullet_localization src/mullet_localization
# px4_msgs is in the base workspace: list its sources so rosdep does not look for it in apt.
RUN apt-get update \
 && rosdep install --from-paths src /opt/ros2_base_ws/src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/* \
 && source /usr/local/share/mullet-ros/setup.bash \
 && colcon build --packages-select mullet_localization \
 && colcon test --packages-select mullet_localization \
 && colcon test-result --verbose \
 && rm -rf build log

CMD ["bash", "-c", "source /opt/mullet_ws/install/setup.bash && exec ros2 run mullet_localization px4_ev_bridge"]
