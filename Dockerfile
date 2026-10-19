# px4_ev_bridge (and ros2 bag for the record service) on mullet-base (ROS 2 Jazzy,
# px4_msgs release/1.17). Drivers and estimators are built by mullet-drivers, mullet-vio
# and mullet-lio; this repo only composes their images. Tests run at build time, so a red
# test means no image.
ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-base:v0.5.1
# livox_ros_driver2 from the driver image, so the recorder and `ros2 topic hz` can subscribe to the
# CustomMsg /livox/lidar with the driver's exact type. Same base release as BASE_IMAGE.
ARG LIVOX_IMAGE=ghcr.io/skymule-ai/mullet-drivers-livox:sha-8d0563c
FROM ${LIVOX_IMAGE} AS livox

FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=livox /opt/livox_ws/install /opt/livox_ws/install
COPY --from=livox /usr/local/lib/liblivox_lidar_sdk_shared.so /usr/local/lib/
RUN ldconfig
WORKDIR /opt/mullet_ws

COPY mullet_localization src/mullet_localization
# px4_msgs is in the base workspace: list its sources so rosdep does not look for it in apt.
RUN apt-get update \
 && rosdep install --from-paths src /opt/ros2_base_ws/src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/* \
 && source /usr/local/share/mullet-ros/setup.bash \
 && source /opt/livox_ws/install/setup.bash \
 && colcon build --packages-select mullet_localization \
 && colcon test --packages-select mullet_localization \
 && colcon test-result --verbose \
 && rm -rf build log

CMD ["bash", "-c", "source /opt/mullet_ws/install/setup.bash && exec ros2 run mullet_localization px4_ev_bridge"]
