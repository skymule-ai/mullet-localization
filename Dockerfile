# mullet-localization for the drone, on the mullet-ros2-base image (ROS 2 Jazzy, px4_msgs
# release/1.17). One stage per role:
#   common  sensor drivers (RealSense D435i, Livox Mid-360), foxglove_bridge, mullet_localization
#           (sensors launch, px4_ev_bridge). Serves the sensors, viz, record and ev_bridge services.
#   backends (openvins, isaac_vslam, vins_fusion, then fast_lio2, fast_livo2) will be stages
#           FROM common, one image tag each. None is built yet: see repos/.
# Tests run at build time, so a red test means no image.
ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-ros2-base:v0.5.0
FROM ${BASE_IMAGE} AS common
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# realsense-ros 4.59 needs librealsense 2.59: move both together (repos/drivers.repos).
ARG LIBREALSENSE_VERSION=v2.59.1
ARG LIVOX_SDK2_VERSION=v1.3.1

# librealsense on the RSUSB (libusb) backend: no Jetson kernel patches, and the D435i IMU works
# without the HID sensor modules L4T does not ship. No CUDA: VIO reads the raw infra stereo only.
RUN apt-get update \
 && apt-get install -y --no-install-recommends libssl-dev libudev-dev libusb-1.0-0-dev pkg-config \
 && rm -rf /var/lib/apt/lists/* \
 && git clone --depth 1 --branch ${LIBREALSENSE_VERSION} \
      https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
 && cmake -S /tmp/librealsense -B /tmp/librealsense/build -DCMAKE_BUILD_TYPE=Release \
      -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=OFF -DCHECK_FOR_UPDATES=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF \
 && cmake --build /tmp/librealsense/build -j"$(nproc)" \
 && cmake --install /tmp/librealsense/build \
 && ldconfig \
 && rm -rf /tmp/librealsense

# Livox-SDK2 uses std::uint8_t without <cstdint>, which GCC 13 (Ubuntu 24.04) no longer pulls in.
RUN git clone --depth 1 --branch ${LIVOX_SDK2_VERSION} \
      https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox-sdk2 \
 && cmake -S /tmp/livox-sdk2 -B /tmp/livox-sdk2/build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-include cstdint" \
 && cmake --build /tmp/livox-sdk2/build -j"$(nproc)" \
 && cmake --install /tmp/livox-sdk2/build \
 && ldconfig \
 && rm -rf /tmp/livox-sdk2

WORKDIR /opt/mullet_ws
COPY repos/drivers.repos drivers.repos
# livox_ros_driver2 ships per-ROS-version manifests; its build.sh copies the ROS 2 one into place.
# librealsense2 is skipped in rosdep because it is built above.
RUN mkdir -p src \
 && vcs import src < drivers.repos \
 && cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y --skip-keys librealsense2 \
 && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-foxglove-bridge \
 && rm -rf /var/lib/apt/lists/* \
 && source /usr/local/share/mullet-ros/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DROS_EDITION=ROS2 -DDISTRO_ROS=${ROS_DISTRO} \
 && rm -rf build log

COPY mullet_localization src/mullet_localization
# All source trees, so the drivers and px4_msgs count as present and apt never shadows them.
RUN apt-get update \
 && rosdep install --from-paths src /opt/ros2_base_ws/src --ignore-src -y --skip-keys librealsense2 \
 && rm -rf /var/lib/apt/lists/* \
 && source /usr/local/share/mullet-ros/setup.bash && source install/setup.bash \
 && colcon build --packages-select mullet_localization \
 && colcon test --packages-select mullet_localization \
 && colcon test-result --verbose \
 && rm -rf build log

# Defaults baked in; compose mounts the repo's config/ over them.
COPY config /config
CMD ["bash", "-c", "source /opt/mullet_ws/install/setup.bash && exec ros2 launch mullet_localization sensors.launch.py"]
