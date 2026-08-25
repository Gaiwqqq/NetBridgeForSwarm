# syntax=docker/dockerfile:1.7

ARG ROS_IMAGE=ros:noetic-ros-core-focal

FROM ${ROS_IMAGE} AS builder

ARG TARGETARCH
ARG BUILD_JOBS=2
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ros-core already provides catkin, message generation and the standard message
# packages. Only the two additional ROS development packages used by this
# project are installed here.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      libopencv-core-dev \
      libopencv-imgcodecs-dev \
      libopencv-imgproc-dev \
      libpcl-dev \
      ros-noetic-pcl-conversions \
      ros-noetic-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# The repository contains Focal-compatible Zenoh 1.9 packages for amd64 and
# arm64. A bind mount keeps the .deb files out of the image layers.
RUN --mount=type=bind,source=zenoh-debs,target=/tmp/zenoh-debs,ro \
    test -n "${TARGETARCH}" \
    && test -d "/tmp/zenoh-debs/${TARGETARCH}" \
    && dpkg -i /tmp/zenoh-debs/${TARGETARCH}/*.deb

WORKDIR /ws
COPY cv_bridge_noetic_fit_version /ws/src/NetBridgeForSwarm/cv_bridge_noetic_fit_version
COPY swarm_ros_bridge /ws/src/NetBridgeForSwarm/swarm_ros_bridge
COPY third_party /ws/src/NetBridgeForSwarm/third_party

RUN ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake /ws/src/CMakeLists.txt \
    && source /opt/ros/noetic/setup.bash \
    && catkin_make install -j"${BUILD_JOBS}" \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/opt/netbridge \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DBUILD_SHARED_LIBS=OFF \
         -DCATKIN_ENABLE_TESTING=OFF \
         -DCV_BRIDGE_BUILD_PYTHON=OFF \
         -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && strip --strip-unneeded \
         /opt/netbridge/lib/swarm_ros_bridge/bridge_new \
         /opt/netbridge/lib/swarm_ros_bridge/bridge_tui \
    && rm -rf \
         /opt/netbridge/include \
         /opt/netbridge/lib/libcv_bridge_noetic_fit_version.a \
         /opt/netbridge/lib/pkgconfig \
         /opt/netbridge/share/cv_bridge_noetic_fit_version/cmake \
         /opt/netbridge/share/swarm_ros_bridge/cmake


FROM ${ROS_IMAGE} AS runtime

ARG TARGETARCH
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Runtime-only dependencies. In particular, no ROS desktop/perception
# metapackage, compiler, PCL headers or OpenCV headers are retained.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libopencv-core4.2 \
      libopencv-imgcodecs4.2 \
      libopencv-imgproc4.2 \
      libpcl-common1.10 \
      libpcl-filters1.10 \
      libpcl-io1.10 \
      libpcl-octree1.10 \
      ros-noetic-topic-tools \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source=zenoh-debs,target=/tmp/zenoh-debs,ro \
    test -n "${TARGETARCH}" \
    && test -d "/tmp/zenoh-debs/${TARGETARCH}" \
    && dpkg -i /tmp/zenoh-debs/${TARGETARCH}/libzenohc_*.deb

COPY --from=builder /opt/netbridge /opt/netbridge
COPY docker/entrypoint.sh /netbridge_entrypoint.sh

ENV ROS_DISTRO=noetic

ENTRYPOINT ["/netbridge_entrypoint.sh"]
CMD ["roslaunch", "swarm_ros_bridge", "example_bridge_station.launch"]
