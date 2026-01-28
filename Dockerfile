FROM ros:foxy-ros-base-focal
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
ENV ROS_WS=/ws

WORKDIR ${ROS_WS}

# =========================
# 1) Base deps + rosdep
# =========================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    pkg-config \
    libeigen3-dev \
    libglm-dev \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    libx11-dev \
    libxext-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxfixes-dev \
    libxi-dev \
    libudev-dev \
    libgl-dev \
    ros-foxy-rmw-cyclonedds-cpp \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || true \
  && rosdep update

# =========================
# 2) SDL3 (build from source)
# =========================
RUN git clone --branch release-3.2.2 --depth 1 https://github.com/libsdl-org/SDL.git /tmp/SDL \
  && cmake -S /tmp/SDL -B /tmp/SDL/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DSDL_STATIC=OFF \
      -DSDL_SHARED=ON \
      -DSDL_TESTS=OFF \
  && cmake --build /tmp/SDL/build -j"$(nproc)" \
  && cmake --install /tmp/SDL/build \
  && ldconfig \
  && rm -rf /tmp/SDL

RUN mkdir -p ${ROS_WS}/src

# =========================
# 3) Copy package.xml first (cache)
# =========================
COPY Mobility_Challenge_Simulator/src/communication_manager/package.xml ${ROS_WS}/src/communication_manager/package.xml
COPY Mobility_Challenge_Simulator/src/domain_bridge/package.xml         ${ROS_WS}/src/domain_bridge/package.xml
COPY Mobility_Challenge_Simulator/src/hv_handler/package.xml           ${ROS_WS}/src/hv_handler/package.xml
COPY Mobility_Challenge_Simulator/src/scene_srv/package.xml            ${ROS_WS}/src/scene_srv/package.xml
COPY Mobility_Challenge_Simulator/src/simulator/package.xml            ${ROS_WS}/src/simulator/package.xml
COPY Mobility_Challenge_Simulator/src/simulator_launch/package.xml     ${ROS_WS}/src/simulator_launch/package.xml

COPY pkg_task_1_1/package.xml ${ROS_WS}/src/pkg_task_1_1/package.xml
COPY pkg_task_1_2/package.xml ${ROS_WS}/src/pkg_task_1_2/package.xml
COPY pkg_task_2/package.xml   ${ROS_WS}/src/pkg_task_2/package.xml
COPY pkg_task_3/package.xml   ${ROS_WS}/src/pkg_task_3/package.xml

# =========================
# 4) rosdep
# =========================
RUN apt-get update \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
  && rm -rf /var/lib/apt/lists/*

# =========================
# 5) Copy full sources
# =========================
COPY Mobility_Challenge_Simulator/src/ ${ROS_WS}/src/
COPY pkg_task_1_1/ ${ROS_WS}/src/pkg_task_1_1/
COPY pkg_task_1_2/ ${ROS_WS}/src/pkg_task_1_2/
COPY pkg_task_2/   ${ROS_WS}/src/pkg_task_2/
COPY pkg_task_3/   ${ROS_WS}/src/pkg_task_3/

# =========================
# 5.5) Copy profiles (with trailing spaces preserved)
# =========================
# profiles/ 폴더 안에 profile*.json␠ 들이 들어있어야 함
COPY profiles/ /ws/

# =========================
# entrypoint
# =========================
COPY entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

# =========================
# 6) Build
# =========================
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

ENV ROS_LOCALHOST_ONLY=0
ENTRYPOINT ["/entrypoint.sh"]
