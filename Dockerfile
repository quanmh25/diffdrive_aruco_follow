ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=0

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-rosdep \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry-msgs \
	    ros-${ROS_DISTRO}-robot-state-publisher \
	    ros-${ROS_DISTRO}-rviz2 \
	    ros-${ROS_DISTRO}-rqt-image-view \
	    ros-${ROS_DISTRO}-sensor-msgs \
	    ros-${ROS_DISTRO}-nav-msgs \
	    ros-${ROS_DISTRO}-tf2-ros \
	    ros-${ROS_DISTRO}-tf2-tools \
	    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    if getent passwd "${USER_UID}" >/dev/null; then \
      existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
      if [ "${existing_user}" != "${USERNAME}" ]; then \
        usermod --login "${USERNAME}" "${existing_user}"; \
        usermod --home "/home/${USERNAME}" --move-home "${USERNAME}" || true; \
      fi; \
    else \
      if ! getent group "${USER_GID}" >/dev/null; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
      fi; \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}"; \
    fi; \
    usermod -aG video,render,dialout "${USERNAME}" 2>/dev/null || true; \
    mkdir -p "/home/${USERNAME}"; \
    chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}"

RUN rosdep init 2>/dev/null || true

USER ${USERNAME}
WORKDIR /home/${USERNAME}/diffdrive_aruco_follow/workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
	    && echo "if [ -f /home/${USERNAME}/diffdrive_aruco_follow/workspace/install/setup.bash ]; then source /home/${USERNAME}/diffdrive_aruco_follow/workspace/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

CMD ["bash"]
