# Ubuntu 22.04 build environment for CrazySim (Crazyflie firmware SITL + Gazebo Garden).
# Kept separate from the host (Ubuntu 24.04, Gazebo Harmonic, ROS 2 Jazzy).
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl wget gnupg lsb-release git sudo \
        build-essential cmake \
        python3 python3-pip python3-venv python3-jinja2 python-is-python3 \
        libboost-program-options-dev libusb-1.0-0-dev \
        libgl1 xvfb \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Garden from the OSRF repo. Garden is end-of-life, so this is the step
# most likely to fail. It is its own layer so the error is easy to read.
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list
RUN apt-get update && apt-get install -y gz-garden \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching the host UID/GID, so files written to the bind mount
# are owned by you, not root.
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} dev && useradd -m -u ${UID} -g ${GID} -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev
USER dev
ENV PATH="/home/dev/.local/bin:${PATH}"
WORKDIR /work
