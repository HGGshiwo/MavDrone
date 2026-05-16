ARG BASE_IMAGE=ubuntu:24.04

###########################################
#  Base image
###########################################
FROM ${BASE_IMAGE} AS base
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
	&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	&& apt-get update \
	&& apt-get install -y sudo \
	&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
	&& chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

# Install language
RUN apt-get update && apt-get install -y \
	locales \
	&& locale-gen en_US.UTF-8 \
	&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
	&& rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
	&& export DEBIAN_FRONTEND=noninteractive \
	&& apt-get update \
	&& apt-get install -y tzdata \
	&& dpkg-reconfigure --frontend noninteractive tzdata \
	&& rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash
ENV PIP_BREAK_SYSTEM_PACKAGES=1 

USER $USERNAME
ENV HOME=/home/$USERNAME
WORKDIR ${HOME}

CMD ["/bin/bash"]

###########################################
#  OpenCV with Gstreamer
###########################################

FROM base AS opencv_with_gstreamer

USER root
WORKDIR ${HOME}

ENV OPENCV_VERSION=4.x

RUN sudo apt-get update && sudo apt-get install -y \
	python3-dev \
	python3-setuptools \
	python3-wheel \
	cmake \
	git \
	gstreamer1.0-gl \
	gstreamer1.0-opencv \
	gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-good \
	gstreamer1.0-plugins-ugly \
	gstreamer1.0-tools \
	libgstreamer-plugins-base1.0-dev \
	libgstreamer1.0-0 \
	libgstreamer1.0-dev \
	gstreamer1.0-libav \
	libgtk2.0-dev \
	pkg-config \
	&& rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN git clone --branch ${OPENCV_VERSION} --depth 1 --recurse-submodules --shallow-submodules https://github.com/opencv/opencv-python.git opencv-python-${OPENCV_VERSION} && \
	cd opencv-python-${OPENCV_VERSION} && \
	export ENABLE_CONTRIB=0 && \
	export ENABLE_HEADLESS=0 && \
	export CMAKE_ARGS="-DWITH_GSTREAMER=ON" && \
	python3 -m pip wheel . --verbose && \
	python3 -m pip install opencv_python*.whl

CMD ["/bin/bash"]

###########################################
#  MAVLink Router 
###########################################
FROM opencv_with_gstreamer AS mavlink_router

USER root
WORKDIR ${HOME}

RUN apt-get update && apt-get install -y --no-install-recommends \
	git \
	meson \
	ninja-build \
	pkg-config \
	gcc \
	g++ \
	systemd \
	&& rm -rf /var/lib/apt/lists/*

USER $USERNAME
RUN git clone --recursive https://github.com/mavlink-router/mavlink-router.git && \
	cd ${HOME}/mavlink-router && \
	meson setup build && \
	ninja -C build

USER root
RUN cd ${HOME}/mavlink-router && ninja -C build install

USER $USERNAME

CMD ["/bin/bash"]

###########################################
#  Develop image
###########################################
FROM mavlink_router AS dev

USER root
WORKDIR ${HOME}

ENV DEBIAN_FRONTEND=noninteractive

USER $USERNAME

CMD ["/bin/bash"]

