# Use an official ROS base image
FROM ros:noetic-ros-core-focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    build-essential \
    cmake \
    psmisc \
    libpcl-dev \
    liboctomap-dev \
    libeigen3-dev \
    libboost-all-dev \
    ros-noetic-pcl-ros \
    ros-noetic-octomap-ros \
    ros-noetic-octovis \
    python3-catkin-tools \
    pcl-tools \
    libflann-dev \
    libvtk7-qt-dev \
    libvtk7-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    freeglut3-dev \
    libgtest-dev \
    qt5-default \
    qtbase5-dev \
    libqt5opengl5-dev \
    qttools5-dev \
    qttools5-dev-tools \
    libx11-dev \
    libxext-dev \
    libxi-dev \
    libxmu-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxinerama-dev \
    libglu1-mesa-dev \
    pkg-config \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Python requirements
RUN pip3 install evo scipy

# Install CloudCompare
RUN apt-get update                                          &&\
    apt-get install -y git-core vim binutils                &&\
    apt-get install -y cmake g++ gcc                        &&\
    apt-get install -y libgdal-dev libfreenect-dev          \
                        libeigen3-dev libtbb-dev            \
                        libavcodec-dev libavformat-dev      \
                        libavutil-dev libboost-thread-dev   \
                        libboost-program-options-dev        \
                        libcgal-dev libcgal-qt5-dev         \
                        libdlib-dev libswscale-dev          \
                        libtbb-dev libqt5opengl5-dev        \
                        qt5-default qttools5-dev            \
                        qttools5-dev-tools libqt5svg5-dev   \
                        ros-noetic-eigen-conversions \
                        libproj-dev libdlib-dev             &&\
    apt-get clean

RUN git clone --branch v2.10.2 --single-branch --recursive \
        https://github.com/CloudCompare/CloudCompare.git

ADD docker-cloudcompare/dockerfile/build /root/CloudCompare/build
RUN cd /root/CloudCompare/build  &&\
    ./configure.sh          &&\
    make                    &&\
    make install            &&\
    make clean


# Set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]
