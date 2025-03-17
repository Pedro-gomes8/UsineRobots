

FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    tzdata \
    gnupg2 \
    lsb-release \
    git \
    curl \
    wget \
    build-essential \
    sudo \
    udev \
    python3-pip \
    #apt-transport-https \
    #ca-certificates \
 && rm -rf /var/lib/apt/lists

RUN apt-get remove --purge --auto-remove cmake -y

RUN apt-get update && apt-get install -y software-properties-common
RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.2/cmake-3.22.2-linux-x86_64.sh && \
    chmod +x cmake-3.22.2-linux-x86_64.sh && \
    ./cmake-3.22.2-linux-x86_64.sh --skip-license --prefix=/usr/local

RUN apt-get update && apt-get install -y \
    software-properties-common && \
    add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && apt-get install -y \
    gcc-10 g++-10 && \
    rm -rf /var/lib/apt/lists/*


RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100


RUN ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

# ----------------------------------------------------------
# ROS

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
 && rm -rf /var/lib/apt/lists/*


RUN rosdep init && rosdep update

RUN git config --global hub.protocol https
RUN git config --global user.email pedro.gomes@poli.ufrj.br
RUN git config --global user.name pedro-gomes8
RUN export GITHUB_TOKEN= !!!!!! !!! !!! Use a access token do github! - Nao vou puxar a minha
# ----------------------------------------------------------
# RealSense
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN echo "deb https://librealsense.intel.com/Debian/apt-repo bionic main" \
    | tee /etc/apt/sources.list.d/realsense.list

RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    librealsense2-dkms \
 && rm -rf /var/lib/apt/lists/*


RUN python3 -m pip install --upgrade pip
RUN pip3 install pyrealsense2

# ----------------------------------------------------------
# Catkin
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# ----------------------------------------------------------
# Niryo Ned
RUN git clone https://github.com/NiryoRobotics/ned_ros.git

# RealSense ROS wrapper 
RUN git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git

RUN apt-get update && apt-get install -y \
    ros-melodic-moveit-msgs \
    ros-melodic-moveit-core \
    ros-melodic-moveit-ros-planning \
    ros-melodic-moveit-ros-planning-interface \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-moveit-ros-perception \
    ros-melodic-moveit-ros-manipulation \
    ros-melodic-moveit-planners-ompl \
    ros-melodic-moveit-simple-controller-manager \
    ros-melodic-ddynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*


RUN echo "Checking ROS Installation..." && ls /opt/ros/melodic/setup.bash
RUN ls /opt/ros/melodic
WORKDIR $CATKIN_WS
RUN rm -rf build devel  # Delete old build files
#RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make -j1"


# ----------------------------------------------------------

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
