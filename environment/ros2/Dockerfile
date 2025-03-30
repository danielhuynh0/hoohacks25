FROM osrf/ros:humble-desktop-full

ENV LIBVA_DRIVER_NAME=d3d12
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

RUN apt-get update && apt-get install -y software-properties-common curl
RUN add-apt-repository ppa:kisak/kisak-mesa -y

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-gazebo-ros \ 
    ros-humble-rviz-imu-plugin \
    wget \
    vainfo \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -m hack && echo "hack:hack123" | chpasswd && adduser hack sudo
USER hack
WORKDIR /home/hack

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "alias rosup='sudo apt update; rosdep update; rosdep install --from-paths ~/ws/src -y --ignore-src'" >> ~/.bashrc
RUN echo "alias rossrc='source ~/ws/install/setup.bash'" >> ~/.bashrc
