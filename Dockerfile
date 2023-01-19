FROM ubuntu:20.04

ENV LANG en_US.UTF-8

RUN apt update && apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install -y \
build-essential \
curl \
gnupg2 \
lsb-release \
&& rm -rf /var/lib/apt/lists/*


RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN apt update && apt upgrade -y && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-foxy-desktop python3-rosdep2
RUN apt install -y ros-dev-tools


RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

COPY . /home/ros_ws/
WORKDIR /home/ros_ws

ENTRYPOINT [ "/home/ros_ws/ros_entrypoint.sh" ]
