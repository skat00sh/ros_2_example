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


RUN apt update && apt upgrade 
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-desktop
RUN apt install ros-dev-tools

# WORKDIR /root/dev_ws/src
# # RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
# WORKDIR /root/dev_ws

# COPY ros2_entrypoint.sh /root/.
# ENTRYPOINT ["/root/ros2_entrypoint.sh"]
# CMD ["bash"]
# COPY ./opt/ros/foxy/setup.bash .
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
# ENTRYPOINT ["/bin/bash", "-c", "./setup.bash"]
# RUN . /opt/ros/foxy/setup.sh
# ENTRYPOINT [ "/bin/bash" , "-c", "ros2 run demo_nodes_cpp talker"]
CMD ["ros2 run demo_nodes_cpp talker"}
# RUN mkdir /home/ros_ws