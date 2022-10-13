ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=melodic-desktop-full
FROM ${BASE_IMAGE}:${BASE_TAG}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
 wget \
 git \
 bash-completion \
 build-essential \
 sudo \
 vim \
 python3-pip \
 python3-future \
 python-future \
 python3-toml \
 python3-jinja2 \
 python3-empy \
 python3-numpy \
 python3-packaging \
 python3-yaml \
 openjdk-11-jdk \
 software-properties-common \
 libgstreamer-plugins-base1.0-dev \
 && rm -rf /var/lib/apt/lists/*
 
# Now create the same user as the host itself
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ros
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ros
RUN usermod -a -G dialout ros
ADD config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

#WORKDIR ${HOME}/catkin_ws/
#COPY --chown=${USER} ./source/ ./src/

#RUN wget -P /home/ros https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
#RUN chmod +x /home/ros/ubuntu_sim_ros_melodic.sh

# Choose to run as user
ENV USER ros
USER ros
# Change HOME environment variable
ENV HOME /home/${USER}
# workspace setup
RUN mkdir -p ${HOME}/catkin_ws/src && mkdir -p ${HOME}/src/installRealSenseROS

#Testing###########################
#Testing###########################

#RUN ./home/ros/ubuntu_sim_ros_melodic.sh
#RUN sudo rm -rf /etc/apt/sources.list.d/ros1*
# set up environment
COPY config/px4_build.sh /sbin/px4_build.sh
RUN sudo chmod +x /sbin/px4_build.sh ; sudo chown ros /sbin/px4_build.sh ; sync ; /bin/bash -c /sbin/px4_build.sh ; sudo rm /sbin/px4_build.sh
# Change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
