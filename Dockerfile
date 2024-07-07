from viitrobotics/r2ws:base

ENV ROS_DISTRO=humble
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 daemon start"

# set arguments
ARG USER_NAME=rosdev
ARG UID=1000
ARG GID=${UID}

RUN apt-get install -y vim && \
    pip3 install rosdep colcon-common-extensions 

# setup username
RUN groupadd --gid ${GID} ${USER_NAME} \
    && useradd -s /bin/bash --uid ${UID} --gid ${GID} -m ${USER_NAME} \
    && mkdir /home/${USER_NAME}/.config && chown ${UID}:${GID} /home/${USER_NAME}/.config


RUN usermod -aG video ${USER_NAME}
#  enable sudo permissions
RUN echo "rosdev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/humble/setup.bash " >> /root/.bashrc && echo "source /opt/ros/humble/setup.bash"  >> /home/rosdev/.bashrc


USER ${USER_NAME}
# # build source
COPY ./ /ws/src/
WORKDIR /ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep init && rosdep update && rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install"

CMD /bin/bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch r2_bringup go.launch.py"



