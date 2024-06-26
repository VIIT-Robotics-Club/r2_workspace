from viitrobotics/r2ws:base

ENV ROS_DISTRO=humble

# set arguments
ARG USRNAME=rosdev
ARG UID=1000
ARG GID=${UID}

RUN apt-get install -y sudo && \
    pip3 install rosdep colcon-common-extensions 

# setup username
RUN groupadd --gid ${GID} ${USRNAME} \
    && useradd -s /bin/bash --uid ${UID} --gid ${GID} -m ${USRNAME} \
    && mkdir /home/${USRNAME}/.config && chown ${UID}:${GID} /home/${USRNAME}/.config


#  enable sudo permissions
RUN echo "rosdev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/humble/setup.bash " >> /root/.bashrc && echo "source /opt/ros/humble/setup.bash"  >> /home/rosdev/.bashrc


# # build source
COPY ./ /ws/src/
WORKDIR /ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep init && rosdep update && rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install"

CMD /bin/bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch r2_bringup testing.launch.py"



