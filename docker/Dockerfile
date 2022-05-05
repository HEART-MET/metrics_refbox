ARG ROS_DISTRO
FROM osrf/ros:$ROS_DISTRO-desktop-full
ENV QT_X11_NO_MITSHM 1

RUN apt-get update
RUN apt-get -y install git ros-$ROS_DISTRO-rospy-message-converter
RUN mkdir -p /home/root/catkin_ws/src
WORKDIR /home/root/catkin_ws/src
RUN git clone https://github.com/HEART-MET/metrics_refbox.git
RUN git clone https://github.com/HEART-MET/metrics_refbox_msgs.git
RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin_init_workspace /home/root/catkin_ws/src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd /home/root/catkin_ws && catkin_make

# Copy in the entrypoint script
COPY start.sh /
RUN chmod +x /start.sh
ENTRYPOINT [ "/start.sh"]
CMD [$ROS_DISTRO]

