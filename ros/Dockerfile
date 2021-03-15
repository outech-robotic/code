FROM osrf/ros:noetic-desktop-full

RUN apt-get update \
  && apt-get install -y ros-noetic-socketcan-bridge

ENV PYTHONPATH="/opt/ros/melodic/lib/python3/dist-packages/:$PYTHONPATH"

CMD ["sh", "-c", "echo \"source /ros_entrypoint.sh && cd outech_ros/catkin_ws && catkin_make\" > /initfile"]
CMD ["bash", "-c", "roscore & bash --init-file /initfile"]

# source /ros_entrypoint.sh && cd outech_ros/catkin_ws && catkin_make