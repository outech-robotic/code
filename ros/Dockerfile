FROM osrf/ros:noetic-desktop-full

RUN apt-get update \
  && apt-get install -y ros-noetic-socketcan-bridge git python3.7

ENV PYTHONPATH="/opt/ros/melodic/lib/python3/dist-packages/:$PYTHONPATH"

CMD ["sh", "-c", "roscore & bash --init-file /outech/ros/initfile"]
