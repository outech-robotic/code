FROM osrf/ros:noetic-desktop-full

ARG USER_ID
ARG PROJECT_ROOT
RUN apt-get update
RUN apt-get install -y ros-noetic-socketcan-bridge git software-properties-common 
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get install -y python3.7 python3-pip
RUN pip3 install protobuf 
RUN adduser --disabled-password --gecos '' --uid $USER_ID user
USER user

ENV PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages/:$PROJECT_ROOT:$PYTHONPATH"

CMD ["sh", "-c", "roscore & bash --init-file /outech/ros/initfile"]
