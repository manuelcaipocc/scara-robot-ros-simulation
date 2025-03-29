# Dockerfile
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Dependencias básicas
RUN apt update && apt install -y \
    git \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    build-essential \
    python3-pip \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libxcb-xinerama0 \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-state-publisher-gui \
    ros-humble-launch \
    ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt update && apt install -y \
    ros-humble-launch-testing-ament-cmake

# Instalar colcon
RUN apt update && apt install -y python3-colcon-common-extensions
# GUI interactiva de MoveIt
RUN apt install -y ros-humble-moveit-visual-tools

# Instalar puente ROS <-> Gazebo (ros_gz)
RUN apt install -y \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim

# Crear workspace ROS 2
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Configuración del entorno
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
