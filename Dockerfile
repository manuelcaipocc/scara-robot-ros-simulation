# Dockerfile
FROM osrf/ros:noetic-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Herramientas básicas y dependencias GUI
RUN apt update && apt upgrade -y && apt install -y \
    git \
    curl \
    wget \
    vim \
    build-essential \
    python3-pip \
    python3-rosdep \
    x11-apps \
    qt5dxcb-plugin \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxkbcommon-x11-0 \
    libxkbcommon0 \
    libfontconfig1 \
    libglu1-mesa \
    libqt5widgets5 \
    libqt5gui5 \
    libxrender1 \
    libxcb-xinput0 \
    libxcb-shape0 \
    libxcb-randr0 \
    libxcb-util1 \
    libxcomposite1 \
    libxi6 \
    libxrandr2 \
    libxfixes3 \
    libxcursor1 \
    libxinerama1 \
    libxss1 \
    libglib2.0-0 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-utils \
    ros-noetic-rqt* \
    ros-noetic-xacro \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-joint-state-publisher-gui \
 && rm -rf /var/lib/apt/lists/*
RUN apt update && apt install -y ros-noetic-moveit
RUN apt update && apt install -y \
    python3-numpy \
    python3-scipy \
    python3-sympy \
    python3-matplotlib

# Inicializar rosdep
RUN rosdep init || true && rosdep update

# Crear y configurar el workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Configuración de entorno
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash" >> /root/.bashrc \
 && echo "export QT_X11_NO_MITSHM=1" >> /root/.bashrc \
 && echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> /root/.bashrc \
 && echo "mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root" >> /root/.bashrc \
#  && echo "export DISPLAY=host.docker.internal:0.0" >> /root/.bashrc \
 && echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc

# Compilación inicial vacía con catkin
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ros_ws && catkin_make || true"


CMD ["/bin/bash"]
