FROM ros:jazzy-ros-base

# MAVROS + geographiclib datasets
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    ros-jazzy-cv-bridge \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    python3-colcon-common-extensions \
    python3-rosdep \
    && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \

WORKDIR /dagozilla_ws
COPY src ./src
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --base-paths src

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /dagozilla_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
