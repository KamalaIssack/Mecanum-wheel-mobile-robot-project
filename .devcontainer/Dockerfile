# Codespace dev environment: ROS 2 Jazzy (ros-base, the same variant as the Pi)
# plus Gazebo Harmonic. Workspace dependencies come from package.xml through
# rosdep in postCreateCommand; only what package.xml must not declare is here.
FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive

# Gazebo arrives as Jazzy vendor packages with ros_gz. Kept out of package.xml
# so rosdep on the Pi never installs a simulator. The package lists are
# deleted, so any later install must run apt-get update first.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Noble images already ship "ubuntu" at UID 1000; reuse it rather than create
# a clashing user. sudo rejects drop-in files that are not 0440.
ARG USERNAME=ubuntu
RUN echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# VS Code terminals bypass ros_entrypoint.sh. Underlay only: the workspace
# overlay is sourced by hand, so a stale install/ is never picked up silently.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc
