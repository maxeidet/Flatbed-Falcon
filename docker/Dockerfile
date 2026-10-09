# Bygger ovanpå PX4:s officiella image (PX4 + Gazebo + ROS 2 + uXRCE-DDS-agent).
# Digest pinnad så att alla i teamet får exakt samma miljö.
FROM px4io/px4-sitl-gazebo-ros2@sha256:2c01c89215a5f541d3a819c1bd2115ff27c9423679f3738dbb4096271d77084b

USER root

# Xvfb = virtuell display att rendera Gazebo mot (containern har ingen egen skärm).
# fluxbox = lättviktig fönsterhanterare, krävs för att Gazebos fönster ska bete sig normalt.
# x11vnc = VNC-server som delar Xvfb-displayen (native VNC-klient, port 5900).
# novnc + websockify = webbläsarbaserad VNC-klient (öppnas via localhost:6080).
RUN apt-get update && apt-get install -y \
    xvfb \
    x11vnc \
    fluxbox \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# ros_gz_bridge = forwards the truck's Gazebo topics (cmd_vel, odometry, bed contact,
# clock) and the gimbal camera to ROS 2. python3-scipy = Lab 3's SplinePath and
# solve_discrete_are (LQR). rviz2 = shows the gimbal camera (ros2_ws/view-camera.sh).
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-rviz2 \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# ros_gz_bridge pulls in ROS's own gz-transport/gz-msgs "vendor" packages. Sourcing ROS then
# points GZ_CONFIG_PATH at only those, which hides the system Gazebo's `gz sim` command, so
# PX4 (whose entrypoint sources ROS) cannot start Gazebo. ROS prepends to this value, so
# setting it here keeps the system Gazebo commands available.
ENV GZ_CONFIG_PATH=/usr/share/gz

# Simulation only: the x500 airframe (and every x500 variant, they all source it) sets
# NAV_DLL_ACT=2, "return home when the ground-station link is lost". With that on, PX4
# refuses to arm ("No connection to the GCS") until QGroundControl is connected. Our
# backend arms over ROS 2 and there is no QGC on Stellar, so switch it off (0 = disabled).
# Fails the build if the line is not found, so a changed base image is noticed.
RUN grep -q "^param set-default NAV_DLL_ACT 2$" /opt/px4-gazebo/etc/init.d-posix/airframes/4001_gz_x500 \
    && sed -i 's/^param set-default NAV_DLL_ACT 2$/param set-default NAV_DLL_ACT 0/' /opt/px4-gazebo/etc/init.d-posix/airframes/4001_gz_x500

# PX4 waits only 30 checks (~30 s) for the Gazebo world before giving up with
# "Timed out waiting for Gazebo world". Heavy worlds (hive_base, baylands) can take longer
# on a Mac, so read the count from PX4_GZ_WORLD_TIMEOUT instead (default stays 30).
# Fails the build if the line is not found, so a changed base image is noticed.
RUN grep -q "^ATTEMPTS=30$" /opt/px4-gazebo/etc/init.d-posix/px4-rc.gzsim \
    && sed -i 's/^ATTEMPTS=30$/ATTEMPTS=${PX4_GZ_WORLD_TIMEOUT:-30}/' /opt/px4-gazebo/etc/init.d-posix/px4-rc.gzsim

ENV DISPLAY=:1
ENV VNC_PORT=5900
ENV NOVNC_PORT=6080

COPY start-vnc.sh /usr/local/bin/start-vnc.sh
RUN chmod +x /usr/local/bin/start-vnc.sh

# 5900 = native VNC-klient (t.ex. macOS Skärmdelning, RealVNC Viewer)
# 6080 = noVNC via webbläsare (http://localhost:6080)
EXPOSE 5900 6080

ENTRYPOINT ["/usr/local/bin/start-vnc.sh"]
CMD ["/bin/bash"]
