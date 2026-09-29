# Metro obstacle detector: ROS 2 Humble node (ros/metro_detector) + a machine-bound detection
# engine (ros/activation).
#
# The detection engine ships in this repository ENCRYPTED, as ros/engine.enc -- its committed,
# fixed hash is the record that the delivered binary is frozen as of delivery. It is never in the
# clear here. During `docker build` -- on YOUR stand, which has internet during the build -- the
# activation step contacts our server, which returns ONLY the key that unlocks ros/engine.enc,
# bound to this machine (and a signed, expiring licence). At start-up the node decrypts the
# committed engine into memory. The plaintext engine never touches disk. See ros/README.md.
#
# Build from the REPOSITORY ROOT (the context needs ros/):
#   docker build -f ros/Dockerfile -t metro_detector .
# Run (host network so the node sees `ros2 bag play` / the lidar driver on the host):
#   docker run --rm -it --net=host --ipc=host metro_detector

ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

# Delivery token, baked into the delivery repo, overridable at build time:
#   docker build --build-arg METRO_DELIVERY_TOKEN=... --build-arg METRO_ACTIVATION_URL=...
ARG METRO_DELIVERY_TOKEN=comexp-metro-delivery-2026
ARG METRO_ACTIVATION_URL=https://ml.comexp.net/api/metro/activate

ENV PYTHONUNBUFFERED=1 \
    METRO_ACTIVATION_DIR=/opt/metro/activation \
    METRO_ENGINE_ENC=/opt/metro/engine.enc \
    METRO_CONTACT=obstacle-detector@comexp.net \
    PYTHONPATH=/opt/metro/pkg

# The run-time loader needs python `cryptography` to open the engine. Installed here (the build
# stage has internet on the stand). ros:humble is Ubuntu 22.04, so no PEP 668 override is needed.
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && pip3 install --no-cache-dir cryptography \
    && rm -rf /var/lib/apt/lists/*

# The committed, encrypted engine (unlocked at run time with the key from activation).
COPY ros/engine.enc /opt/metro/engine.enc

# Activation code: fingerprint / loader / wire are shared by build time and run time.
COPY ros/activation /opt/metro/pkg/activation

# ---- ACTIVATION -----------------------------------------------------------------------------
# Runs on the customer's stand during the build. Computes this machine's fingerprint, fetches the
# machine-bound key + signed licence for ros/engine.enc, and writes them (manifest.json +
# license.json) into the image. Fails the build LOUDLY with the fingerprint + manual fallback if
# the server is unreachable. The engine itself is not fetched -- it is the committed ros/engine.enc.
RUN METRO_DELIVERY_TOKEN="${METRO_DELIVERY_TOKEN}" METRO_ACTIVATION_URL="${METRO_ACTIVATION_URL}" \
    python3 /opt/metro/pkg/activation/activate.py

# ---- ROS 2 node -----------------------------------------------------------------------------
COPY ros/metro_detector /opt/metro/ws/src/metro_detector
COPY ros/docker/build_node.sh ros/docker/entrypoint.sh ros/docker/fastdds_large.xml /opt/metro/
RUN bash /opt/metro/build_node.sh /opt/metro/ws && rm -rf /opt/metro/ws/build /opt/metro/ws/log
COPY ros/tools /opt/metro/tools
WORKDIR /opt/metro
ENTRYPOINT ["/opt/metro/entrypoint.sh"]
CMD ["ros2", "launch", "metro_detector", "metro_detector.launch.py"]
