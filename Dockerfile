# syntax=docker/dockerfile:1.7.0@sha256:dbbd5e059e8a07ff7ea6233b213b36aa516b4c53c645f1817a4dd18b83cbea56
# Ubuntu Noble; digest obtained from the official library/ubuntu registry.
ARG BASE_IMAGE=ubuntu:24.04@sha256:2198e4c65076e9cb0b3d695219df667c0e3b0072da64ca3c2648aca17dcecaf5
FROM ${BASE_IMAGE}

ARG UBUNTU_SNAPSHOT=20260619T000000Z
ARG ROS_SNAPSHOT=2026-06-18
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 ROS_DISTRO=jazzy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Freeze both repositories. apt still verifies their signed Release/Packages files.
# ROS snapshots currently serve HTTP; do not disable TLS verification to use HTTPS.
RUN sed -i -E "s|http://(archive.ubuntu.com/ubuntu|security.ubuntu.com/ubuntu)/?|http://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT}/|g" \
      /etc/apt/sources.list.d/ubuntu.sources
ADD --chmod=0644 --checksum=sha256:4a91c49af0d6f0016108b93698782b596c27ccd836937e18e0e36c3347dc602f \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential cmake pkg-config git ca-certificates \
      python3-venv python3-colcon-common-extensions python3-rosdep \
      python3-numpy python3-scipy python3-yaml python3-psutil python3-pytest \
      python3-flask python3-werkzeug python3-requests python3-retrying \
      python3-nest-asyncio python3-importlib-metadata python3-typing-extensions \
      python3-packaging python3-setuptools \
      ros-jazzy-ros-base ros-jazzy-ros-gz \
      ros-jazzy-gz-physics-vendor=0.0.7-1noble.20260604.105412 \
      ros-jazzy-gz-dartsim-vendor=0.0.3-1noble.20260225.232143 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/testbed
COPY . .
RUN test -d .git && git cat-file -e 'HEAD^{tree}' \
    && git config --system --add safe.directory /opt/testbed \
    && python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir --no-deps --require-hashes \
      -r tools/docker/requirements.txt \
    && rosdep init \
    && rosdep update --rosdistro jazzy \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
      -t build -t build_export -t buildtool -t buildtool_export -t exec \
      --skip-keys 'ament_python python3-dash-pip python3-plotly' \
    && rm -rf /var/lib/apt/lists/*

ENV PATH=/opt/venv/bin:$PATH \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
    CMAKE_BUILD_PARALLEL_LEVEL=2 GIT_OPTIONAL_LOCKS=0
RUN source /opt/ros/jazzy/setup.bash \
    && python /usr/bin/colcon build --merge-install --parallel-workers 2 \
      --event-handlers console_direct+ \
    && source install/setup.bash \
    && mkdir -p /opt/testbed-image \
    && python tools/docker/verify_install.py > /opt/testbed-image/installation.json \
    && dpkg-query -W > /opt/testbed-image/apt-packages.txt \
    && python -m pip freeze > /opt/testbed-image/python-packages.txt \
    && sha256sum Dockerfile compose.yaml .dockerignore tools/docker/* \
      > /opt/testbed-image/deployment-files.sha256

# Named-volume copy-up preserves this ownership on first creation. No root entrypoint.
RUN groupadd --gid 10001 testbed \
    && useradd --uid 10001 --gid 10001 --create-home --shell /bin/bash testbed \
    && install -d -o testbed -g testbed /data \
    && chmod 755 tools/docker/entrypoint.sh
USER 10001:10001
ENV HOME=/home/testbed XDG_CACHE_HOME=/data/cache ROS_LOG_DIR=/data/ros-log \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST QT_QPA_PLATFORM=offscreen
EXPOSE 8050
STOPSIGNAL SIGTERM
HEALTHCHECK --interval=15s --timeout=5s --start-period=60s --retries=4 \
    CMD python -c "import json,urllib.request; r=json.load(urllib.request.urlopen('http://127.0.0.1:8050/dashboard-health',timeout=3)); assert r['service']=='robot_dashboard' and r['catalog']=='/data/catalog.yaml'"
ENTRYPOINT ["/opt/testbed/tools/docker/entrypoint.sh"]
CMD ["/opt/testbed/install/lib/robot_dashboard/dashboard", "--host", "0.0.0.0", "--port", "8050", "--data-dir", "/data", "--stop-jobs-on-exit"]
