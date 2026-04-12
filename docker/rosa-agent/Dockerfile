# rosa-agent Dockerfile
# Runs NASA JPL ROSA in read-only mode, connected to ainex ROS master.
# Python 3.9+ required for ROSA; ROS Noetic user-space tools added via deadsnakes + apt.
# Hardware access: NONE. Only reads topics/services from ainex via ROS_MASTER_URI.

FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ---------- 1. System dependencies ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    wget \
    git \
    lsb-release \
    gnupg2 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ---------- 2. Python 3.9 via deadsnakes ----------
RUN add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y --no-install-recommends \
    python3.9 \
    python3.9-dev \
    python3.9-distutils \
    python3.9-venv \
    && rm -rf /var/lib/apt/lists/*

# Install pip for Python 3.9
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.9 \
    && python3.9 -m pip install --upgrade pip setuptools wheel

# ---------- 3. ROS Noetic user-space (Python 3.8 bindings) ----------
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | apt-key add - \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    python3-rospy \
    python3-rosnode \
    python3-rostopic \
    python3-rosservice \
    python3-rosparam \
    python3-rosgraph \
    python3-rosmsg \
    python3-rospkg \
    ros-noetic-rosnode \
    ros-noetic-rostopic \
    ros-noetic-rosservice \
    ros-noetic-rosparam \
    ros-noetic-rosgraph \
    ros-noetic-rosmsg \
    ros-noetic-rospack \
    && rm -rf /var/lib/apt/lists/*

# Make ROS Python packages visible to Python 3.9
# rospy, rosgraph, std_msgs, sensor_msgs, etc. are pure Python — safe to expose
ENV PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH}"

# Source ROS in bash sessions
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# ---------- 4. Build ainex_interfaces message bindings ----------
# ainex_interfaces contains custom .msg/.srv definitions used by the Ainex robot.
# We build them here so ainex_agent_tools can import them (read-only use only).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ros-noetic-catkin \
    python3-catkin-tools \
    python3-catkin-pkg \
    && rm -rf /var/lib/apt/lists/*

# Create a minimal catkin workspace for custom message/service packages
RUN mkdir -p /opt/ainex_msgs_ws/src

# Build context is docker/ (set in docker-compose.yml), so ros_ws_src is accessible
COPY ros_ws_src/ainex_interfaces /opt/ainex_msgs_ws/src/ainex_interfaces
COPY ros_ws_src/ainex_driver/ros_robot_controller /opt/ainex_msgs_ws/src/ros_robot_controller
COPY ros_ws_src/uuid_msgs /opt/ainex_msgs_ws/src/uuid_msgs
COPY ros_ws_src/py_trees_msgs /opt/ainex_msgs_ws/src/py_trees_msgs

# Build both packages (pure Python message/service bindings; no C++ hardware code).
RUN bash -c "\
    source /opt/ros/noetic/setup.bash && \
    cd /opt/ainex_msgs_ws && \
    catkin_make --pkg ainex_interfaces ros_robot_controller uuid_msgs py_trees_msgs \
      -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && echo 'ainex_interfaces + ros_robot_controller + py_trees_msgs build: OK'"

# Add generated Python bindings to PYTHONPATH
ENV PYTHONPATH="/opt/ainex_msgs_ws/devel/lib/python3/dist-packages:${PYTHONPATH}"

# ---------- 4c. ttyd web terminal (aarch64 prebuilt binary) ----------
# ttyd is not in Ubuntu 20.04 arm64 apt. Download from GitHub releases.
# Used to expose ainex_agent.py as a browser terminal on port 7681.
RUN curl -fsSL \
      "https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64" \
      -o /usr/local/bin/ttyd \
    && chmod +x /usr/local/bin/ttyd

# ---------- 5. Install ROSA from local vendor copy (preferred) or PyPI ----------
WORKDIR /opt/rosa-agent

# Copy requirements first (layer-cache friendly)
COPY rosa-agent/requirements.txt /opt/rosa-agent/requirements.txt
RUN python3.9 -m pip install --ignore-installed -r /opt/rosa-agent/requirements.txt

# Copy ROSA vendor source (patched local copy with max_execution_time + rostopic_echo fixes).
# Detect by rosa.py existing directly in vendor/rosa/ (flat layout, not pip-installable).
COPY rosa-agent/vendor/ /opt/rosa-agent/vendor/
RUN if [ -f /opt/rosa-agent/vendor/rosa/rosa.py ]; then \
        echo "Using local vendor ROSA (patched)"; \
    elif [ -d /opt/rosa-agent/vendor/rosa/src ]; then \
        python3.9 -m pip install -e /opt/rosa-agent/vendor/rosa; \
        echo "Installed ROSA from local vendor source (pip)"; \
    else \
        echo "Using ROSA installed from PyPI (requirements.txt)"; \
    fi

# Vendor rosa/ must be importable before PyPI rosa — prepend vendor/ to PYTHONPATH
ENV PYTHONPATH="/opt/rosa-agent/vendor:${PYTHONPATH}"

# ---------- 6. Copy agent workspace ----------
# Build context is docker/, so rosa-agent/ files are referenced with that prefix
COPY rosa-agent/ainex_agent_tools /opt/rosa-agent/ainex_agent_tools
COPY rosa-agent/config /opt/rosa-agent/config
COPY rosa-agent/ainex_agent.py /opt/rosa-agent/ainex_agent.py
COPY rosa-agent/llm_config.py /opt/rosa-agent/llm_config.py
COPY rosa-agent/summarize_ros_logs.py /opt/rosa-agent/summarize_ros_logs.py

# Install ainex_agent_tools as editable package
COPY rosa-agent/setup.py /opt/rosa-agent/setup.py
RUN python3.9 -m pip install -e /opt/rosa-agent

# ---------- 7. Environment ----------
ENV PYTHONPATH="/opt/rosa-agent:${PYTHONPATH}"
ENV ROS_MASTER_URI="http://127.0.0.1:11311"
# Read-only mode guard — set to "false" only after explicit audit
ENV AINEX_WRITE_ENABLED="false"

WORKDIR /opt/rosa-agent

# Entry point: run the ROSA agent CLI
CMD ["python3.9", "ainex_agent.py"]
