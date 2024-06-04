ARG WORKSPACE=/opt/ros/ur5e_ws
ARG FROM_IMAGE=ros:noetic
ARG IP=192.168.0.64
ARG BUILD_SHA

# Cache apt dependencies
FROM $FROM_IMAGE as apt-depends
LABEL "rhoban.project.name"="ur5-rhoban" \
"author"="Kohio Deflesselle"

RUN --mount=type=cache,target=/var/cache/apt \
DEBIAN_FRONTEND=noninteractive apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
wget libpcl-dev libmodbus5 libpcap0.8 \
ros-${ROS_DISTRO}-tf2-tools \
ros-${ROS_DISTRO}-rqt \
ros-${ROS_DISTRO}-rqt-common-plugins \
ros-${ROS_DISTRO}-rqt-robot-plugins \
ros-${ROS_DISTRO}-image-transport-plugins \
&& rm -rf /var/lib/apt/lists/*

# Caching stage
FROM $FROM_IMAGE AS cacher
ARG WORKSPACE

WORKDIR $WORKSPACE/src
COPY src .

# Separate package.xml files in /tmp directory
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
find . -name "package.xml" | \
xargs cp --parents -t /tmp/opt && \
find . -name "CATKIN_IGNORE" | \
xargs cp --parents -t /tmp/opt || true


#Building stage
FROM apt-depends as builder
ARG WORKSPACE

WORKDIR ${WORKSPACE}

# Install all workspace packages dependencies
COPY --from=cacher /tmp/$WORKSPACE/src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
&& apt-get update \
&& rosdep update \
&& rosdep install -r -y --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO} \
&& apt-get upgrade -y \
&& rm -rf /var/lib/apt/lists/*

# Copy project files
COPY src src

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make

ENV WORKSPACE=$WORKSPACE
RUN sed --in-place --expression \
    '$isource "$WORKSPACE/devel/setup.bash"' \
    /ros_entrypoint.sh \
    && echo "source ${WORKSPACE}/devel/setup.bash" >> ~/.bashrc

# COPY Dockerfile .

# ARG BUILD_SHA
# LABEL "rhoban.rr100.build.sha"=${BUILD_SHA}

# FROM builder as simulation
# ENV ROS_MASTER_URI=http://localhost:11311

# # Do not remove, used to identify build target
# LABEL target="simulation"

# FROM builder as real
# ARG IP
# ENV ROS_MASTER_URI=http://rr-100-07:11311
# ENV ROS_IP=${IP}

# # Do not remove, used to identify build target
# LABEL target="real"
