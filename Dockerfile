ARG WORKSPACE=/opt/ros/ur_ws
ARG FROM_IMAGE=ros:noetic
ARG IP=192.168.0.64
ARG BUILD_SHA

# Cache apt dependencies
FROM $FROM_IMAGE as apt-depends
LABEL "rhoban.project.name"="ur5-rhoban" \
"author"="Kohio Deflesselle"

RUN --mount=type=cache,target=/var/cache/apt \
DEBIAN_FRONTEND=noninteractive apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
git wget libpcl-dev libmodbus5 libpcap0.8 python3-pip python-is-python3 \
python3-dev libpython3-dev libboost-python-dev doxygen libjsoncpp-dev \
ros-${ROS_DISTRO}-tf2-tools \
ros-${ROS_DISTRO}-rqt \
ros-${ROS_DISTRO}-rqt-common-plugins \
ros-${ROS_DISTRO}-rqt-robot-plugins \
ros-${ROS_DISTRO}-image-transport-plugins

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


FROM apt-depends as placo-builder

WORKDIR /root/
RUN <<-EOF
apt install -qqy lsb-release curl
mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list

apt update
apt-get install -y robotpkg-hpp-fcl robotpkg-eiquadprog \
    robotpkg-pinocchio=2.7.0 robotpkg-py38-pinocchio=2.7.0 robotpkg-py38-eigenpy=3.5.0

export PATH=/opt/openrobots/bin:$PATH 
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

git clone https://gitlab.com/libeigen/eigen.git
cd eigen
git checkout 3.4
mkdir build 
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make install
cd ../..

git clone https://github.com/Soralsei/placo.git
cd placo
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 24 install

export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH

echo "export PATH=/opt/openrobots/bin:$PATH\n \
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH\n \
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH\n \
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH\n \
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH\n \
export PYTHONPATH=/usr/local/lib/python3.8/site-packages:$PYTHONPATH" >> ~/.bashrc

EOF

#Building stage
FROM placo-builder as rosdep-install
ARG WORKSPACE

WORKDIR ${WORKSPACE}

# Install all workspace packages dependencies
COPY --from=cacher /tmp/$WORKSPACE/src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
&& apt-get update \
&& rosdep update \
&& rosdep install -r -y --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO} \
&& apt-get upgrade -y


FROM rosdep-install as dev
ARG WORKSPACE
ENV WORKSPACE=${WORKSPACE}

WORKDIR ${WORKSPACE}

COPY . .
RUN <<-EOF
    . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make
    echo "source $WORKSPACE/devel/setup.bash" >> ~/.bashrc
    python3 -m pip install --upgrade pip && pip install --ignore-installed -r requirements.txt
EOF


FROM rosdep-install as release
ARG WORKSPACE

WORKDIR ${WORKSPACE}

# Copy project files
COPY src src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make

ENV WORKSPACE=$WORKSPACE
RUN sed --in-place --expression \
    '$isource "$WORKSPACE/devel/setup.bash"' \
    /ros_entrypoint.sh \
    && echo "source $WORKSPACE/devel/setup.bash" >> ~/.bashrc

COPY requirements.txt .
RUN python3 -m pip install --upgrade pip \
&& pip install --ignore-installed -r requirements.txt