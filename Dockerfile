FROM roslab/roslab:kinetic-nvidia

USER root

RUN apt-get update \
 && apt-get install -yq --no-install-recommends \
    ros-kinetic-cv-bridge \
    ros-kinetic-tf \
    ros-kinetic-message-filters \
    ros-kinetic-image-transport \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install -yq --no-install-recommends \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ceres-solver/ceres-solver.git /ceres-solver \
 && cd /ceres-solver \
 && mkdir build \
 && cd build \
 && cmake ../ \
 && make -j4 install \
 && rm -fr /ceres-solver

RUN mkdir -p ${HOME}/catkin_ws/src/vins-mono
COPY . ${HOME}/catkin_ws/src/vins-mono/.
RUN cd ${HOME}/catkin_ws \
 && mv src/vins-mono/README.ipynb .. \
 && apt-get update \
 && /bin/bash -c "source /opt/ros/kinetic/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc

RUN chown -R ${NB_UID} ${HOME}

USER ${NB_USER}
WORKDIR ${HOME}
