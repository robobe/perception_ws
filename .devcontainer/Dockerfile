FROM humble:dev
# FROM docker.io/humble/cuda:dev

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get -y install --no-install-recommends \
      tmux \
      tmuxp \
      psmisc \
      ros-humble-image-view \
      ros-humble-cv-bridge \
      ros-humble-ros2bag \
      ros-humble-rosbag2-* \
      python3-colcon-clean \
      htop \
      #gst
      libgstreamer1.0-0 \
      gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly \
      gstreamer1.0-tools \
      python3-gi \
      gir1.2-gstreamer-1.0 \
      libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-libav && \
      apt-get autoremove -y && \
      apt-get clean -y && \
      rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

COPY .devcontainer/.tmux.conf /etc/tmux.conf

COPY .devcontainer/requirements.txt requirements.txt
RUN pip3 install -r requirements.txt
