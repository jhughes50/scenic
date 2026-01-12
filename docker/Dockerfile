FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04 

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 vim \
 tmux \
 cmake \
 gcc \
 g++ \
 git \
 build-essential \
 sudo \
 wget \
 curl \
 zip \
 unzip \
 python3-venv \
 python3-pip

# install libtorch 
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.0-1_all.deb \
 && dpkg -i cuda-keyring_1.0-1_all.deb \
 && apt-get update \
 && apt-get -y install cuda
RUN wget https://download.pytorch.org/libtorch/cu128/libtorch-shared-with-deps-2.9.1%2Bcu128.zip\
 && unzip *.zip \
 && mv libtorch/ /opt \
 && rm *.zip

RUN userdel ubuntu
ARG user_id=1000
ENV USER jason
RUN useradd -U --uid ${user_id} -ms /bin/bash $USER \
 && echo "$USER:$USER" | chpasswd \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER
WORKDIR /home/$USER
USER $USER

RUN sudo apt-get update \
 && sudo apt-get install -y --no-install-recommends \
 libgoogle-glog-dev \
 libopencv-dev \
 libtbb-dev \
 libjsoncpp-dev \
 libyaml-cpp-dev \
 libgtest-dev

RUN python3 -m venv lang \
 && lang/bin/pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu126 \
 && lang/bin/pip3 install scikit-image scikit-learn opencv-python-headless \ 
 && lang/bin/pip3 install transformers torch-kmeans

RUN lang/bin/pip3 install matplotlib

RUN sudo apt install -y ipython3

# install ROS
USER root
RUN apt-get update \
  && export TZ="America/New_York" \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y locales \
  && ln -fs "/usr/share/zoneinfo/$TZ" /etc/localtime \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && apt-get clean

RUN sudo apt install -y software-properties-common \
  && sudo add-apt-repository universe

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')\
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
  && sudo apt install /tmp/ros2-apt-source.deb \
  && sudo apt update \
  && sudo apt install -y ros-dev-tools

RUN sudo apt update \
  && sudo apt install -y ros-jazzy-desktop-full

RUN sudo apt install -y ros-jazzy-gtsam
USER $USER

RUN mkdir -p ws/src

ENV CUDA_HOME=/usr/local/cuda
RUN sudo chown $USER:$USER .bashrc \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc
