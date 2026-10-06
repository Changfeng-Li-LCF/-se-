FROM harbor.spacemit.com/bianbu/bianbu:latest

ENV DEBIAN_FRONTEND=noninteractive

# 添加 ROS 2 软件源并指定密钥
RUN sed -i '/^Suites:/s/$/ noble-ros/' /etc/apt/sources.list.d/bianbu.sources && \
    echo "Signed-By: /usr/share/keyrings/bianbu-archive-keyring.gpg" >> /etc/apt/sources.list.d/bianbu.sources

# 创建优先级配置文件（使用 echo 代替 heredoc，更稳定）
RUN echo "Package: src:opencv" > /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin: release o=Spacemit, n=noble-ros" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin-Priority: 50" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Package: src:qtbase-opensource-src" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin: release o=Spacemit, n=noble-ros" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin-Priority: 50" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Package: src:qtbase-opensource-src-gles" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin: release o=Spacemit, n=noble-ros" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin-Priority: 50" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Package: src:pyqt5" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin: release o=Spacemit, n=noble-ros" >> /etc/apt/preferences.d/noble-ros.pref && \
    echo "Pin-Priority: 50" >> /etc/apt/preferences.d/noble-ros.pref

# 更新并安装 ROS 2
RUN apt update && apt upgrade -y && \
    apt install -y \
        ros-humble-desktop \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        python3-rosdep \
        vim nano iputils-ping net-tools \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspace/racecar
CMD ["/bin/bash"]
