source /etc/bash.bashrc
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${RACECAR_ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
python3 /mnt/d/RacecarWork/tools/configure-fastdds.py
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/d/RacecarWork/tools/fastdds-runtime.xml
