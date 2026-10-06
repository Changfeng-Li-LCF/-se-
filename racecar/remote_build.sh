#!/bin/bash
# 远程编译脚本

HOST="bianbu@192.168.3.8"
PASS="1"

echo "=== 正在连接到远程机器并编译 ==="

ssh -o StrictHostKeyChecking=no -o BatchMode=no "$HOST" << 'EOF'
cd ~/zk/racecar
echo "[INFO] 开始编译 racecar 包..."
colcon build --packages-select racecar 2>&1 | tail -50
echo "[INFO] 编译完成，等待 3 秒后测试启动..."
sleep 3
EOF

echo "=== 编译完成 ==="
