#!/usr/bin/env bash
# 通过 SSH 在远程编译和测试 rf2o_laser_odometry

echo "========================================"
echo "正在远程编译 racecar 包..."
echo "========================================"

# 使用 expect 来自动输入密码
expect << 'EXPECT_EOF'
set timeout 120
spawn ssh -t bianbu@192.168.3.8 "cd ~/zk/racecar && colcon build --packages-select racecar 2>&1 | grep -E '(Summary|rf2o|error|ERROR)'"
expect "password:"
send "1\r"
expect eof
EXPECT_EOF

echo ""
echo "========================================"
echo "编译完成，3 秒后启动测试..."
echo "========================================"
sleep 3

echo ""
echo "========================================"
echo "启动 car.sh 并检查节点列表..."
echo "========================================"

# 启动车辆系统并检查节点
expect << 'EXPECT_EOF'
set timeout 15
spawn ssh -t bianbu@192.168.3.8 "cd ~/zk/racecar && timeout 10 bash car.sh 2>&1 | tail -20 & sleep 3 && ros2 node list 2>&1"
expect "password:"
send "1\r"
expect eof
EXPECT_EOF

echo ""
echo "========================================"
echo "完成！"
echo "========================================"
