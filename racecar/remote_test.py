#!/usr/bin/env python3
import subprocess
import sys
import time

HOST = "bianbu@192.168.3.8"
PASSWORD = "1"
WORK_DIR = "~/zk/racecar"

def run_ssh_command(cmd, timeout=120):
    """运行 SSH 命令并获取输出"""
    try:
        # 使用 sshpass 来自动输入密码
        full_cmd = f"sshpass -p {PASSWORD} ssh -o StrictHostKeyChecking=no {HOST} '{cmd}'"
        print(f"[执行] {full_cmd}")
        result = subprocess.run(full_cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired:
        print(f"[错误] 命令超时（{timeout}秒）")
        return "", "Timeout", -1
    except Exception as e:
        print(f"[错误] 执行失败: {e}")
        return "", str(e), -1

def main():
    print("=" * 50)
    print("远程编译和测试 rf2o_laser_odometry")
    print("=" * 50)
    
    # 检查 sshpass 是否可用
    result = subprocess.run("which sshpass", shell=True, capture_output=True)
    if result.returncode != 0:
        print("[警告] sshpass 未安装，尝试安装...")
        subprocess.run("sudo apt-get install -y sshpass", shell=True)
    
    # 第一步：编译
    print("\n[步骤 1] 编译 racecar 包...")
    cmd = f"cd {WORK_DIR} && colcon build --packages-select racecar 2>&1 | tail -20"
    stdout, stderr, code = run_ssh_command(cmd, timeout=180)
    if stdout:
        print(stdout)
    if stderr:
        print("[stderr]", stderr)
    
    # 第二步：等待并启动测试
    print("\n[步骤 2] 等待 5 秒后启动测试...")
    time.sleep(5)
    
    print("\n[步骤 3] 启动 car.sh 并检查节点...")
    # 启动 car.sh 并在后台运行，然后查询节点列表
    cmd = f"cd {WORK_DIR} && bash car.sh &\nsleep 8\nsource install/setup.bash 2>/dev/null && ros2 node list"
    stdout, stderr, code = run_ssh_command(cmd, timeout=30)
    if stdout:
        print("[输出]")
        print(stdout)
    if stderr:
        print("[stderr]", stderr)
    
    print("\n[完成] 测试结束")

if __name__ == "__main__":
    main()
