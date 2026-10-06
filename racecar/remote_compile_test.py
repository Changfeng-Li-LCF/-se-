#!/usr/bin/env python3
"""
通过 SSH 在远程机器上编译和测试 rf2o_laser_odometry
使用 paramiko 库实现自动化
"""

import sys
import time

try:
    import paramiko
except ImportError:
    print("[错误] paramiko 未安装，正在尝试安装...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "paramiko"])
    import paramiko

HOST = "192.168.3.8"
PORT = 22
USERNAME = "bianbu"
PASSWORD = "1"
WORK_DIR = "~/zk/racecar"

def execute_ssh_command(ssh_client, command):
    """执行 SSH 命令并返回输出"""
    print(f"\n[执行] {command}")
    print("-" * 50)
    
    stdin, stdout, stderr = ssh_client.exec_command(command)
    
    out_text = ""
    err_text = ""
    
    # 实时读取输出
    for line in stdout:
        print(line.rstrip())
        out_text += line
    
    for line in stderr:
        if line.strip():
            print(f"[ERR] {line.rstrip()}")
            err_text += line
    
    return out_text, err_text

def main():
    print("=" * 60)
    print("远程编译和测试 rf2o_laser_odometry")
    print(f"目标: {USERNAME}@{HOST}:{WORK_DIR}")
    print("=" * 60)
    
    # 创建 SSH 连接
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"\n[连接] 正在连接到 {HOST}...")
        ssh.connect(HOST, port=PORT, username=USERNAME, password=PASSWORD, timeout=10)
        print("[成功] SSH 连接已建立")
        
        # 第一步：编译
        print("\n" + "=" * 60)
        print("[步骤 1] 编译 racecar 包")
        print("=" * 60)
        
        cmd = f"cd {WORK_DIR} && colcon build --packages-select racecar"
        execute_ssh_command(ssh, cmd)
        
        # 第二步：等待
        print("\n[等待] 5 秒后启动系统...")
        time.sleep(5)
        
        # 第三步：启动并测试
        print("\n" + "=" * 60)
        print("[步骤 2] 启动系统并检查节点")
        print("=" * 60)
        
        # 启动 car.sh（在后台）并等待节点启动
        cmd = f"""
cd {WORK_DIR}
bash car.sh > /tmp/car.log 2>&1 &
sleep 10
source install/setup.bash 2>/dev/null
echo "====== 节点列表 ======"
ros2 node list
echo ""
echo "====== 话题列表 ======"
ros2 topic list | grep -i odom
echo ""
echo "====== 检查 rf2o_laser_odometry_node ======"
ros2 node list | grep rf2o || echo "[未找到] rf2o_laser_odometry_node"
"""
        
        execute_ssh_command(ssh, cmd)
        
        print("\n" + "=" * 60)
        print("[完成] 测试结束")
        print("=" * 60)
        
    except paramiko.AuthenticationException:
        print("[错误] 身份验证失败，请检查用户名和密码")
        sys.exit(1)
    except paramiko.SSHException as e:
        print(f"[错误] SSH 连接失败: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"[错误] 未知错误: {e}")
        sys.exit(1)
    finally:
        ssh.close()
        print("\n[关闭] SSH 连接已关闭")

if __name__ == "__main__":
    main()
