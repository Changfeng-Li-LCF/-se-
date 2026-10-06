#!/usr/bin/env python3
"""Integration checks with virtual serial, isolated DDS, temporary latch files only."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
import yaml

os.environ['ROS_DOMAIN_ID']='197'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
os.environ.pop('RMW_FASTRTPS_USE_QOS_FROM_XML',None)
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

BIN=Path(sys.argv[1]);CONFIG=Path(sys.argv[2])
calibration=yaml.safe_load(CONFIG.read_text())['racecar_driver']['ros__parameters']
BRAKE_PWM=int(calibration['brake_pwm'])
BRAKE_DURATION=float(calibration['brake_duration_s'])
assert 500 <= BRAKE_PWM < calibration['motor_neutral_pwm'] and 0 < BRAKE_DURATION <= .5
rclpy.init();node=rclpy.create_node('reverse_brake_virtual_serial_test')
results=[]


def test(executable,topic,enabled):
    with tempfile.TemporaryDirectory(prefix='brake-pty-test-') as tmp:
        master,slave=pty.openpty();device=os.ttyname(slave);assert device.startswith('/dev/pts/')
        latch=Path(tmp)/'latch';stream=(Path(tmp)/'driver.log').open('w')
        command=[str(BIN/executable),'--ros-args','--params-file',str(CONFIG),
                 '-p','serial_port:='+device,'-p','emergency_stop_state_file:='+str(latch),
                 '-p','reverse_brake_enabled:='+str(enabled).lower()]
        process=None;packets=[];buffer=bytearray()
        nav=node.create_publisher(Twist,topic,1);teleop=node.create_publisher(Twist,'/teleop_cmd_vel',1)
        stop_topic=node.create_publisher(Bool,'/emergency_stop',10)
        stop_client=node.create_client(Trigger,'/racecar_driver/emergency_stop')
        reset_client=node.create_client(Trigger,'/racecar_driver/reset_emergency_stop')
        def collect():
            if select.select([master],[],[],0)[0]:buffer.extend(os.read(master,65536))
            while len(buffer)>=7:
                frame=bytes(buffer[:7]);del buffer[:7]
                assert frame[0]==0xaa and frame[6]==0x55 and frame[5]==sum(frame[1:5])%256,frame
                motor=frame[1]+256*frame[2];servo=frame[3]+256*frame[4]
                packets.append((time.monotonic(),motor,servo))
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01);collect()
        def until(predicate,seconds=5):
            end=time.monotonic()+seconds
            while not predicate() and time.monotonic()<end:spin(.02)
            assert predicate(),('timeout',packets[-12:])
        def call(client):
            assert client.wait_for_service(timeout_sec=5)
            f=client.call_async(Trigger.Request());until(f.done);return f.result()
        def move():
            msg=Twist();msg.linear.x=.2;msg.angular.z=.1;nav.publish(msg)
        def start():
            nonlocal process
            packets.clear();buffer.clear()
            process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT)
            until(lambda:any(m==1500 for _,m,_ in packets),8)
            until(lambda:nav.get_subscription_count()>0)
            spin(.15);packets.clear()
        def stop():
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT);process.wait(timeout=5)
            collect()
        def drive():
            packets.clear()
            for i in range(30):
                move();spin(.05)
                if any(m==1550 for _,m,_ in packets):return
            raise AssertionError('No forward packet')
        try:
            start();drive();held_servo=packets[-1][2];packets.clear()
            assert call(stop_client).success
            first=time.monotonic()
            while time.monotonic()-first<.6:
                move();t=Twist();t.linear.x=1600.;t.angular.z=90.;teleop.publish(t)
                stop_topic.publish(Bool(data=True));spin(.03)
            assert packets and all(m<=1500 for _,m,_ in packets),packets
            assert all(s==held_servo for _,_,s in packets),packets
            reverse=[t for t,m,s in packets if m==BRAKE_PWM]
            if enabled:
                assert reverse,'Missing brake pulse';assert max(reverse)-min(reverse)<BRAKE_DURATION+.1,reverse
                neutral_after=[t for t,m,s in packets if m==1500 and t>reverse[0]]
                assert neutral_after,'Did not return to neutral'
                pulse_observed=neutral_after[0]-reverse[0]
                assert max(0,BRAKE_DURATION-.04)<=pulse_observed<=BRAKE_DURATION+.12,pulse_observed
                assert packets[-1][1]==1500
            else:assert not reverse and all(m==1500 for _,m,_ in packets)
            stop_topic.publish(Bool(data=False));move();spin(.04)
            assert not call(reset_client).success,'Reset accepted while commands were active'
            assert latch.exists();spin(.65)
            assert call(reset_client).success;assert not latch.exists()
            drive();assert call(stop_client).success;spin(.25);stop()
            # Persistence: startup + renewed forward commands must remain neutral, no repeated pulse.
            start()
            for i in range(10):move();spin(.04)
            assert packets and all(m==1500 for _,m,_ in packets),packets
            spin(.6);assert call(reset_client).success
            drive();packets.clear();spin(.95)
            if enabled:
                assert any(m==BRAKE_PWM for _,m,_ in packets),'Watchdog did not start brake pulse'
                assert latch.exists() and packets[-1][1]==1500
                spin(.1);assert call(reset_client).success
                drive();packets.clear();nav.publish(Twist());spin(.35)
                assert any(m==BRAKE_PWM for _,m,_ in packets),'Zero command did not brake'
                assert packets[-1][1]==1500 and latch.exists()
            else:assert all(m>=1500 for _,m,_ in packets)
            results.append({'entry':executable,'reverse_enabled':enabled,'virtual_serial_only':True,
                            'status':'passed','test_brake_pwm':BRAKE_PWM,'test_duration_s':BRAKE_DURATION,
                            'observed_first_pulse_s':pulse_observed if enabled else None})
            print(json.dumps(results[-1]),flush=True)
        finally:
            stop();stream.close();os.close(master);os.close(slave)
            for pub in (nav,teleop,stop_topic):node.destroy_publisher(pub)
            node.destroy_client(stop_client);node.destroy_client(reset_client)
            rclpy.spin_once(node,timeout_sec=.2)

try:
    test('racecar_driver_node','/car_cmd_vel',True)
    test('racecar_driver_node_one','/cmd_vel',True)
    test('racecar_driver_node','/car_cmd_vel',False)
    print(json.dumps({'all_passed':True,'tests':results},indent=2))
finally:
    node.destroy_node();rclpy.shutdown()
