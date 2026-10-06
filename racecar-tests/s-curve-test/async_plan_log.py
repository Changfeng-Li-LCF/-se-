"""Deserialize and record bulky ROS paths outside the navigation session loop.

The session receives serialized bytes only. Geometry and history are identical
to NavigationPlanLog; the worker has no ROS node and cannot publish commands.
"""
import json
import multiprocessing
from pathlib import Path
import queue
import threading
import time


def _worker(inbox, latest, stop, output):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Path as RosPath
    from navigation_plan_log import NavigationPlanLog
    import math
    health={'received':0,'written':0,'failed':0,'error':None}
    pending=None
    logger=NavigationPlanLog(Path(output))
    try:
        while True:
            try:item=inbox.get(timeout=.05)
            except queue.Empty:
                if stop.is_set():break
                item=None
            if item is not None:
                payload,received=item;health['received']+=1
                try:
                    msg=deserialize_message(payload,RosPath)
                    poses=[]
                    for p in msg.poses:
                        q=p.pose.orientation
                        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                        poses.append((p.pose.position.x,p.pose.position.y,yaw))
                    entry=logger.capture(msg.header.frame_id,poses,
                        msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,received)
                    health['written']+=1
                    pending=(entry,dict(health))
                except Exception as exc:
                    health['failed']+=1;health['error']=str(exc)
                    pending=(logger.latest,dict(health))
            if pending is not None:
                # Only the diagnostic snapshot may be coalesced. Every path
                # accepted into inbox is still written to the history file.
                try:latest.put_nowait(pending);pending=None
                except queue.Full:pass
    finally:
        try:Path(output).with_name('plan_log_health.json').write_text(json.dumps(health))
        except OSError:pass
        # Never wait for the parent to read a diagnostic snapshot on exit.
        latest.cancel_join_thread()


class AsyncPlanLog:
    def __init__(self,path):
        ctx=multiprocessing.get_context('spawn')
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.inbox=ctx.Queue(maxsize=32);self.output=ctx.Queue(maxsize=1);self.stop=ctx.Event()
        self.process=ctx.Process(target=_worker,args=(self.inbox,self.output,self.stop,str(self.path)),daemon=True)
        self.process.start()
        self.latest=None;self.sequence=0;self.enqueued=0;self.dropped=0;self.closed=False
        self.health={'received':0,'written':0,'failed':0,'error':None}
        self._received_snapshot=(None,dict(self.health))
        self._receiver_stop=threading.Event()
        self._receiver=threading.Thread(target=self._receive_snapshots,
            name='plan-snapshot-receiver',daemon=True)
        self._receiver.start()

    def capture_serialized(self,payload,received):
        if self.closed or not self.process.is_alive():self.dropped+=1;return False
        try:self.inbox.put_nowait((payload,received));self.enqueued+=1;return True
        except queue.Full:self.dropped+=1;return False

    def _receive_snapshots(self):
        while not self._receiver_stop.is_set():
            try:
                # Queue.get_nowait can still wait for the remainder of a large
                # pipe message. Receive/unpickle off the session main thread.
                snapshot=self.output.get(timeout=.05)
            except queue.Empty:continue
            except (EOFError,OSError,ValueError):return
            self._received_snapshot=snapshot

    def refresh(self):
        # Atomic reference read only: no pipe, deserialization, or waiting.
        entry,health=self._received_snapshot
        self.health=health
        if entry is not None and entry['sequence']>=self.sequence:
            self.latest=entry;self.sequence=entry['sequence']

    def snapshot(self):
        return dict(self.health,enqueued=self.enqueued,dropped=self.dropped,
            worker_alive=self.process.is_alive(),closed=self.closed,
            incomplete=bool(self.dropped or self.health['failed'] or self.health['error'] or
                (self.closed and self.health['received']!=self.enqueued)))

    def close(self,timeout_s=3.):
        if not self.closed:
            self.closed=True;self.stop.set();self.process.join(timeout_s)
            if self.process.is_alive():
                self.process.terminate();self.process.join(1.)
                self.health['error']='Path logger exceeded bounded shutdown'
            else:
                try:self.health=json.loads(self.path.with_name('plan_log_health.json').read_text())
                except (OSError,ValueError):pass
            self._receiver_stop.set();self._receiver.join(timeout=.2)
            self.inbox.cancel_join_thread();self.output.cancel_join_thread()
            self.inbox.close();self.output.close()
        return self.snapshot()
