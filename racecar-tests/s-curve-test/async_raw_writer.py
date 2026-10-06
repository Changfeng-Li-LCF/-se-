"""Bounded separate-process ROS message serialization; never creates ROS nodes."""
import json
import math
import multiprocessing
import queue
import signal
import time


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return str(value)
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


def _write(path,incoming,written,errors,artificial_delay):
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    try:
        from rosidl_runtime_py.convert import message_to_ordereddict
        with open(path,'w',encoding='utf-8') as stream:
            last_flush=time.monotonic()
            while True:
                try:record=incoming.get(timeout=.2)
                except queue.Empty:
                    stream.flush();continue
                if record is None:break
                if artificial_delay:time.sleep(artificial_delay)
                stamp,topic,msg=record
                value=msg if isinstance(msg,dict) else message_to_ordereddict(msg)
                stream.write(json.dumps(clean({'t':stamp,'topic':topic,'message':value}),ensure_ascii=False)+'\n')
                with written.get_lock():written.value+=1
                if time.monotonic()-last_flush>=.2:
                    stream.flush();last_flush=time.monotonic()
    except BaseException as exc:
        try:errors.put_nowait(repr(exc))
        except queue.Full:pass
        raise


class AsyncRawWriter:
    def __init__(self,path,capacity=128,artificial_delay=0):
        ctx=multiprocessing.get_context('spawn')
        self.queue=ctx.Queue(maxsize=capacity)
        self.errors=ctx.Queue(maxsize=1)
        self.written=ctx.Value('L',0)
        self.process=ctx.Process(target=_write,args=(str(path),self.queue,self.written,self.errors,artificial_delay),daemon=True)
        self.enqueued=0;self.dropped=0;self.closed=False
        self.process.start()

    def enqueue(self,t,topic,msg):
        if self.closed:raise RuntimeError('Raw writer is closed')
        try:self.queue.put_nowait((t,topic,msg))
        except queue.Full:self.dropped+=1;return False
        self.enqueued+=1;return True

    def check(self):
        if not self.process.is_alive():
            try:detail=self.errors.get_nowait()
            except queue.Empty:detail='exit_code='+str(self.process.exitcode)
            raise RuntimeError('Raw log writer exited: '+detail)

    def close(self,timeout=4.):
        if self.closed:return self.result
        self.closed=True;deadline=time.monotonic()+timeout
        try:self.queue.put(None,timeout=max(.01,timeout/2))
        except queue.Full:pass
        self.process.join(timeout=max(0.,deadline-time.monotonic()))
        forced=self.process.is_alive()
        if forced:self.process.terminate();self.process.join(timeout=2.)
        try:error=self.errors.get_nowait()
        except queue.Empty:error=None
        self.result=dict(enqueued=self.enqueued,written=self.written.value,dropped=self.dropped,
                         incomplete=forced or self.written.value!=self.enqueued,
                         exit_code=self.process.exitcode,error=error)
        # A killed worker must not make the parent wait forever on a feeder pipe.
        self.queue.cancel_join_thread();self.queue.close();self.errors.close()
        return self.result
