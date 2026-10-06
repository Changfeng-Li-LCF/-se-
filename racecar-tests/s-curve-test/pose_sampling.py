"""Source-stamped localization sampling, with no ROS or hardware dependencies."""
from collections import Counter
import math


class PoseSampler:
    def __init__(self, rate_hz=20.0, max_age_s=.5):
        self.period=1.0/rate_hz
        self.max_age_s=max_age_s
        self.rejected=Counter()
        self.last_received_stamp=None
        self.last_written=None
        self.last_fresh_receipt=None
        self.sample_count=0
        self.phase_stamp=None
        self.last_sample_bucket=None
        self.max_source_gap_s=self.max_receipt_gap_s=self.max_recorded_gap_s=0.
        self.clean_samples=0

    def offer(self, *, stamp, x, y, yaw, now, receipt):
        if not all(math.isfinite(v) for v in (stamp,x,y,yaw,now,receipt)):
            self.rejected['nonfinite_pose']+=1;return None
        age=now-stamp
        if not -.05<=age<=self.max_age_s:
            self.rejected['stale_or_future_pose']+=1;return None
        if self.last_received_stamp is not None and stamp<=self.last_received_stamp:
            self.rejected['duplicate_or_out_of_order_pose']+=1;return None
        if self.last_received_stamp is not None:
            self.max_source_gap_s=max(self.max_source_gap_s,stamp-self.last_received_stamp)
        if self.last_fresh_receipt is not None:
            self.max_receipt_gap_s=max(self.max_receipt_gap_s,receipt-self.last_fresh_receipt)
        self.last_received_stamp=stamp
        self.last_fresh_receipt=receipt
        if self.phase_stamp is None:self.phase_stamp=stamp
        # Fixed source-time buckets avoid resetting the period after each
        # sample: 49/51 ms jitter must not halve a 20 Hz source stream.
        bucket=math.floor((stamp-self.phase_stamp)/self.period+.5+1e-6)
        if self.last_sample_bucket is not None and bucket<=self.last_sample_bucket:
            self.rejected['pose_rate_limited']+=1;return None
        quality='ok'
        if self.last_written:
            dt=stamp-self.last_written['stamp']
            self.max_recorded_gap_s=max(self.max_recorded_gap_s,dt)
            if math.hypot(x-self.last_written['x'],y-self.last_written['y'])>.2+dt:
                quality='pose_jump';self.rejected[quality]+=1
        row=dict(t=receipt,stamp=stamp,x=x,y=y,yaw=yaw,tf_age_s=age,quality=quality)
        self.last_written=row
        self.last_sample_bucket=bucket
        self.sample_count+=1
        self.clean_samples=self.clean_samples+1 if quality=='ok' else 0
        return row

    @property
    def ready(self):
        return self.clean_samples>=2
