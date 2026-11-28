"""Bounded timestamp pairing, independent of ROS callback delivery order."""
from collections import deque


class ImageScanBuffer:
    def __init__(self, tolerance=.12, wait=.12):
        self.tolerance=tolerance;self.wait=wait
        self.scans=deque(maxlen=5);self.images=deque()

    def scan(self,stamp,message):
        self.scans.append((stamp,message))

    def image(self,stamp,message,received):
        self.images.append((stamp,message,received))

    def ready(self,now):
        result=[]
        while self.images:
            stamp,image,received=self.images[0]
            closest=min(self.scans,key=lambda s:abs(s[0]-stamp)) if self.scans else None
            matched=closest is not None and abs(closest[0]-stamp)<=self.tolerance
            if not matched and now-received<self.wait and len(self.images)<=5:break
            self.images.popleft()
            # Expired unmatched inputs are passed to normal validation,
            # which publishes an invalid result. No stale clear is invented.
            result.append((image,closest[1] if closest else None))
        return result
