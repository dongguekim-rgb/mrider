"""Fail-closed camera diagnostics, independent of the segmentation backend."""
import math


def camera_fault(sample, now, wall, timeout=.35):
    if sample is None:return 'camera_missing'
    data,received=sample
    if not isinstance(data,dict):return 'camera_invalid_status'
    stamp=data.get('stamp')
    if isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp<=0:
        return 'camera_invalid_stamp'
    if not 0<=wall-received<=timeout:return 'camera_status_timeout'
    if now-stamp<-.05:return 'camera_future_stamp'
    if now-stamp>timeout:return 'camera_input_stale'
    if data.get('valid') is not True:return 'camera_invalid_input'
    if data.get('clear') is not True:
        return 'camera_unknown' if data.get('reason')=='unknown_region' else 'camera_obstacle'
    return None
