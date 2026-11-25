"""Classical scan registration and calibrated floor-colour segmentation; no truth input."""
import math
from collections import deque
import numpy as np
from scipy.spatial import cKDTree


def slam_ranges(ranges, range_min, range_max, max_range_is_clear=False):
    """No-hit rays are not surfaces. Adapt only the SLAM copy, never /scan.

    Karto's correlation lookup uses unfiltered points, even if its range
    threshold is smaller than range_max. NaN is explicitly excluded there.
    Unknown/invalid measurements are also excluded, not marked as free.
    """
    if not (math.isfinite(range_min) and math.isfinite(range_max)
            and 0 < range_min < range_max):
        raise ValueError('scan_range_limits')
    result=np.asarray(ranges,dtype=float).copy()
    invalid=~np.isfinite(result)|(result<range_min)|(result>range_max)
    if max_range_is_clear:
        invalid|=np.isclose(result,range_max,rtol=0.,atol=1e-6)
    result[invalid]=np.nan
    return result


def register_scan(current, previous, initial=None, max_translation=.25):
    """Robust 2D ICP: current laser frame -> previous laser frame."""
    if min(len(current),len(previous))<60:raise ValueError('insufficient_scan')
    tree=cKDTree(previous);rotation=np.eye(2);translation=np.zeros(2)
    if initial is not None:
        translation=np.array(initial[:2],dtype=float);c=math.cos(initial[2]);s=math.sin(initial[2])
        rotation=np.array([[c,-s],[s,c]])
    # Local surface normals constrain motion along walls; nearest-point ICP
    # alone aliases densely sampled corridor walls and underestimates travel.
    _,neighbors=tree.query(previous,k=7)
    patches=previous[neighbors];centered=patches-patches.mean(axis=1,keepdims=True)
    covariance=np.einsum('nki,nkj->nij',centered,centered)
    _,vectors=np.linalg.eigh(covariance);normals=vectors[:,:,0]
    for _ in range(30):
        moved=current@rotation.T+translation
        distances,indices=tree.query(moved)
        mask=distances<=min(.35,max(1e-6,float(np.quantile(distances,.9))))
        if mask.sum()<40:raise ValueError('insufficient_matches')
        a=moved[mask];b=previous[indices[mask]];n=normals[indices[mask]]
        residual=np.sum(n*(b-a),axis=1)
        design=np.column_stack((n[:,0],n[:,1],-n[:,0]*a[:,1]+n[:,1]*a[:,0]))
        if np.linalg.cond(design)>1000:raise ValueError('degenerate_scan_geometry')
        delta=np.linalg.lstsq(design,residual,rcond=None)[0]
        c=math.cos(delta[2]);sn=math.sin(delta[2]);r=np.array([[c,-sn],[sn,c]])
        rotation=r@rotation;translation=r@translation+delta[:2]
        if np.linalg.norm(delta)<1e-6:break
    distances,_=tree.query(current@rotation.T+translation)
    rmse=float(np.sqrt(np.mean(np.sort(distances)[:int(.8*len(distances))]**2)))
    angle=math.atan2(rotation[1,0],rotation[0,0])
    if rmse>.08 or np.linalg.norm(translation)>max_translation or abs(angle)>.2:raise ValueError('scan_registration_failed')
    return translation,angle,rmse


def floor_clear_fraction(rgb):
    """Fixed simulation floor appearance, not a general-purpose road detector."""
    if rgb.ndim!=3 or rgb.shape[2]!=3 or min(rgb.shape[:2])<40:raise ValueError('invalid_image')
    h,w=rgb.shape[:2];roi=rgb[int(h*.7):int(h*.95),int(w*.35):int(w*.65)].astype(float)
    # Gray floor with bounded brightness; no trained weights or distance inference.
    floor=(roi.max(axis=2)-roi.min(axis=2)<25)&(roi.mean(axis=2)>75)&(roi.mean(axis=2)<155)
    return float(floor.mean())


class KeyframeOdometry:
    """Scan-to-keyframe ICP. Reference resets after 0.5 m or 0.15 rad.

    Initial guess comes only from the preceding scan estimate, never commands
    or simulator pose. Failure leaves the reference and estimate unchanged.
    """
    def __init__(self):
        self.reference=None;self.anchor=np.zeros(3);self.relative=np.zeros(3);self.pose=np.zeros(3)

    def update(self,points):
        if self.reference is None:
            if len(points)<60:raise ValueError('insufficient_scan')
            self.reference=points.copy();return self.pose.copy(),0.
        delta,angle,rmse=register_scan(points,self.reference,self.relative,.8)
        c=math.cos(self.anchor[2]);s=math.sin(self.anchor[2])
        pose=np.array([*(self.anchor[:2]+np.array([[c,-s],[s,c]])@delta),self.anchor[2]+angle])
        self.pose=pose;self.relative=np.array([*delta,angle])
        if np.linalg.norm(delta)>=.5 or abs(angle)>=.15:
            self.reference=points.copy();self.anchor=pose.copy();self.relative=np.zeros(3)
        return pose.copy(),rmse


class VelocityWindow:
    """Pose-derived velocity over at most four scans; no command feedback.

    At 10 Hz this averages 0.3 s, suppressing stationary ICP jitter that would
    otherwise continually reset the stopped-vehicle clearance dwell. Pose and
    source timestamps remain current; deceleration is conservatively delayed.
    """
    def __init__(self):
        self.samples=deque(maxlen=4)

    def update(self,pose,stamp):
        if not math.isfinite(stamp) or (self.samples and stamp<=self.samples[-1][0]):
            raise ValueError('velocity_timestamp')
        self.samples.append((stamp,np.array(pose,dtype=float).copy()))
        if len(self.samples)<2:return 0.,0.
        before,old=self.samples[0];dt=stamp-before;dx,dy=pose[:2]-old[:2]
        heading=(old[2]+pose[2])/2
        return float((dx*math.cos(heading)+dy*math.sin(heading))/dt),float((pose[2]-old[2])/dt)
