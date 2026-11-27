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


def register_scan(current, previous, initial=None, max_translation=.25, diagnostics=None):
    """Robust 2D ICP: current laser frame -> previous laser frame."""
    info=diagnostics if diagnostics is not None else {}
    info.clear();info['input_points']=len(current)
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
    for iteration in range(30):
        moved=current@rotation.T+translation
        distances,indices=tree.query(moved)
        mask=distances<=min(.35,max(1e-6,float(np.quantile(distances,.9))))
        if mask.sum()<40:raise ValueError('insufficient_matches')
        a=moved[mask];b=previous[indices[mask]];n=normals[indices[mask]]
        residual=np.sum(n*(b-a),axis=1)
        design=np.column_stack((n[:,0],n[:,1],-n[:,0]*a[:,1]+n[:,1]*a[:,0]))
        # Distance gating alone admits slowly moving foreground returns.
        # Redescending point-to-plane weights keep that minority from pulling
        # the pose. Begin with a wide basin for genuine scan displacement;
        # a 3 cm cutoff from iteration zero would freeze forward corridor motion.
        scale=max(.03,.1*.6**max(0,iteration-2),
                  4.685*1.4826*float(np.median(np.abs(residual-np.median(residual)))))
        weights=np.ones(len(residual)) if iteration<2 else np.maximum(0.,1.-(residual/scale)**2)**2
        if iteration>=2:weights[np.abs(residual)>=scale]=0.
        weighted=design*np.sqrt(weights[:,None])
        if np.count_nonzero(weights)<40 or np.linalg.cond(weighted)>1000:
            raise ValueError('degenerate_scan_geometry')
        delta=np.linalg.lstsq(weighted,residual*np.sqrt(weights),rcond=None)[0]
        c=math.cos(delta[2]);sn=math.sin(delta[2]);r=np.array([[c,-sn],[sn,c]])
        rotation=r@rotation;translation=r@translation+delta[:2]
        if iteration>=6 and np.linalg.norm(delta)<1e-6:break
    moved=current@rotation.T+translation
    distances,indices=tree.query(moved)
    # Score the same bounded correspondences used by the fit. Sorting an
    # arbitrary 80% reintroduced moving-object returns >1 m away that the
    # optimizer had correctly excluded, rejecting an otherwise static pose.
    # Require broad support; a small matching fragment is not localization.
    mask=distances<=min(.35,max(1e-6,float(np.quantile(distances,.9))))
    info.update(supported_points=int(mask.sum()),support_fraction=float(mask.mean()))
    if mask.sum()<max(60,math.ceil(.7*len(current))):
        raise ValueError('insufficient_static_matches')
    a=moved[mask];n=normals[indices[mask]]
    design=np.column_stack((n[:,0],n[:,1],-n[:,0]*a[:,1]+n[:,1]*a[:,0]))
    if np.linalg.cond(design)>1000:raise ValueError('degenerate_scan_geometry')
    # Retain the original 20% trimming for angular sampling/occlusion edges,
    # but only INSIDE accepted correspondences. Do not reintroduce excluded
    # foreground points merely to fill a fixed percentage of the whole scan.
    quality=np.sort(distances[mask])
    rmse=float(np.sqrt(np.mean(quality[:int(.8*len(quality))]**2)))
    angle=math.atan2(rotation[1,0],rotation[0,0])
    info.update(rmse=rmse,translation_m=float(np.linalg.norm(translation)),rotation_rad=angle)
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
    """Keyframe ICP with a bounded, two-reference fallback on rejection.

    Initial guess comes only from the preceding scan estimate, never commands
    or simulator pose. Each fallback fit must pass the original quality gates,
    then agree in the odometry frame. Failure changes no pose/reference state.
    """
    def __init__(self,max_reference_age=2.):
        if not math.isfinite(max_reference_age) or max_reference_age<=0:
            raise ValueError('invalid_reference_age')
        self.reference=None;self.anchor=np.zeros(3);self.relative=np.zeros(3);self.pose=np.zeros(3)
        self.diagnostics={}
        self.max_reference_age=max_reference_age;self.reference_stamp=None;self.last_stamp=None
        self.recent=deque(maxlen=3)

    @staticmethod
    def rotation(yaw):
        c,s=math.cos(yaw),math.sin(yaw)
        return np.array([[c,-s],[s,c]])

    def recent_consensus(self,points,stamp):
        references=[r for r in self.recent if stamp is not None and
                    r[0] is not None and 0<stamp-r[0]<=.35]
        if len(references)<2:raise ValueError('recent_reference_unavailable')
        candidates=[];fits=[]
        for source,reference,anchor in list(reversed(references))[:2]:
            relative=self.rotation(anchor[2]).T@(self.pose[:2]-anchor[:2])
            guess=np.r_[relative,self.pose[2]-anchor[2]]
            quality={}
            delta,angle,rmse=register_scan(points,reference,guess,.25,quality)
            candidate=np.r_[anchor[:2]+self.rotation(anchor[2])@delta,anchor[2]+angle]
            candidates.append(candidate)
            fits.append(dict(stamp=source,age_s=stamp-source,**quality))
        position_gap=float(np.linalg.norm(candidates[0][:2]-candidates[1][:2]))
        yaw_gap=abs(math.atan2(math.sin(candidates[0][2]-candidates[1][2]),
                               math.cos(candidates[0][2]-candidates[1][2])))
        self.diagnostics.update(recent_fits=fits,consensus_position_gap_m=position_gap,
                                consensus_yaw_gap_rad=yaw_gap)
        if position_gap>.02 or yaw_gap>.01:
            raise ValueError('recent_reference_disagreement')
        candidate=candidates[0]
        if np.linalg.norm(candidate[:2]-self.pose[:2])>.25 or abs(candidate[2]-self.pose[2])>.2:
            raise ValueError('recent_reference_motion_limit')
        self.diagnostics.update(registration_mode='recent_consensus',
                                matched_reference_stamp=fits[0]['stamp'],
                                reference_age_s=fits[0]['age_s'])
        return candidate,max(fit['rmse'] for fit in fits)

    def remember(self,points,pose,stamp):
        self.recent.append((stamp,points.copy(),pose.copy()))

    def update(self,points,stamp=None):
        if stamp is not None and (not math.isfinite(stamp) or stamp<0 or
                                  self.last_stamp is not None and stamp<=self.last_stamp):
            raise ValueError('keyframe_timestamp')
        self.diagnostics.clear()
        if self.reference is None:
            if len(points)<60:raise ValueError('insufficient_scan')
            self.reference=points.copy();self.reference_stamp=stamp;self.last_stamp=stamp
            self.remember(points,self.pose,stamp)
            self.diagnostics.update(registration_mode='initial',active_reference_stamp=stamp)
            return self.pose.copy(),0.
        age=stamp-self.reference_stamp if stamp is not None and self.reference_stamp is not None else 0.
        self.diagnostics.update(registration_mode='keyframe',matched_reference_stamp=self.reference_stamp,
                                reference_age_s=age)
        recovered=False
        quality={}
        try:
            delta,angle,rmse=register_scan(points,self.reference,self.relative,.8,quality)
            pose=np.r_[self.anchor[:2]+self.rotation(self.anchor[2])@delta,self.anchor[2]+angle]
            relative=np.r_[delta,angle]
            self.diagnostics.update(quality)
        except (ValueError,np.linalg.LinAlgError) as error:
            self.diagnostics.update(primary_failure=str(error),primary_quality=quality,
                                    primary_reference_age_s=age)
            # Both recent observations are independently registered and must
            # still satisfy >=60 / >=70%, geometry and 0.08 m residual checks.
            # A stale/missing second reference cannot silently become a
            # single-frame estimate. No rejected scan is remembered.
            pose,rmse=self.recent_consensus(points,stamp)
            recovered=True
            relative=np.zeros(3)
        # A parked vehicle must not keep a foreground occluder in its
        # reference indefinitely. Reanchor only AFTER a valid registration;
        # retain the estimated pose, never assert zero velocity or use truth.
        self.pose=pose;self.relative=relative
        if recovered or np.linalg.norm(relative[:2])>=.5 or abs(relative[2])>=.15 or age>=self.max_reference_age:
            self.reference=points.copy();self.anchor=pose.copy();self.relative=np.zeros(3)
            self.reference_stamp=stamp
        self.last_stamp=stamp
        self.remember(points,pose,stamp)
        self.diagnostics['active_reference_stamp']=self.reference_stamp
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
