"""Classical marker watershed with calibrated ground projection and 2D scan support.

Labels are geometric hypotheses, not learned semantic identities:
0 unknown, 1 floor candidate, 2 obstacle candidate, 3 wall/long-surface candidate.
No map, pose, scene IDs, or ground-truth segmentation is accepted.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass(frozen=True)
class CameraGeometry:
    fx: float
    fy: float
    cx: float
    cy: float
    height: float
    pitch: float=0.
    x: float=0.
    y: float=0.
    laser_height: float=.2

    def validate(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()):raise ValueError('invalid_calibration')
        if min(self.fx,self.fy,self.height)<=0 or abs(self.pitch)>.6:raise ValueError('invalid_calibration')


def segment(rgb,ranges,angle_min,angle_increment,range_min,range_max,camera,max_range_is_clear=False):
    camera.validate()
    if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or min(rgb.shape[:2])<40:raise ValueError('invalid_image')
    if not all(math.isfinite(v) for v in [angle_min,angle_increment,range_min,range_max]) or angle_increment<=0 or not 0<range_min<range_max:raise ValueError('invalid_scan')
    if rgb.shape[1]>160:
        from dataclasses import replace
        h0,w0=rgb.shape[:2];w=160;h=round(h0*w/w0);sx=w/w0;sy=h/h0
        small=cv2.resize(rgb,(w,h),interpolation=cv2.INTER_AREA)
        scaled=replace(camera,fx=camera.fx*sx,fy=camera.fy*sy,cx=camera.cx*sx,cy=camera.cy*sy)
        labels,_,result=segment(small,ranges,angle_min,angle_increment,range_min,range_max,scaled,max_range_is_clear)
        labels=cv2.resize(labels,(w0,h0),interpolation=cv2.INTER_NEAREST)
        palette=np.array([[80,80,80],[30,190,80],[230,65,40],[65,120,235]],dtype=np.uint8)
        return labels,cv2.addWeighted(rgb,.55,palette[labels],.45,0),result
    ranges=np.asarray(ranges,dtype=float)
    if ranges.ndim!=1 or len(ranges)<180 or (len(ranges)-1)*angle_increment<6.:raise ValueError('scan_coverage')
    # Only an explicitly verified scanner contract may treat a finite max-range
    # ray as observed clear. NaN/inf/zero/out-of-range never provide evidence.
    valid=np.isfinite(ranges)&(ranges>=range_min)&(ranges<range_max-.02)
    observed=valid | (bool(max_range_is_clear)&np.isfinite(ranges)&(np.abs(ranges-range_max)<1e-6))
    h,w=rgb.shape[:2];vv,uu=np.indices((h,w))
    right=(uu-camera.cx)/camera.fx;down=(vv-camera.cy)/camera.fy
    c,s=math.cos(camera.pitch),math.sin(camera.pitch)
    rx=c-s*down;ry=-right;rz=-s-c*down
    t=np.divide(-camera.height,rz,out=np.full_like(rz,np.nan),where=rz<-.01)
    gx=camera.x+t*rx;gy=camera.y+t*ry
    bearing=np.arctan2(gy,gx)
    index=np.rint(np.nan_to_num((bearing-angle_min)/angle_increment,nan=-999)).astype(int)
    in_scan=(index>=0)&(index<len(ranges));index=np.clip(index,0,len(ranges)-1)
    ground_distance=np.hypot(gx,gy)
    with np.errstate(invalid='ignore'):
        support=in_scan&observed[index]&np.isfinite(t)&(t>0)&(gx>.15)&(gx<8.)&(ground_distance<ranges[index]-.15)
    # Image evidence: estimate floor appearance from a broad, geometrically
    # supported near-field strip, not an absolute gray/brightness threshold.
    seed_area=support&(vv>h*.85)&(vv<h*.96)&(uu>w*.12)&(uu<w*.88)
    lab=cv2.cvtColor(rgb,cv2.COLOR_RGB2LAB).astype(float)
    if seed_area.sum()<w:raise ValueError('insufficient_floor_seed')
    reference=np.median(lab[seed_area],axis=0)
    distance=np.linalg.norm((lab-reference)*np.array([.35,1.,1.]),axis=2)
    appearance=distance<22.
    floor_seed=seed_area&appearance
    if floor_seed.sum()<w:raise ValueError('ambiguous_floor_seed')
    markers=np.zeros((h,w),np.int32);markers[floor_seed]=1
    # Upper border seeds unknown; laser-surface seeds supply non-floor evidence.
    markers[:3,:]=4
    angles=angle_min+np.arange(len(ranges))*angle_increment
    px=ranges*np.cos(angles);py=ranges*np.sin(angles)
    wall=np.zeros(len(ranges),bool)
    # Batched local PCA avoids hundreds of Python/SVD calls per image.
    windows=np.arange(9)[None,:]+np.arange(len(ranges)-8)[:,None]
    points=np.stack((px[windows],py[windows]),axis=2)
    points=np.nan_to_num(points,nan=0.,posinf=0.,neginf=0.)
    centered=points-points.mean(axis=1,keepdims=True)
    covariance=np.einsum('nki,nkj->nij',centered,centered)
    _,vectors=np.linalg.eigh(covariance)
    residual=np.max(np.abs(np.einsum('nki,ni->nk',centered,vectors[:,:,0])),axis=1)
    length=np.ptp(np.einsum('nki,ni->nk',centered,vectors[:,:,1]),axis=1)
    gaps=np.max(np.linalg.norm(np.diff(points,axis=1),axis=2),axis=1)
    wall[4:-4]=valid[windows].all(axis=1)&(gaps<.5)&(residual<.04)&(length>.6)
    for i in np.flatnonzero(valid):
        # Surface intersection at known scan height 0.20 m. Only a local seed;
        # do not invent the entire wall height from a single 2D scan.
        bx,by,bz=px[i]-camera.x,py[i]-camera.y,camera.laser_height-camera.height
        depth=c*bx-s*bz;image_down=-s*bx-c*bz
        if depth<=.1:continue
        u=int(round(camera.cx-camera.fx*by/depth));v=int(round(camera.cy+camera.fy*image_down/depth))
        if 2<=u<w-2 and 2<=v<h-2:
            cv2.circle(markers,(u,v),2,3 if wall[i] else 2,-1)
    segmented=cv2.watershed(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),markers)
    labels=np.zeros((h,w),np.uint8)
    labels[(segmented==1)&support&appearance]=1
    labels[segmented==2]=2;labels[segmented==3]=3
    # Keep regions outside any geometric viewing evidence unknown.
    labels[vv<camera.cy*.5]=0
    roi=(vv>=h*.7)&(vv<h*.95)&(uu>=w*.35)&(uu<w*.65)
    floor_fraction=float(np.mean(labels[roi]==1))
    blocked_fraction=float(np.mean((labels[roi]==2)|(labels[roi]==3)))
    unknown_fraction=float(np.mean(labels[roi]==0))
    palette=np.array([[80,80,80],[30,190,80],[230,65,40],[65,120,235]],dtype=np.uint8)
    overlay=cv2.addWeighted(rgb,.55,palette[labels],.45,0)
    reason='clear' if floor_fraction>.65 else 'obstacle_region' if blocked_fraction>.2 else 'unknown_region'
    return labels,overlay,{'floor_fraction':floor_fraction,'obstacle_fraction':blocked_fraction,
                           'unknown_fraction':unknown_fraction,'clear':reason=='clear','reason':reason}
