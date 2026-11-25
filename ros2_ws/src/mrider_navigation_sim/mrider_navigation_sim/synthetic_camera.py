"""Synthetic camera renderer shared by the world and offline diagnostics."""
from .geometry import ray_distances


def render_camera(pose,boxes,floor_rgb=(110,110,110),wall_rgb=(65,100,155),image_gain=1.):
    import numpy as np
    h,w,f=240,320,180.
    rgb=np.zeros((h,w,3),dtype=np.uint8);rgb[:h//2]=[190,210,230];rgb[h//2:]=floor_rgb
    x,y,yaw=pose
    angles=np.arctan((w/2-np.arange(w))/f)
    depths=ray_distances(x,y,yaw+angles,boxes)*np.cos(angles)
    tops=np.maximum(0,(h/2-f*.6/np.maximum(.1,depths)).astype(int))
    bottoms=np.minimum(h,(h/2+f*.6/np.maximum(.1,depths)).astype(int))
    rows=np.arange(h)[:,None]
    occupied=(depths[None,:]<14.9)&(rows>=tops)&(rows<bottoms)
    rgb[occupied]=wall_rgb
    rgb=np.clip(rgb.astype(float)*image_gain,0,255).astype(np.uint8)
    return rgb
