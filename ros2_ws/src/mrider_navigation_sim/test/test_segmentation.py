import math
import cv2
import numpy as np
import pytest
from mrider_navigation_sim.segmentation import CameraGeometry, segment
from mrider_navigation_sim.geometry import ray_distance


def scene(floor=(110,110,110),gain=1.,obstacle=False):
    h,w,f=240,320,180.
    boxes=[(5.,-5.,5.2,5.),(-1.,-3.,6.,-2.8),(-1.,2.8,6.,3.)]
    if obstacle:boxes.append((.9,-.5,1.2,.5))
    ranges=[ray_distance(0,0,-math.pi+i*2*math.pi/360,boxes) for i in range(360)]
    rgb=np.empty((h,w,3),np.uint8);rgb[:120]=[190,210,230];rgb[120:]=floor
    for col in range(w):
        a=math.atan((w/2-col)/f);d=ray_distance(0,0,a,boxes)*math.cos(a)
        if d<14.9:
            top=max(0,int(120-f*.6/max(.1,d)));bottom=min(h,int(120+f*.6/max(.1,d)))
            rgb[top:bottom,col]=[65,100,155]
    rgb=np.clip(rgb.astype(float)*gain,0,255).astype(np.uint8)
    return rgb,ranges


def run(rgb,ranges,camera=None):
    return segment(rgb,ranges,-math.pi,2*math.pi/360,.05,15.,camera or CameraGeometry(180,180,160,120,.6))


@pytest.mark.parametrize('floor,gain',[
 ((110,110,110),1.),((175,120,65),1.),((35,35,35),1.),
 ((180,180,180),1.),((100,150,70),.5),((100,150,70),1.4),
])
def test_floor_not_fixed_gray(floor,gain):
    labels,overlay,status=run(*scene(floor,gain))
    assert status['clear'],status
    assert labels.dtype==np.uint8 and overlay.shape==(240,320,3)
    assert np.any(labels==3), 'long surface candidate missing'


def test_obstacle_blocks():
    labels,_,status=run(*scene(obstacle=True))
    assert not status['clear'] and status['obstacle_fraction']>.2,status
    assert np.any(labels==2)


def test_same_color_obstacle_not_free_by_scan():
    labels,_,status=run(*scene(floor=(65,100,155),obstacle=True))
    assert not status['clear'],status


def test_unknown_no_scan_return_not_free():
    rgb,_=scene()
    with pytest.raises(ValueError,match='floor_seed'):
        run(rgb,[15.]*360)


def test_missing_calibration():
    with pytest.raises(ValueError,match='calibration'):
        run(*scene(),camera=CameraGeometry(0,180,160,120,.6))


def test_nan_calibration():
    with pytest.raises(ValueError,match='calibration'):
        run(*scene(),camera=CameraGeometry(180,180,160,120,float('nan')))


def test_shadow_remains_conservative():
    rgb,ranges=scene();rgb[180:220,140:180]//=3
    labels,_,status=run(rgb,ranges)
    assert status['unknown_fraction']>0, status


def test_vector_rays_preserve_geometry():
    from mrider_navigation_sim.geometry import ray_distances
    boxes=[(-2,-3,-1,4),(3,-2,4,2),(-1,5,6,6)]
    angles=np.linspace(-math.pi,math.pi,721)
    actual=ray_distances(.3,-.1,angles,boxes)
    expected=[ray_distance(.3,-.1,a,boxes) for a in angles]
    assert np.allclose(actual,expected,atol=1e-10)


def test_verified_clear_max_range_supports_floor():
    rgb,_=scene()
    _,_,status=segment(rgb,[15.]*360,-math.pi,2*math.pi/360,.05,15.,CameraGeometry(180,180,160,120,.6),True)
    assert status['clear']


@pytest.mark.parametrize('invalid',[0.,float('inf'),float('nan'),16.])
def test_verified_contract_does_not_accept_invalid_rays(invalid):
    rgb,_=scene()
    with pytest.raises(ValueError,match='floor_seed'):
        segment(rgb,[invalid]*360,-math.pi,2*math.pi/360,.05,15.,CameraGeometry(180,180,160,120,.6),True)
