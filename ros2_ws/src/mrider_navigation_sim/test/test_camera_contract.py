import math
import pytest
from mrider_navigation_sim.camera_contract import camera_fault

@pytest.mark.parametrize('data,wall,now,reason',[
 ({'stamp':10.,'valid':True,'clear':True},10.1,10.1,None),
 ({'stamp':10.,'valid':True,'clear':False},10.1,10.1,'camera_obstacle'),
 ({'stamp':10.,'valid':True,'clear':False,'reason':'unknown_region'},10.1,10.1,'camera_unknown'),
 ({'stamp':10.,'valid':False,'clear':True},10.1,10.1,'camera_invalid_input'),
 ({'stamp':10.,'valid':True,'clear':True},10.5,10.1,'camera_status_timeout'),
 ({'stamp':10.,'valid':True,'clear':True},10.1,10.5,'camera_input_stale'),
 ({'stamp':11.,'valid':True,'clear':True},10.1,10.1,'camera_future_stamp'),
 ({'stamp':math.nan,'valid':True,'clear':True},10.1,10.1,'camera_invalid_stamp'),
 ({'stamp':'10','valid':True,'clear':True},10.1,10.1,'camera_invalid_stamp'),
 ([],10.1,10.1,'camera_invalid_status'),
])
def test_camera_reason(data,wall,now,reason):
    assert camera_fault((data,10.),now,wall)==reason

def test_missing_camera():
    assert camera_fault(None,10.,10.)=='camera_missing'
