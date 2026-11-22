import math
import pytest
from mrider_navigation_sim.indoor import Pedestrians, walls, furniture, box
from mrider_navigation_sim.geometry import obstacles, clearance, ray_distance


def test_indoor_start_goal_clear():
    boxes=obstacles('indoor')
    assert len(boxes)>20
    for pose in [(0.,0.,0.),(8.,0.,0.)]:
        assert clearance(pose,boxes)>.5
    assert obstacles('indoor_dynamic')==boxes


def test_doorway_and_wall_scan():
    assert ray_distance(6.,0.,math.pi/2,obstacles('indoor'))>4.5
    assert ray_distance(3.,0.,math.pi/2,walls())==pytest.approx(2.5)
    assert ray_distance(6.,0.,math.pi/2,[box(6.,1.)])==pytest.approx(.75)


def test_actor_passes_through_door_and_never_hits_static_world():
    actor=Pedestrians();seen=[]
    for i in range(1000):
        boxes=actor.step(.02,4.)
        seen.append(actor.crossing_y)
        for a,b,c,d in boxes:
            assert clearance(((a+c)/2,(b+d)/2,0.),obstacles('indoor'),.25,.25,.25)>0
    assert min(seen)<-2 and max(seen)==pytest.approx(3.3) and actor.finished
    assert any(abs(y)<1e-9 for y in seen)


def test_actor_trigger_and_stop_independence():
    actor=Pedestrians()
    for i in range(50):actor.step(.1,2.4)
    assert actor.crossing_elapsed is None and actor.crossing_y==-2.1
    actor.step(.1,2.5)
    for i in range(200):actor.step(.1,2.5)
    assert actor.finished  # stationary vehicle cannot freeze actor


@pytest.mark.parametrize('dt',[-1.,math.nan,math.inf])
def test_invalid_actor_time(dt):
    with pytest.raises(ValueError):Pedestrians().step(dt,0.)
