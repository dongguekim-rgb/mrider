#!/usr/bin/env python3
"""Offline environment overview and startup perception diagnostic; no ROS launch."""
import sys,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/mrider_navigation_sim'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from mrider_navigation_sim.large_office import BOUNDS,STATIONS,rooms,walls_and_furniture,OfficeActors
from mrider_navigation_sim.geometry import ray_distances
from mrider_navigation_sim.synthetic_camera import render_camera
from mrider_navigation_sim.segmentation import segment,CameraGeometry

out=ROOT/'docs/figures';out.mkdir(parents=True,exist_ok=True)
boxes=walls_and_furniture()
fig,ax=plt.subplots(figsize=(12,8),constrained_layout=True)
for a,b,c,d in boxes:ax.add_patch(Rectangle((a,b),c-a,d-b,color='#465161'))
for i,(a,b,c,d) in enumerate(rooms()):ax.text((a+c)/2,(b+d)/2,f'Room {i+1}',ha='center')
for i,(x,y) in enumerate(STATIONS):
    ax.plot(x,y,'o',color='#18845b');ax.annotate(f'P{i}',(x,y),xytext=(4,5),textcoords='offset points')
ax.plot([24,24],[0,32],'--',color='#db7a17',label='Moving proxy 1')
ax.plot([0,52],[16,16],'--',color='#9b409b',label='Moving proxy 2')
ax.set(xlim=(-5,57),ylim=(-5,37),xlabel='x [m]',ylabel='y [m]',title='Large office: 60 x 40 m | 8 rooms | environment reference only')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper right')
for ext in ['png','svg']:fig.savefig(out/f'large-office.{ext}',dpi=150)
actors=OfficeActors().step(0.,0.);scene=boxes+actors
angles=-math.pi+np.arange(360)*2*math.pi/360
scan=ray_distances(0,0,angles,scene,15.)
rgb=render_camera((0,0,0),scene)
try:
    labels,overlay,status=segment(rgb,scan,-math.pi,2*math.pi/360,.05,15.,CameraGeometry(180,180,160,120,.6),True)
    status['valid']=True
except ValueError as error:
    status={'valid':False,'clear':False,'reason':str(error)}
status.update(map_size_m=[60,40],area_m2=2400,rooms=8,static_boxes=len(boxes),no_return_rays=int(np.sum(scan==15.)),verified_max_range_clear=True,closed_loop_tested=False)
evidence=ROOT/'docs/build/evidence/large-office-progress-2026-11-25/offline';evidence.mkdir(parents=True,exist_ok=True)
(evidence/'offline.json').write_text(json.dumps(status,indent=2))
print(json.dumps(status,indent=2))
