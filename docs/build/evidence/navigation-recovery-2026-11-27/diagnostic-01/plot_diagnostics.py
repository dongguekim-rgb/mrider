import json,gzip,math,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
p=Path(__file__).parent
d=json.load(gzip.open(p/'large_office_turns.trace.json.gz'));g=d['/mrider_sim/goal_grid'][-1];v=d['/mrider_sim/goal_validation'][-1]
a=np.asarray(g['data']).reshape(g['height'],g['width']);res=g['resolution'];ox,oy=g['origin'][:2]
fig,ax=plt.subplots(figsize=(11,7));ax.imshow(a,origin='lower',extent=[ox,ox+g['width']*res,oy,oy+g['height']*res],vmin=-1,vmax=100,cmap='gray_r')
ax.plot(*v['current'][:2],'ro',label='estimated pose');ax.plot(*v['destination'][:2],'bx',label='goal');ax.set(xlim=(20,29),ylim=(10,19));ax.legend();ax.grid();fig.savefig(p/'large-detail.png')
# Closest unknown/occupied to final goal and along a straight candidate.
blocked=np.array(np.where((a<0)|(a>20))).T
for pos in [v['current'][:2],v['destination'][:2],(25.5,14.),(25.5,15.)]:
 row,col=np.floor((np.array(pos)[::-1]-[oy,ox])/res).astype(int);print('large grid',pos,'value',a[row,col],'clearance',np.min(np.linalg.norm(blocked-np.array([row,col]),axis=1))*res)
d=json.load(gzip.open(p/'sensor_turns.trace.json.gz'));r=json.loads(gzip.open(p/'sensor_turns.json.gz','rt').read());g=d['/local_costmap/costmap_raw'][-1]
a=np.asarray(g['data']).reshape(g['height'],g['width']);res=g['resolution'];ox,oy=g['origin'][:2]
fig,ax=plt.subplots(figsize=(10,6));ax.imshow(a,origin='lower',extent=[ox,ox+g['width']*res,oy,oy+g['height']*res],vmin=0,vmax=255,cmap='gray_r')
plan=np.asarray(r['planned_paths'][0]);ax.plot(plan[:,0],plan[:,1],label='planned path')
tr=np.asarray(r['trajectory']);print('trace rows',tr.shape)
od=np.array([m['pose'][:2] for m in d['/mrider_sim/odometry']]);ax.plot(od[:,0],od[:,1],label='odometry')
arc=d['/lookahead_collision_arc'][-1];ap=np.asarray(arc['poses']);ax.plot(ap[:,0],ap[:,1],'r.-',label='collision projection')
f=np.asarray(d['/local_costmap/published_footprint'][-1]['points']);f=np.vstack([f,f[0]]);ax.plot(f[:,0],f[:,1],label='current padded footprint')
yaw=math.atan2(ap[-1,1]-ap[-2,1],ap[-1,0]-ap[-2,0]);theta0=2*math.atan2(*d['/mrider_sim/odometry'][-2]['pose'][2:]);tf=d['/mrider_sim/odometry'][-2]['pose'];delta=yaw-theta0
R=np.array([[math.cos(delta),-math.sin(delta)],[math.sin(delta),math.cos(delta)]])
future=(f-np.array(tf[:2]))@R.T+ap[-1,:2];ax.plot(future[:,0],future[:,1],'r--',label='projected footprint')
ax.set(xlim=(0,5),ylim=(-1.5,2));ax.legend();ax.grid();fig.savefig(p/'small-detail.png')
for x,y in plan[::8,:2]: print('plan',round(x,3),round(y,3))
