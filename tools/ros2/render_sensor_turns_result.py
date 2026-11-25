#!/usr/bin/env python3
"""Plot recorded estimates and safety output, without feeding truth to control."""
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

source=Path(sys.argv[1])
data=json.loads(source.read_text())
fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
trace=data['trajectory']
axes[0].plot([p[0] for p in trace],[p[1] for p in trace],color='royalblue')
axes[0].set(title='LiDAR odometry trajectory (not ground truth)',xlabel='odom x [m]',ylabel='odom y [m]')
axes[0].grid(True)
states=data['safety_history'];start=states[0][0]
times=[t-start for t,_ in states]
axes[1].plot(times,[s['speed'] for _,s in states],color='royalblue',label='Gated speed command')
axes[1].fill_between(times,0,.4,where=[s['state']=='STOPPED' for _,s in states],alpha=.2,color='red',label='STOPPED')
axes[1].set(title='Obstacle wait, resume, then camera fault test',xlabel='Elapsed [s]',ylabel='Speed [m/s]')
axes[1].legend();axes[1].grid(True)
fig.savefig(source.with_suffix('.png'),dpi=160)
