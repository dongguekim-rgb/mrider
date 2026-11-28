#!/usr/bin/env python3
"""Render saved, measured scenario data (requires matplotlib; no ROS)."""
import argparse
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/mrider_navigation_sim'))
from mrider_navigation_sim.geometry import obstacles


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('report',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    report=json.loads(args.report.read_text())
    trace=report['trajectory'];path=report['planned_path'];result=report['result']
    assert result['case'] in ('static_obstacle','indoor','indoor_dynamic') and trace and path
    scenario=result['case']
    indoor=scenario.startswith('indoor')
    fig,(ax,velocity)=plt.subplots(1,2,figsize=(13,6))
    for x0,y0,x1,y1 in obstacles(scenario):
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='#9b3838',alpha=.8))
    ax.plot([p[0] for p in path],[p[1] for p in path],'--',color='#328345',label='Smac planned path')
    ax.plot([p[0] for p in trace],[p[1] for p in trace],color='#246aa1',label='Simulated rear-axle trajectory')
    ax.scatter([0,8],[0,0],marker='x',color='black',label='Start / goal')
    ax.set(xlim=(-2,12) if indoor else (-.5,8.7),ylim=(-5,5) if indoor else (-2,2),xlabel='x [m]',ylabel='y [m]')
    if indoor:
        from mrider_navigation_sim.indoor import ROOMS
        for x,y,label in ROOMS:ax.text(x,y,label,fontsize=7,ha='center')
        actor=report.get('pedestrian_trajectory',[])
        if actor:ax.plot([p[0] for p in actor],[p[1] for p in actor],':',color='orange',label='Pedestrian crossing')
    ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend(fontsize=8,loc='upper right')
    distance=[0.]
    for a,b in zip(trace,trace[1:]):distance.append(distance[-1]+math.hypot(b[0]-a[0],b[1]-a[1]))
    velocity.plot(distance,[p[2] for p in trace],color='#246aa1')
    velocity.axhline(.5,linestyle='--',color='gray',label='0.5 m/s limit')
    velocity.set(xlabel='Travelled distance [m]',ylabel='Measured simulated speed [m/s]',ylim=(-.02,.56))
    velocity.grid(alpha=.25);velocity.legend(fontsize=8)
    fig.suptitle('Smac Hybrid + RPP: ideal scan, known map, ground-truth odometry')
    fig.text(.5,.015,'Goal error: %.4f m | Minimum body clearance: %.4f m | Collision: %s' %
             (result['goal_error_m'],result['min_clearance_m'],result['collision']),ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.05,1,.95));args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
