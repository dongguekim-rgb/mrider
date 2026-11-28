#!/usr/bin/env python3
"""Render recorded ROS simulation telemetry, not a desktop/RViz capture."""
import argparse
import bisect
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Polygon, Rectangle

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/mrider_navigation_sim'))
from mrider_navigation_sim.geometry import obstacles, footprint


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    report=json.loads(args.report.read_text())
    frames=report['video_frames']
    times=[f['t']-frames[0]['t'] for f in frames]
    import subprocess
    from matplotlib import font_manager
    fontpath=subprocess.check_output(['fc-match','-f','%{file}','Noto Sans CJK KR'],text=True).strip()
    font_manager.fontManager.ttflist.insert(0,font_manager.FontEntry(fname=fontpath,name=font_manager.FontProperties(fname=fontpath).get_name()))
    plt.rcParams['font.family']=font_manager.FontProperties(fname=fontpath).get_name()
    plt.rcParams['axes.unicode_minus']=False
    fig=plt.figure(figsize=(16,9),facecolor='#f5f7fa')
    grid=fig.add_gridspec(3,1,height_ratios=[5,1,1],hspace=.55)
    ax=fig.add_subplot(grid[0]);ax.set_aspect('equal');ax.set_xlim(-2,24);ax.set_ylim(-5,5)
    ax.set_title('좌·우 조향 우회 → 보행자 정지 → 자동 재개 → 목표 도착',fontsize=18,pad=16)
    ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.grid(alpha=.15)
    for a,b,c,d in obstacles('indoor_turns'):
        ax.add_patch(Rectangle((a,b),c-a,d-b,facecolor='#66768a',edgecolor='white'))
    ax.plot(20,0,'*',color='#18a56e',markersize=18,label='목표')
    trail,=ax.plot([],[],color='#148bcc',lw=2,label='실제 차량 궤적')
    car=Polygon(footprint((0,0,0)),facecolor='#148bcc',edgecolor='#083d63',zorder=6)
    ax.add_patch(car)
    actor=Rectangle((11.75,-2.35),.5,.5,color='#f07e27',zorder=7,label='횡단 보행자')
    ax.add_patch(actor)
    heading,=ax.plot([],[],color='white',lw=3,zorder=8)
    ax.legend(loc='upper right',fontsize=9)
    status=fig.text(.07,.925,'',fontsize=12,color='#223047')
    sub=[];cursors=[]
    for idx,(key,label,color) in enumerate([('speed','실제 속도 [m/s]','#148bcc'),('steering','명령 조향각 [rad]','#b34c93')]):
        a=fig.add_subplot(grid[idx+1]);a.plot(times,[f[key] for f in frames],color=color,lw=1.4)
        a.set_ylabel(label);a.set_xlim(0,times[-1]);a.grid(alpha=.25)
        if key=='steering':a.axhline(0,color='gray',lw=.6);a.set_ylim(-.41,.41)
        else:a.set_ylim(-.03,.55)
        cursors.append(a.axvline(0,color='#e27d21',lw=2));sub.append(a)
    sub[-1].set_xlabel('재생 시간 [s] · 실제 기록 시간과 1:1')
    fig.text(.07,.015,'ROS 주행 로그 재생 영상 (RViz 화면 녹화 아님) · 정적 지도와 횡단 보행자 표시 · SLAM/카메라 미사용',fontsize=10)
    fig.subplots_adjust(top=.87,bottom=.1,left=.08,right=.96)
    writer=FFMpegWriter(fps=15,codec='libx264',extra_args=['-pix_fmt','yuv420p','-crf','22','-preset','fast'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with writer.saving(fig,str(args.output),dpi=100):
        for n in range(math.ceil(times[-1]*15)+1):
            t=min(n/15,times[-1]);i=max(0,bisect.bisect_right(times,t)-1);f=frames[i]
            car.set_xy(footprint((f['x'],f['y'],f['yaw'])))
            trail.set_data([v['x'] for v in frames[:i+1]],[v['y'] for v in frames[:i+1]])
            actor.set_y((f['pedestrian_y'] if f['pedestrian_y'] is not None else -2.1)-.25)
            heading.set_data([f['x'],f['x']+.75*math.cos(f['yaw'])],[f['y'],f['y']+.75*math.sin(f['yaw'])])
            turn='좌조향' if f['steering']>.02 else '우조향' if f['steering']<-.02 else '직진'
            status.set_text(f"{t:05.1f} s  |  {turn}  |  임무: {f['mission']}  |  안전: {f['safety'].get('state','')} / {f['safety'].get('reason','')}")
            for cursor in cursors:cursor.set_xdata([t,t])
            writer.grab_frame()
    plt.close(fig)
    print(args.output)

if __name__=='__main__':main()
