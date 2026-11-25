#!/usr/bin/env python3
"""Paper-style Korean I/O diagram with manually routed, non-crossing edges."""
from pathlib import Path
import subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
ROOT=Path(__file__).resolve().parents[2]
fontpath=subprocess.check_output(['fc-match','-f','%{file}','Noto Sans CJK KR'],text=True).strip()
name=font_manager.FontProperties(fname=fontpath).get_name()
font_manager.fontManager.ttflist.insert(0,font_manager.FontEntry(fname=fontpath,name=name))
plt.rcParams.update({'font.family':name,'svg.fonttype':'path','pdf.fonttype':3})
fig,ax=plt.subplots(figsize=(20,14))
fig.subplots_adjust(0,0,1,1);ax.set_xlim(0,1600);ax.set_ylim(1120,0);ax.axis('off')
def text(x,y,s,size=11,color='#243247',weight='normal'):
 from matplotlib.textpath import TextPath
 from matplotlib.patches import PathPatch
 from matplotlib.transforms import Affine2D
 rows=s.split('\n');step=size*1.75
 for i,row in enumerate(rows):
  path=TextPath((0,0),row,size=size*1.12,prop=font_manager.FontProperties(fname=fontpath,weight=weight))
  bounds=path.get_extents()
  transform=Affine2D().scale(1,-1).translate(x-(bounds.x0+bounds.x1)/2,y+(i-(len(rows)-1)/2)*step+(bounds.y0+bounds.y1)/2)
  ax.add_patch(PathPatch(path,transform=transform+ax.transData,facecolor=color,lw=0))
def box(x,y,w,h,title,body='',color='#edf3f8'):
 ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0,rounding_size=12',facecolor=color,edgecolor='#394b61',linewidth=1.2))
 text(x+w/2,y+25,title,12,weight='bold')
 if body:text(x+w/2,y+h/2+17,body,10)
def arrow(x1,y1,x2,y2,color='#34475d'):
 ax.add_patch(FancyArrowPatch((x1,y1),(x2,y2),arrowstyle='-|>',mutation_scale=15,lw=1.6,color=color))
def line(points,color='#34475d'):
 ax.plot(*zip(*points),color=color,lw=1.6)
text(800,32,'기존 GT 기준 주행 시험 — 데이터별 입력과 출력',21,weight='bold')
text(800,69,'같은 이름의 입력은 동일한 ROS 데이터를 사용합니다. 센서·상태 데이터와 계산된 비용지도를 구분했습니다.',11)
# Four independent source contracts, not one combined pose/TF sensor.
box(40,105,350,110,'지도','/map · OccupancyGrid\n알려진 정적 장애물 지도', '#eaf2f9')
box(430,105,350,110,'LiDAR','/scan · LaserScan\n모의 광선 거리 관측 · 10 Hz', '#eaf2f9')
box(820,105,350,110,'Odometry','/mrider_sim/odometry\n정답 위치·자세·속도 · 50 Hz', '#eaf2f9')
box(1210,105,350,110,'TF','/tf · /tf_static\n좌표계 사이의 위치·방향 변환', '#eaf2f9')
text(800,239,'위 데이터의 공급자: world_node  |  지도는 미리 생성하며 Odometry와 TF는 모델 정답에서 생성  |  SLAM·카메라 미실행',10,color='#58677a')
# Perception is shown as its own computation; costmaps are outputs.
box(40,283,260,110,'비용지도 생성 입력','지도 /map\nLiDAR /scan + TF', '#eaf2f9')
box(390,283,360,110,'주변 인식 · 비용지도 생성','정적 지도 + scan 장애물 표시·제거\n좌표 변환 + 장애물 주변 비용 확장', '#dceedd')
arrow(300,338,390,338)
box(900,271,290,62,'전역 비용지도','', '#dceedd')
box(900,363,290,62,'지역 비용지도','', '#dceedd')
line([(750,338),(815,338),(815,302)]);arrow(815,302,900,302)
line([(815,338),(815,394)]);arrow(815,394,900,394)
text(1380,300,'map 좌표계 / 경로 계획에 사용',11)
text(1380,393,'odom 좌표계 / 주변 8 × 8 m\n경로 추종의 장애물 판단에 사용',11)
text(800,458,'주행 데이터 흐름  ·  위에서 계산한 전역·지역 비용지도를 아래 입력에 이름으로 연결',12,weight='bold')
# Explicit consumer contracts distinguish odometry from TF.
box(40,497,330,128,'경로 계획 입력','전역 비용지도 (위 처리 결과)\nTF → 현재 위치\n목표 PoseStamped (임무 관리자)', '#f0f4f8')
box(440,497,330,128,'경로 추종 입력','지역 비용지도 (위 처리 결과)\nTF → 현재 위치·경로 좌표 변환\nOdometry → 차량 속도', '#f0f4f8')
box(840,497,330,128,'안전 판단 입력','LiDAR → 장애물 거리\nOdometry → 차량 상태·속도\n운영자 허가 / 임무 상태', '#f0f4f8')
box(1240,497,320,128,'모의 차량 제원','축거 0.65 m / 최대 0.5 m/s\n최대 조향 ±0.3927 rad\n가감속·조향 변화율 제한', '#f0f4f8')
for x in [205,605,1005,1400]:arrow(x,625,x,677)
box(40,680,330,137,'① Smac Hybrid 경로 계획','전진 전용 Dubins 탐색\n최소 회전반경 2.2 m\n출력: 計획 경로 Path'.replace('計','계'), '#ffe9ad')
box(440,680,330,137,'② RPP 경로 추종 · 30 Hz','계획 경로 + 현재 위치 + 지역 비용\n곡률·장애물·목표 접근에 따른 감속\n출력: 후보 속도 v, 각속도 ω', '#ffddb9')
box(840,680,330,137,'③ 안전 게이트 · 50 Hz','입력 유효성·차체 이동 영역·정지거리\nTwist → 조향각 / 감속·정지 상태 유지\nTF 직접 구독 없음', '#e4dcf3')
box(1240,680,320,137,'④ 모의 차량 · 50 Hz','자전거 운동 모델 적분\n입력: 최종 속도 v, 조향각 δ\n출력: 갱신된 차량 정답 상태', '#d2eaf2')
for a,b in [(370,440),(770,840),(1170,1240)]:arrow(a,725,b,725)
text(405,778,'계획 경로\nPath',9)
text(805,778,'후보 명령\nTwist',9)
text(1205,778,'최종 명령\nAckermann',9)
text(205,859,'ComputePathToPose 결과 → 임무 관리자\n→ FollowPath action으로 경로 전달',10)
text(605,859,'/mrider_sim/candidate_cmd_vel\nv [m/s], ω [rad/s]',10)
text(1005,859,'/mrider_sim/command\nAckermannDriveStamped: v [m/s], δ [rad]',10)
box(40,916,770,82,'world_node: 모의 환경과 데이터 발행','차량 상태 + 벽·가구·보행자 → LiDAR / Odometry / TF 갱신', '#eaf2f9')
line([(1400,817),(1400,956),(850,956)]);arrow(850,956,810,956)
text(1110,928,'차량 정답 상태의 되먹임',10)
text(800,1030,'임무 제어: 최초 arm/start → 계획·추종 → 장애물 정지·취소 → 3초 안전 조건 확인 → 재허가·저장 목표 재계획',11)
text(800,1070,'센서·명령 오류는 수동 재허가  |  map→odom과 base_link→laser는 고정 항등 TF  |  게이트는 laser 관측점을 차량 기준으로 사용',10,color='#58677a')
text(800,1101,'실제 센서 입력·SLAM 결과가 아닌 모의 검증 구조  |  표시 주기는 설정값  |  조향각 양수는 좌회전',10,color='#58677a')
base=ROOT/'docs/figures/indoor-navigation-pipeline'
for extension in ['svg','pdf']:fig.savefig(str(base)+'.'+extension,facecolor='white')
fig.savefig('/tmp/mrider-architecture-clean.png',dpi=100,facecolor='white')
