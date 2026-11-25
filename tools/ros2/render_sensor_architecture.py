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
fig,ax=plt.subplots(figsize=(20,15))
fig.subplots_adjust(0,0,1,1);ax.set_xlim(0,1600);ax.set_ylim(1200,0);ax.axis('off')
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

text(800,32,'변경된 자율주행 구조 — 카메라·LiDAR 전용 / 비학습 방식',21,weight='bold')
text(800,72,'합성 RGB·모의 LiDAR 기반 센서 폐루프  |  최신 시험 결과는 각 시나리오 문서 참조  |  실센서 검증은 미완료',11)
text(800,104,'같은 이름의 입력은 동일 토픽의 연결 포트입니다. 반복 표시는 긴 연결선의 교차를 피하기 위한 것입니다.',10,color='#58677a')
box(40,161,260,140,'LiDAR 관측','/scan · LaserScan\n거리·각도 배열\n현재: 잡음 포함 모의 관측', '#dceaf6')
box(380,161,310,140,'LiDAR 위치 추정','키프레임 기반 점-직선 ICP\n정합 검사 / 로컬 원점 초기화\n명령 적분·정답 위치 입력 없음', '#dceedd')
box(770,161,330,140,'slam_toolbox · 지도 작성','입력: /mrider_sim/slam_scan + 추정 TF\nscan 정합 / 지도 생성·위치 보정\n미리 만든 정답 지도 입력 없음', '#dceedd')
box(1180,161,380,140,'SLAM 출력','/map · OccupancyGrid\nmap → odom 보정 TF\n실행 중 관측으로 만들어진 지도', '#eaf2f9')
for a,b in [(300,380),(690,770),(1100,1180)]:arrow(a,207,b,207)
text(340,255,'연속 scan',9)
text(730,255,'추정 TF',9)
text(1140,255,'지도·TF',9)
text(535,336,'추정 /mrider_sim/odometry + odom→base_link TF\nOdometry: 위치·자세·속도 / TF: 좌표계 변환',10)
text(935,336,'추가 입력: 원본 scan에서 만든 SLAM 복사본\n무반사점 제외 / 원본 시각·frame 유지',10)
box(40,400,260,122,'카메라 RGB','/camera/image_raw\nRGB + CameraInfo / 합성 영상', '#dceaf6')
box(380,400,310,122,'기하·영상 후보 영역 분할','보정 + LiDAR 기하 + Watershed\n바닥·장애물·긴 표면·미확인 후보', '#ffe9ad')
box(770,400,330,122,'카메라 판정 출력','/mrider_sim/camera_status\nvalid / clear / 지연·후보 비율', '#eaf2f9')
arrow(300,450,380,450);arrow(690,450,770,450)
text(340,492,'RGB',9);text(730,492,'검사 결과',9)
text(1370,450,'추가 입력: LiDAR /scan + CameraInfo\n외부 보정·시각 동기 필요 / 학습 모델 없음\n벽은 의미 확정이 아닌 긴 표면 후보',11)
text(800,574,'주행 입력과 제어 — 추정 위치·SLAM 지도·센서 관측만 사용',13,weight='bold')
box(40,611,330,126,'Nav2 입력','SLAM 지도 + LiDAR + TF → 비용지도\n추정 Odometry + TF → 차량 상태\n운영자 목표 → 임무 관리자', '#eaf2f9')
box(440,611,330,126,'경로 추종 입력','계획 경로 + 지역 비용지도\n추정 TF + Odometry\n차량 설정: vehicle1.yaml', '#eaf2f9')
box(840,611,330,126,'안전 게이트 입력','LiDAR + 추정 Odometry\n카메라 판정 + 주행 허가·임무 상태\nTF·Ground Truth 직접 구독 없음', '#eaf2f9')
box(1240,611,320,126,'실측값 기반 가상 차량','축거 0.533 m / 측정 반경 2.10 m\n모의 최대 속도 0.5 m/s\n등가 조향 한계 약 ±0.249 rad', '#eaf2f9')
for x in [205,605,1005,1400]:arrow(x,737,x,783)
box(40,786,330,128,'① Smac Hybrid 경로 계획','전진 경로 / 센서 실험 반경 3.0 m\n공식 차량 Nav2 반경 2.21 m와 구분\n출력: Path → 임무 관리자', '#ffe9ad')
box(440,786,330,128,'② RPP 경로 추종','곡률·장애물·목표 접근 감속\n출력: 후보 속도 v, 각속도 ω\n/mrider_sim/candidate_cmd_vel', '#ffddb9')
box(840,786,330,128,'③ 최종 안전 게이트','센서 유효성·차체 영역·정지거리\n카메라 불량/중단 시 정지 유지\n출력: 속도 v, 조향각 δ', '#e4dcf3')
box(1240,786,320,128,'④ 모의 차량 운동','/mrider_sim/command\nAckermannDriveStamped\n운동 결과로 다음 센서 관측 생성', '#d2eaf2')
for a,b in [(370,440),(770,840),(1170,1240)]:arrow(a,830,b,830)
text(405,883,'Path',9);text(805,883,'Twist',9);text(1205,883,'Ackermann',9)
box(40,974,730,78,'센서 관측의 폐루프','모의 환경 + 차량 운동 → 다음 RGB / LiDAR 관측 → 상단 입력으로 되먹임', '#eaf2f9')
line([(1400,914),(1400,1012),(820,1012)]);arrow(820,1012,770,1012)
text(1080,982,'모델 내부 상태는 센서 생성에만 사용',10)
box(40,1090,730,72,'Ground Truth: 평가 전용','/evaluation/ground_truth → 검사 노드의 목표 오차·추정 오차 측정', '#f6eded')
text(1190,1122,'평가 결과에서 주행 알고리즘으로 가는 연결 없음\nencoder·IMU 불필요 / 후보 분할의 실환경 적용은 후속 검증',10,color='#865050')
base=ROOT/'docs/figures/sensor-only-navigation-pipeline'
for extension in ['svg','pdf']:fig.savefig(str(base)+'.'+extension,facecolor='white')
fig.savefig(str(base)+'.png',dpi=100,facecolor='white')
