# 좌·우 우회와 보행자 자동 재개 영상

[MP4 재생](indoor-turns-ko.mp4)

2026-11-23 `ros2_humble`에서 `test_turns_launch.py`를 다시 실행하며 기록한
실제 ROS 시뮬레이션 telemetry를 시간축 그대로 재생한 영상이다.
RViz 화면 캡처나 실차 촬영 영상은 아니다. 정적 실내 지도, 차량 궤적,
횡단 보행자, 속도·조향 명령, 임무·안전 상태를 표시한다.
별도의 방 안에서 움직이는 두 번째 보행자 모형은 이 재생 그림에서 생략했다.

- 주행·정상 종료 검사 2개 통과: [XML](test_test_turns_launch.py.xunit.xml).
- 목표 오차 0.175847 m, 최소 이격 0.531914 m, 충돌 없음.
- [원본 시간별 기록과 판정 JSON](indoor_turns.json).
- 누적 회전과 조향 검사 기준은 [시험 안내](../../../turns-simulation-guide.md) 참조.

재생성 (호스트에서 ffmpeg, fontconfig, CJK 폰트, Python matplotlib 필요):

```bash
python3 tools/ros2/render_turns_video.py \
  docs/build/evidence/turns-video-2026-11-23/indoor_turns.json \
  docs/build/evidence/turns-video-2026-11-23/indoor-turns-ko.mp4
```

영상은 전체 주행 기록과 속도·조향 그래프를 포함한다. 주행 경과는 세로 커서로 표시하며,
상태 문자열은 해당 시점에 검사기가 마지막으로 수신한 값이다.
