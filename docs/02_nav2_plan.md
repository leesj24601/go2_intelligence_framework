# Implementation Plan: Go2 Nav2 Navigation in Isaac Sim

## Goal
Isaac Sim 환경에서 RTAB-Map으로 생성한 맵(또는 실시간 SLAM 중인 맵)과 Odometry/TF를 기반으로, ROS2 Navigation2 (Nav2) 스택을 연동하여 Go2 로봇이 목표 지점(Goal Pose)까지 스스로 장애물을 회피하며 이동(자율주행)하도록 구현한다.

## 현재 상태 및 입력(Input)
- **로봇 및 센서**: Unitree Go2, 가상 RealSense 카메라 (RGB-D)
- **TF 트리**: `odom` -> `base_link` -> `camera_link` -> `camera_optical_frame`
- **Odometry**: `/odom` 토픽 (OmniGraph IsaacComputeOdometry에서 발행, 6DoF)
- **Map**: `/map` (OccupancyGrid, RTAB-Map 또는 맵 서버에서 제공)
- **Sensor Data**: `/camera/depth/image_rect_raw` 등, 또는 변환된 2D LaserScan / PointCloud2 (장애물 회피용)

## 아키텍처 및 연동 흐름 (예상)

```
[Isaac Sim (Go2)] --(Odometry/Depth)--> [Nav2 Stack] --(cmd_vel)--> [Isaac Sim (Go2 속도 명령)]
```

### Nav2 핵심 컴포넌트
1. `nav2_amcl` (또는 RTAB-Map Localization 모드): 로봇의 현재 위치 파악 (Map -> Odom TF 발행)
2. `nav2_planner` (Global Planner): 시작점에서 목표점까지의 최적 경로 생성 (A*, Dijsktra 등)
3. `nav2_controller` (Local Planner / DWB): 전역 경로를 따라가며 실시간 장애물을 피하기 위한 속도(`cmd_vel`) 계산
4. `nav2_costmap_2d`: 전역/지역 맵과 센서(Depth/Laser) 데이터를 합성하여 장애물 회피 구역 설정
5. `nav2_bt_navigator`: Behavior Tree 기반의 네비게이션 상태(시작-회피-종료) 관리

---

## Phase 1: 센서 데이터 변환 및 준비 (Depth to LaserScan)
Nav2의 Costmap은 2D LaserScan(`sensor_msgs/LaserScan`) 데이터를 다룰 때 가장 가볍고 안정적이다. (또는 PointCloud2 사용 가능)
- **목표**: Isaac Sim의 Depth 이미지나 RTAB-Map의 PointCloud를 2D LaserScan으로 변환하는 노드(예: `depthimage_to_laserscan` 또는 `pointcloud_to_laserscan`) 설정.

## Phase 2: Nav2 파라미터 (YAML) 커스텀 설정
Go2 로봇은 바퀴형 로봇(Differential Drive 등)이 아닌 족형(Omnidirectional/Holonomic 가능) 로봇 모델을 따른다.
- **다루어야 할 주요 설정**:
  - `robot_radius` 또는 `footprint` (로봇 크기)
  - Controller (DWB)의 속도 제한 (`max_vel_x`, `max_vel_y`, `max_vel_theta`)
  - Costmap의 센서 소스 (observation sources) 추가 (LaserScan 토픽)

## Phase 3: Launch 파일 작성 및 노드 통합
- `nav2_bringup` 패키지를 활용한 커스텀 launch 파일 작성 (예: `go2_nav2.launch.py`)
- SLAM + Nav2 동시 실행(SLAM 모드) 또는 맵 로드 + Nav2 실행(Localization 모드) 분기 처리

## Phase 4: `cmd_vel` 제어권 연동
- **목표**: Nav2가 발행하는 `/cmd_vel` (geometry_msgs/Twist)을 Isaac Lab 환경 스크립트(`go2_sim.py`)가 수신하여 로봇을 구동하도록 수정
- 현재 키보드 제어(`WasdKeyboard`) 부분을 ROS2 `cmd_vel` 구독 형태로 변경하거나 두 가지 모드를 스위칭할 수 있도록 커스텀 명령 노드 작성.

## Phase 5: 테스트 및 튜닝
- RViz2에서 `2D Goal Pose` 버튼을 클릭하여 로봇 이동 명령 테스트
- Costmap 파라미터(인플레이션 반지름 등) 튜닝
- 로봇이 기둥이나 상자를 부딪히지 않고 부드럽게 회피하는지 검증

---

## 작업 목록 관리 (Task Checklist)

### [ ] Phase 1: 센서 변환
- [ ] `depthimage_to_laserscan` (또는 유사 패키지) 설치 확인 및 Launch 설정
- [ ] RViz에서 변환된 LaserScan 데이터가 지면과 평행하게 잘 나오는지 검증

### [ ] Phase 2 & 3: Nav2 설정 및 Launch
- [ ] Nav2 설치 (`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`)
- [ ] `config/go2_nav2_params.yaml` 생성 및 Go2 모델에 맞춰 파라미터 튜닝
- [ ] `launch/go2_navigation.launch.py` 작성
- [ ] Nav2 노드 활성화 및 디버깅

### [ ] Phase 4: Isaac Sim 명령 수신 연동
- [ ] `go2_sim.py` 내부에 `/cmd_vel` Topic 수신 (또는 OmniGraph 연결) 구현
- [ ] 수신한 Twist 명령을 로봇 제어 명령(`base_velocity`)에 매핑 (상태 전이 확인)

### [ ] Phase 5: 최종 테스트
- [ ] RViz2 Nav2 패널에서 2D Goal Pose 지정 후 이동 확인
- [ ] 로컬 장애물 회피 능력 평가 및 튜닝
