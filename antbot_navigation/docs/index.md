# AntBot Navigation Documentation

AntBot 스워브 드라이브 로봇의 Nav2 네비게이션 스택 문서.

## 문서 목차

| 문서 | 설명 |
|------|------|
| [nav2-overview.md](nav2-overview.md) | Nav2 프레임워크 개요 및 AntBot 적용 차별점 |
| [getting-started.md](getting-started.md) | 빠른 시작 가이드 (5분 안에 시뮬레이션 + 네비게이션 실행) |
| [architecture.md](architecture.md) | 시스템 아키텍처, TF 트리, 노드 그래프, 데이터 흐름 |
| [configuration.md](configuration.md) | 모든 설정 파일 파라미터 상세 설명 |
| [swerve-nav2-tuning.md](swerve-nav2-tuning.md) | 스워브 드라이브 특화 Nav2 튜닝 가이드 |
| [simulation.md](simulation.md) | Gazebo Ignition 시뮬레이션 환경 설명 |
| [troubleshooting.md](troubleshooting.md) | 문제 해결 가이드 |
| [changelog/](changelog/) | 변경 이력 |

## 패키지 구조

```
antbot_navigation/
├── config/
│   ├── sim/
│   │   ├── nav2_params.yaml            # Nav2 시뮬레이션용 파라미터
│   │   ├── ekf.yaml                    # EKF 센서 퓨전 (sim)
│   │   └── slam_toolbox_params.yaml    # SLAM 맵핑 (sim)
│   └── real/
│       ├── nav2_params.yaml            # Nav2 실제 로봇용 파라미터
│       ├── ekf.yaml                    # EKF 센서 퓨전 (real)
│       └── slam_toolbox_params.yaml    # SLAM 맵핑 (real)
├── launch/
│   ├── navigation.launch.py        # 전체 네비게이션 (맵 + AMCL + Nav2)
│   ├── localization.launch.py      # 로컬라이제이션 전용
│   └── slam.launch.py              # SLAM 맵핑 모드
├── maps/
│   ├── depot_sim.yaml              # 시뮬레이션용 맵
│   └── depot_sim.pgm
├── rviz/
│   └── navigation.rviz             # Nav2 시각화 설정
└── docs/                           # 이 문서
```
