# Nav2 Overview

Nav2는 ROS 2의 공식 네비게이션 프레임워크로, 로봇을 A 지점에서 B 지점으로 안전하게 이동시키는 전체 파이프라인을 제공한다.

## 핵심 컴포넌트

```
사용자가 목표 설정
       │
       ▼
┌─ BT Navigator ─────────────────────────────────┐
│  행동 트리로 전체 흐름 오케스트레이션             │
│  "계획 실패? → 스핀 → 재시도 → 대기 → 포기"     │
│                                                  │
│  ┌──────────────┐    ┌───────────────────┐      │
│  │ Planner      │    │ Controller        │      │
│  │ Server       │───▶│ Server            │      │
│  │              │    │                   │      │
│  │ "어디로      │    │ "어떻게           │      │
│  │  갈 것인가"  │    │  따라갈 것인가"   │      │
│  │              │    │                   │      │
│  │ NavFn(A*)    │    │ MPPI/DWB/RPP     │      │
│  │ SmacPlanner  │    │                   │      │
│  └──────┬───────┘    └────────┬──────────┘      │
│         │                     │                  │
│  ┌──────▼───────┐    ┌───────▼──────────┐       │
│  │ Global       │    │ Local            │       │
│  │ Costmap      │    │ Costmap          │       │
│  │ (정적맵+장애물│    │ (실시간 장애물    │       │
│  │  +inflation) │    │  +inflation)     │       │
│  └──────────────┘    └──────────────────┘       │
│                                                  │
│  ┌──────────────┐                               │
│  │ Behavior     │  spin, backup, wait           │
│  │ Server       │  "막혔을 때 복구 행동"         │
│  └──────────────┘                               │
└──────────────────────────────────────────────────┘
```

## Planner Server — 글로벌 경로 계획

맵 위에서 출발점에서 목표점까지의 경로를 계산한다.

| 플러그인 | 방식 | 특징 |
|----------|------|------|
| **NavFn** | A* / Dijkstra | 가장 기본, 격자 기반 |
| **SmacPlanner2D** | 2D State Lattice | 회전 반경 고려 가능 |
| **SmacPlannerHybrid** | Hybrid-A* | Ackermann 로봇에 적합 |
| **Theta*** | Any-angle | 대각선 경로 가능 |

AntBot은 홀로노믹이라 **NavFn A***로 충분하다.

## Smoother Server — 경로 스무딩

Planner가 만든 격자 기반 경로의 꺾임을 부드러운 곡선으로 변환한다.

```
Before:  ┌──┐          After:  ╭──╮
         │  │                  │  │
    ─────┘  └─────        ────╯  ╰─────
```

## Controller Server — 로컬 경로 추종

글로벌 경로를 실시간으로 따라가면서 장애물을 회피한다.

| 플러그인 | 방식 | 적합한 로봇 |
|----------|------|-------------|
| **DWB** | Dynamic Window 샘플 평가 | diff-drive, 간단한 홀로노믹 |
| **MPPI** | 2000개 궤적 rollout 최적화 | **스워브, 복잡한 동역학** |
| **RPP** | Regulated Pure Pursuit | Ackermann, 고속 주행 |

AntBot은 steering re-alignment 지연이 있어 **MPPI**가 최적이다.

## Costmap — 장애물 지도

```
레이어 구조:

Static Layer      ← 저장된 맵 (벽, 구조물)
    +
Obstacle Layer    ← 실시간 센서 (LiDAR)
    +
Inflation Layer   ← 장애물 주변 안전 거리
    =
최종 Costmap      ← Planner/Controller가 사용
```

비용 값: 0(free) ~ 253(inscribed) ~ 254(lethal) ~ 255(unknown)

## Behavior Server — 복구 행동

경로 추종 실패 시 자동으로 시도하는 행동:

| 행동 | 동작 | 언제 |
|------|------|------|
| **Spin** | 제자리 회전 | 길이 막혔을 때 |
| **BackUp** | 후진 | 좁은 곳에 끼었을 때 |
| **Wait** | 대기 | 동적 장애물이 지나가길 기다림 |

## BT Navigator — 행동 트리

모든 컴포넌트를 조합하는 오케스트레이터. 실패 시 자동으로 복구 행동 → 재계획 → 재시도.

```
NavigateToPose
  ├── ComputePathToPose (Planner)
  ├── SmoothPath (Smoother)
  ├── FollowPath (Controller)
  └── RecoveryNode
       ├── Spin
       ├── Wait
       └── BackUp
```

## Localization — 위치 추정

Nav2가 경로를 계획하려면 로봇이 맵에서 어디에 있는지 알아야 한다.

- **Map Server**: 저장된 맵을 `/map` 토픽으로 발행
- **AMCL**: 파티클 필터로 맵 상 위치 추정 (`map→odom` 변환)
- **EKF**: 바퀴 오도메트리 + IMU를 퓨전 (`odom→base_link` 변환)
- **SLAM Toolbox**: 맵이 없을 때 맵을 생성하면서 localization

## Lifecycle 관리

모든 Nav2 노드는 managed lifecycle 패턴을 따른다:

```
unconfigured → inactive → active → finalized
```

`lifecycle_manager`가 모든 노드를 순서대로 configure → activate하며, 하나라도 실패하면 전체가 중단된다.

## AntBot에서의 차별점

| 항목 | 일반 diff-drive | AntBot swerve |
|------|-----------------|---------------|
| Controller | DWB | **MPPI** |
| Motion Model (AMCL) | DifferentialMotionModel | **OmniMotionModel** |
| 속도 자유도 | vx, wz (2DOF) | **vx, vy, wz (3DOF)** |
| Inflation | ~0.3m | **0.75m** (steering 오버슈트 대비) |
| LiDAR | 단일 | **듀얼 2D + 3D** |
| Odometry TF | 직접 발행 | **EKF 센서 퓨전** (충돌 방어) |
