# Swerve Drive Nav2 Tuning Guide

일반 diff-drive와 다른 AntBot 스워브 드라이브만의 Nav2 튜닝 포인트.

> **sim vs real 튜닝**: Nav2 파라미터는 `config/sim/`과 `config/real/`로 분리되어 있다.
> 시뮬레이션에서는 steering 지연이 없고 센서 노이즈가 적으므로 더 공격적인 파라미터가 가능하다.
> 실제 로봇에서는 inflation, speed limit, rejection threshold 등을 보수적으로 설정해야 한다.
> Launch 시 `mode:=sim` 또는 `mode:=real`로 해당 설정이 자동 선택된다.

## 왜 MPPI인가? (DWB 대신)

| 항목 | DWB | MPPI |
|------|-----|------|
| 방식 | 단일 샘플 평가 | 2000개 궤적 rollout + 비용 최적화 |
| 모션 모델 | holonomic 가능하나 제한적 | `Omni` 모델 네이티브 지원 |
| steering 지연 대응 | 즉각 반응 가정 | rollout으로 지연 고려 가능 |
| discontinuous 전환 | 대응 어려움 | 여러 후보 중 안정적 궤적 선택 |

스워브 드라이브는 이론상 홀로노믹이지만 실제로:
- 모드 전환 시 steering re-alignment이 필요
- jerk/acceleration/velocity limiting이 걸림
- 특정 전환에서 불연속 정지 후 재시작

이런 특성에 MPPI의 rollout 기반 최적화가 더 적합하다.

## MPPI 핵심 튜닝

### Body Heading 정렬 (가장 중요)

스워브는 바퀴가 독립 회전하므로 body heading과 이동 방향이 무관할 수 있다. crab-walking을 방지하려면:

```yaml
# vy를 강하게 제한 (crab-walk 방지)
vy_max: 0.15          # 횡방향은 미세 조정용만 허용

# 전진 방향 선호 critic
PreferForwardCritic:
  cost_weight: 15.0    # 높을수록 전진 선호 강함

# 경로 방향과 body 각도 정렬
PathAngleCritic:
  cost_weight: 20.0    # 높을수록 body 회전하여 경로 방향 정렬

# 불필요한 회전 억제
TwirlingCritic:
  cost_weight: 15.0
```

### 적응형 속도 (직선 가속 / 커브 감속)

```yaml
vx_max: 1.5           # 직선에서 최대 속도 허용
temperature: 0.2       # 낮을수록 최적 궤적에 집중

# PathAngleCritic(20.0)이 커브에서 고속 궤적에 자동 페널티
# → 커브 진입 시 MPPI가 자연스럽게 낮은 속도 궤적을 선택
```

### Rollout 파라미터

```yaml
time_steps: 56         # rollout 스텝 수
model_dt: 0.05         # 스텝당 시간 → 총 2.8초 미래 예측
batch_size: 2000       # 후보 궤적 수
vx_std: 0.3            # 속도 샘플링 표준편차 (넓을수록 다양한 속도 탐색)
```

## Costmap 튜닝

### Inflation — 스워브 특화 고려사항

```yaml
inflation_radius: 0.75         # 로봇 inscribed radius(0.30m) + 0.45m 여유
cost_scaling_factor: 1.5       # 낮을수록 장애물에서 멀리까지 비용 유지
```

왜 일반 diff-drive보다 inflation이 넉넉해야 하는가:
- 스워브는 steering 전환 중 약간의 오버슈트 가능
- re-alignment 동안 정확한 궤적 추종이 안 됨
- 충분한 여유가 없으면 벽 근처에서 충돌

### Obstacle Min Range

```yaml
obstacle_min_range: 0.25       # 라이다 self-hit 데이터 필터링
raytrace_min_range: 0.25       # 라이다가 본체에 맞는 ~0.22m 이하 제거
```

2D LiDAR가 로봇 본체 내부(x=±0.32m)에 위치하여 일부 레이가 body mesh에 맞는다. `obstacle_min_range`로 이 데이터를 필터링.

## EKF 튜닝

### 충돌 시 odom 보호

```yaml
odom0_rejection_threshold: 2.0   # Mahalanobis distance 기반 outlier 거부
```

벽 충돌 시 바퀴 슬립 → wheel velocity 스파이크 → 이 threshold로 자동 무시.

### 퓨전 구성

```
Wheel Odom (/odom):   vx, vy, vyaw  ← 선속도 + 각속도
IMU:                  yaw, vyaw     ← 절대 heading + 각속도
```

스워브의 vy(횡속도)를 EKF에 퓨전하는 것이 diff-drive와의 차이. diff-drive는 vy=0 가정이지만 스워브는 실제 vy가 존재.

## Swerve Controller Sim 전용 파라미터

| 파라미터 | 실제 HW | Sim | 이유 |
|----------|---------|-----|------|
| `non_coaxial_ik_iterations` | 0 | 2 | Gazebo가 55mm 오프셋 오차를 노출 |
| `enable_steering_scrub_compensator` | true | false | Gazebo에 실제 scrub 없음 |
| `velocity_rolling_window_size` | 1 | 5 | sim encoder 노이즈 평활화 |
| `odom_integration_method` | rk4 | analytic_swerve | piecewise-constant에 정확 |
| `enable_odom_tf` | true | true* | standalone sim용, Nav2 실행 시 비활성화 |

*Nav2 launch가 3초 후 `ros2 param set`으로 false 전환.

## 튜닝 체크리스트

- [ ] RViz에서 local costmap에 장애물이 보이는가?
- [ ] 로봇이 벽 근처에서 충분한 거리를 유지하는가?
- [ ] 직선 구간에서 가속하고 커브에서 감속하는가?
- [ ] body heading이 이동 방향과 정렬되는가?
- [ ] 벽 충돌 후 odom이 안정적인가?
- [ ] 웨이포인트 순회 시 progress checker가 fail하지 않는가?
