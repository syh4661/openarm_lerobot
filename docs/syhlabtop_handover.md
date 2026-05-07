# Syhlabtop Real Robot Handover
**Date**: 2026-05-07  
**From**: syhai PC (GPU/Isaac)  
**To**: syhlabtop@10.252.216.81 (real OpenArm robot)  
**Deadline**: 2026-05-14 (7 days)

---

## Mission

**Goal**: Quest VR 양팔 텔레오퍼레이션으로 "박스 잡고 박스 안에 캔 넣기" 데모를 수집하고 **π0.5(pi0.5)** 로 학습해서 자율 수행.

**Pipeline**:
```
Quest VR → QuestSpatialTeleop → SafeOpenArmFollower → 실물 OpenArm
                                                           ↓
                                              LeRobot HDF5 데이터 수집 (카메라 포함)
                                                           ↓
                                                  pi0.5 파인튜닝 → 배포
```

---

## 현재 상태 (2026-05-07 기준)

### 완료된 것
- `openarm_lerobot` master, `openarm_isaac_lab` main 모두 최신 푸시 완료
- `__init__.py` lazy import 수정 (`import can` 즉시 로드 버그 제거)
- Isaac Sim Task 4 통과 (환경 설정 검증 완료)

### 핵심 미해결 문제
**IK / 좌표 변환 불일치** — 이전 실물 테스트에서 팔이 위험하게 움직임. 전체적으로 좌표계가 맞지 않았음.

**2026-05-08 bus 매핑 정정**: LED ENABLE/DISABLE 직접 테스트로 `can0 = 물리 좌팔`, `can1 = 물리 우팔`이 확정됨 (`/tmp/bus_arm_mapping.md`, `/tmp/bus_arm_decision.md`). 양팔 모두 motor ID `0x01..0x08`을 공유하므로 arm 구분은 CAN 인터페이스 번호로만 이뤄진다. 최종 목표 매핑은 왼손 Quest 컨트롤러 → 좌팔(`can0`), 오른손 Quest 컨트롤러 → 우팔(`can1`)이다.

원인: `configs/record_quest_right_nocam.json`의 `coord_transform_vec: [-2.0, -1.0, -3.0, 4.0]`이 Quest 컨트롤러 공간 → OpenArm EE 공간 매핑을 담당하는데, 실제 설치 방향과 맞지 않음. bus-map 발견 전 튜닝은 오른손 컨트롤러가 물리 좌팔(`can0`)을 움직인 결과였으므로, 최종 좌/우 컨트롤러 매핑은 각각 다시 no-send axis sweep가 필요하다.

`coord_transform_vec` 의미 (`src/openarm_lerobot/quest_teleop.py:401`):
- 4개 값 각각 = 출력 x,y,z,w에 매핑할 입력 축 (1=x, 2=y, 3=z, 4=gripper)
- 부호 = 방향 반전 여부
- 현재값 `[-2,-1,-3,4]` → Quest Y→-Robot X, Quest X→-Robot Y, Quest Z→-Robot Z

---

## 즉시 할 것 (syhlabtop 접속 후)

### Step 0: 환경 준비
```bash
cd ~/workspace/openarm_lerobot
git pull origin master

# CAN 인터페이스 확인
ip link show | grep can

# Python 환경 확인
python -c "import openarm_lerobot; print('ok')"
```

### Step 1: IK 출력 확인 (로봇 안 움직임)
```bash
python scripts/record_quest_closed_loop.py \
  --config configs/record_quest_right_nocam.json \
  --no-send-action \
  --dry-run
```
Quest 컨트롤러를 천천히 움직이면서 로그에서 joint position 커맨드가 합리적인지 확인.  
관절값이 `joint_limits` 범위 내에서 의도한 방향으로 변하는지 체크.

### Step 2: coord_transform_vec 튜닝
Quest를 한 축씩 움직여보며 로봇 EE 방향 확인. **한 번에 하나씩만 수정**.

| 증상 | 수정 |
|------|------|
| 앞/뒤가 반대 | 해당 축 부호 반전 |
| 축이 바뀜 | 숫자 순서 교체 |
| 스케일 너무 큼 | `spatial_scale` 줄이기 (0.5 → 0.3) |

### Step 3: 실물 소폭 테스트
`--no-send-action` 제거, `spatial_scale: 0.3`, `max_ee_step_m: 0.03`으로 낮추고:
```bash
python scripts/record_quest_closed_loop.py \
  --config configs/record_quest_right_nocam.json \
  --control-time-s 10
```

---

## 7일 로드맵

| 날짜 | 목표 |
|------|------|
| Day 1 (5/7-8) | `--no-send-action`으로 IK 확인, `coord_transform_vec` 튜닝 |
| Day 2 (5/8-9) | 좌/우 단일팔 최종 매핑 no-send 재검증, 안전 검증 |
| Day 3 (5/9-10) | 양팔 Quest 설정 생성 + 양팔 `--no-send-action` 확인 |
| Day 4-5 (5/10-11) | 박스+캔 태스크 데모 수집 — **카메라 포함** (pi0.5 입력 필요) |
| Day 5-6 (5/11-12) | pi0.5 파인튜닝 (GPU: syhai PC 또는 클라우드) |
| Day 6-7 (5/12-14) | 배포 + 태스크 실행 검증 |

---

## 주요 파일

| 파일 | 용도 |
|------|------|
| `configs/record_quest_left_nocam.json` | 왼손 컨트롤러 → 물리 좌팔(can0) Quest 설정. legacy calibration id 유지 |
| `configs/record_quest_right_nocam.json` | 오른손 컨트롤러 → 물리 우팔(can1) Quest 설정. no-send 재검증 필요 |
| `configs/record_full.json` | 양팔 + 카메라 설정 (CAN 포트, 카메라 시리얼 참고) |
| `scripts/record_quest_closed_loop.py` | 메인 녹화 스크립트 |
| `scripts/debug_quest_input_only.py` | Quest 입력 디버그 (로봇 없이) |
| `src/openarm_lerobot/quest_teleop.py:401` | `_coord_vec_to_matrix` 변환 로직 |
| `src/openarm_lerobot/safe_followers.py` | 안전 셧다운 포함 follower |

## 양팔 Quest 설정 (아직 없음, 만들어야 함)

`configs/record_quest_bimanual_nocam.json` 신규 생성 필요.  
- robot 섹션: `record_full.json`의 양팔 CAN 포트 구조 참고  
- teleop 섹션: `record_quest_right_nocam.json`의 Quest 구조 참고  
- 왼팔 `coord_transform_vec`는 오른팔과 다를 수 있음 (미러 관계 가능성)

## pi0.5 데이터 수집 시 주의사항

pi0.5는 **비전 입력 필수**. `nocam` 설정으로 수집한 데이터는 학습에 사용 불가.  
데이터 수집 시 `record_full.json` 기반으로 카메라 활성화 필요:
- right wrist: RealSense `230322273311`
- left wrist: RealSense `315122270766`  
- chest: RealSense `234322070493`

학습 커맨드 (LeRobot + pi0.5):
```bash
# syhai GPU PC에서 실행
lerobot train \
  --policy.type=pi0 \
  --dataset.repo_id=local/openarm_bimanual_box_can \
  --output_dir=outputs/pi0_box_can
```

---

## 안전 체크리스트 (실물 테스트 전)

- [ ] `can0` (left), `can1` (right) 인터페이스 UP 상태 확인
- [ ] 로봇 주변 장애물 제거
- [ ] 비상 정지 수단 손 닿는 곳에
- [ ] 첫 실물 테스트는 `spatial_scale: 0.3` 이하
- [ ] `max_ee_step_m: 0.03` (기본 0.05에서 줄이기)

---

## 에이전트 지시사항

1. **먼저**: Step 0-1 실행해서 IK 출력이 합리적인지 확인
2. **coord_transform_vec 변경 시**: 한 번에 한 축씩만 수정 후 테스트
3. **실물 움직임 전**: 반드시 `--no-send-action`으로 먼저 검증
4. **양팔 설정**: 좌팔(can0) 단일 설정 완전 안정화 후 시작. 우팔(can1)은 별도 axis sweep 필요
5. **데이터 수집**: 카메라 반드시 포함, 에피소드당 20초, 태스크 완성 에피소드만 저장
6. **학습**: syhai GPU PC (10.252.216.81에서 SSH 역방향 또는 직접 접근)에서 pi0.5 파인튜닝
