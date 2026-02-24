# 결정: RL 정책 → unitree_rl_lab으로 전환

## 결정 요약

IsaacLab 공식 제공 Go2 정책에서 **unitree_rl_lab** 기반 정책으로 전환하기로 결정.

---

## 기존 방식 (IsaacLab 공식)

| 항목 | 내용 |
|------|------|
| 소스 | IsaacLab 서버 자동 다운로드 |
| 태스크 | `Isaac-Velocity-Rough-Unitree-Go2-v0` |
| 체크포인트 | `model.pt` (약 6.6MB, RSL-RL 포맷) |
| 프레임워크 | IsaacLab 내장 RSL-RL |
| 실로봇 배포 | ❌ 시뮬 전용 정책 (실로봇 배포 코드 없음) |

---

## 변경 방식 (unitree_rl_lab)

| 항목 | 내용 |
|------|------|
| 소스 | [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) (Unitree 공식) |
| 태스크 | `Unitree-Go2-Velocity` |
| 체크포인트 | RSL-RL 포맷 (동일 구조) |
| 프레임워크 | IsaacLab 2.3.0 + RSL-RL (기존 환경과 호환) |
| 실로봇 배포 | ✅ `deploy/robots/go2/` 배포 코드 포함 |

---

## 전환 이유

1. **실로봇 배포 목표**: Go2 실제 하드웨어에 Nav2 자율주행을 올리는 것이 최종 목표.
   - IsaacLab 공식 정책은 시뮬 검증용이며 실로봇 배포 파이프라인이 없음.
   - unitree_rl_lab은 `deploy/` 폴더에 실로봇 배포 코드가 포함되어 있음.

2. **동일 기술 스택**: RSL-RL 기반으로 기존 `go2_sim.py` 파이프라인 변경 최소화.

3. **Unitree 공식 지원**: 하드웨어 제조사 직접 제공 → Go2 관절/제어 파라미터 최적화됨.

---

## 전환 내용

### 설치
```bash
# conda lab 환경에서
cd /home/cvr/Desktop/sj/unitree_rl_lab
./unitree_rl_lab.sh -i
```

### USD 모델 경로 설정
- 파일: `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`
- 변경: `UNITREE_MODEL_DIR = "/home/cvr/Desktop/sj/unitree_model"`
- USD: `/home/cvr/Desktop/sj/unitree_model/Go2/usd/go2.usd`

### 학습 명령
```bash
cd /home/cvr/Desktop/sj/unitree_rl_lab
python scripts/rsl_rl/train.py --headless --task Unitree-Go2-Velocity
```

### 학습 재개 (중단 시)
```bash
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity \
  --headless \
  --resume \
  --load_run 2026-02-24_12-51-50 \
  --checkpoint model_37500.pt \
  --max_iterations 12500
```

### 검증 (play)
```bash
./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity
```

---

## 학습 결과 경로

```
/home/cvr/Desktop/sj/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/
└── 2026-02-24_12-51-50/
    ├── model_37500.pt   ← 현재 마지막 체크포인트
    └── model_50000.pt   ← 목표 (50,000 iterations)
```

---

## go2_sim.py 통합 계획

학습 완료 후 `go2_sim.py`의 체크포인트 경로를 변경:

```python
# 기존 (IsaacLab 공식)
# 자동 다운로드 방식

# 변경 후 (unitree_rl_lab)
runner.load_checkpoint("/home/cvr/Desktop/sj/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/<run_name>/model_50000.pt")
```

> 정책 파라미터 포맷이 RSL-RL로 동일하므로 로딩 코드 변경 최소화.

---

## 관련 문서

- `02_nav2_plan.md` — Nav2 자율주행 구현 계획 (Phase 6에서 실로봇 cmd_vel 연동)
