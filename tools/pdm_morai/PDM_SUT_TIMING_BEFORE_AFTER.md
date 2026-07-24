# PDM-MORAI SUT 시간 구조 변경: 기존 vs 현재

## 변경 목적

기존 구현은 SIM 내부 물리 적분 간격인 `physics_delta_time`을 PDM/SUT 외부 제어기의
계산 `dt`로도 사용했다. SIM frame 주기와 physics substep 주기가 다르면 PDM 호출
주기와 PDM 내부 수치 계산 시간이 불일치할 수 있었다.

현재 구현은 두 시간축을 분리한다.

- PDM/SUT 외부 제어 step: `--pdm-step-delta-time`
- SIM 내부 physics 적분 step: `--physics-delta-time`

## 기존 구조

기존 fixed-step 계산:

```text
control_hz = --fixed-target-fps
pdm_dt     = --physics-delta-time / 1000
sim_dt     = --physics-delta-time
```

예:

```powershell
--fixed-target-fps 50 `
--physics-delta-time 10
```

결과:

```text
PDM 호출 주기 의미 = 50 Hz = 20 ms
PDM 계산 dt         = 10 ms
SIM physics dt      = 10 ms
```

PDM은 50 Hz로 호출되지만 rate limit과 kinematic forecast는 호출마다 10 ms가
경과했다고 계산하는 불일치가 발생했다.

```text
--fixed-target-fps ───────────────→ PDM/SUT target FPS

--physics-delta-time ─────────────→ SIM physics dt
                    └─────────────→ PDM config.time_step
```

문제점:

- SIM 내부 적분 간격과 PDM 외부 update 주기를 같은 값으로 간주했다.
- `target_fps`와 PDM `time_step`이 역수 관계인지 검증하지 않았다.
- SIM physics 설정 변경이 PDM controller 특성까지 변경했다.

## 현재 구조

현재는 `--pdm-step-delta-time`을 PDM/SUT 외부 시간축의 기준으로 사용한다.

```text
pdm_dt     = --pdm-step-delta-time / 1000
control_hz = 1 / pdm_dt
sim_dt     = --physics-delta-time
```

예:

```powershell
--fixed-step `
--pdm-step-delta-time 20 `
--physics-delta-time 10
```

결과:

```text
PDM/SUT dt          = 20 ms
PDM/SUT target FPS  = 50 Hz
SIM physics dt      = 10 ms
```

```text
--pdm-step-delta-time ────────────→ PDM config.time_step
                      └───────────→ PDM/SUT target FPS

--physics-delta-time ─────────────→ SIM internal physics dt
```

SIM 내부에서 10 ms 적분을 몇 회 수행하는지는 SIM이 관리한다. PDM/MORAI SUT는
외부 제어 step을 20 ms로 유지한다.

## 인자 우선순위와 호환성

### `--pdm-step-delta-time` 지정

이 값이 PDM/SUT 시간축의 단일 기준이다.

```text
20 ms → 50 Hz
10 ms → 100 Hz
25 ms → 40 Hz
50 ms → 20 Hz
```

정확한 정수 FPS로 변환할 수 없는 값은 오류로 차단한다.

```text
15 ms → 66.666... Hz → 오류
```

### `--pdm-step-delta-time` 생략

기존 CLI 호환을 위해 다음 규칙을 사용한다.

```text
fixed-step:
  control_hz = --fixed-target-fps
  pdm_dt     = 1 / control_hz

realtime:
  control_hz = suite control_hz
  pdm_dt     = 1 / control_hz
```

생략 시에도 `physics_delta_time`으로 PDM `dt`를 계산하지 않는다.

## PDM에서 `dt`가 적용되는 부분

`PdmLiteMoraiConfig.time_step`은 다음 계산에 사용된다.

- 목표속도 증가·감소 rate limit
- steering rate limit
- throttle 및 brake rate limit
- kinematic bicycle forecast
- realtime 실행 루프 period

예를 들어 throttle 증가율이 `1.6/s`이고 PDM step이 20 ms이면:

```text
step당 최대 throttle 증가량 = 1.6 × 0.020 = 0.032
```

PDM step이 10 ms이면:

```text
step당 최대 throttle 증가량 = 1.6 × 0.010 = 0.016
```

## TCP fixed-step 설정

Fixed mode 설정 시 SIM에는 다음 값이 전달된다.

```text
target_fps         = PDM/SUT step에서 계산한 FPS
physics_delta_time = --physics-delta-time
rtf                = --rtf
user_control       = --user-control
```

예:

```text
pdm_step_delta_time = 20 ms
physics_delta_time  = 10 ms

TCP SetSimulationTimeMode:
  target_fps         = 50
  physics_delta_time = 10
```

설정 후 `GetSimulationTimeStatus`로 SIM 적용값을 검증한다.

## 실행 예

### PDM 20 ms, SIM physics 20 ms

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\cto-1467-0\cto-1467-0.msuite" `
  --load-suite `
  --scenario-play `
  --fixed-step `
  --tcp-port 20000 `
  --pdm-step-delta-time 20 `
  --physics-delta-time 20
```

### PDM 20 ms, SIM physics 10 ms

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\cto-1467-0\cto-1467-0.msuite" `
  --load-suite `
  --scenario-play `
  --fixed-step `
  --tcp-port 20000 `
  --pdm-step-delta-time 20 `
  --physics-delta-time 10
```

두 실행 모두 PDM/SUT 출력 주기는 20 ms, 50 Hz이다. 두 번째 실행에서만 SIM 내부
physics 적분 간격이 10 ms이다.

## 실행 로그

실행 시작 시 실제 적용 예정 값이 출력된다.

```text
timing_config: pdm_step_delta_ms=20 pdm_target_fps=50 sim_physics_delta_ms=10
```

## 변경 전후 비교

| 항목 | 기존 | 현재 |
|---|---|---|
| PDM `dt` 기준 | `physics_delta_time` | `pdm_step_delta_time` |
| PDM target FPS | `fixed_target_fps` | `1000 / pdm_step_delta_time_ms` |
| SIM physics dt | `physics_delta_time` | `physics_delta_time` |
| SIM physics 변경이 PDM dt에 미치는 영향 | 직접 변경 | 영향 없음 |
| FPS와 PDM dt 일관성 | 보장하지 않음 | 하나의 인자에서 함께 계산 |
| 기존 CLI 호환 | 해당 없음 | `fixed_target_fps` 역수 사용 |
| 비정수 FPS step | 허용 가능 | 오류로 차단 |

## 현재 보장 범위

이번 변경이 보장하는 것:

- PDM/SUT 계산 `dt`
- PDM/SUT target FPS
- controller rate limit `dt`
- kinematic forecast `dt`

이번 변경 범위가 아닌 것:

- UDP command의 정확한 SIM 적용 tick
- UDP와 TCP 사이의 처리 순서
- SIM 내부 physics substep 횟수
- Vehicle Info UDP frame delivery timing

이번 변경은 통신 인과관계가 아니라 PDM/MORAI SUT 자체의 시간 인자 일관성을
확보한다.
