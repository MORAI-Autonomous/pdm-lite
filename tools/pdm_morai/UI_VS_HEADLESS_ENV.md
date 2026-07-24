# PDM-MORAI UI / Headless 실행 요약

현재 기준은 두 가지 실행 흐름이다.

- UI SIM: 사용자가 MORAI UI에서 suite를 열고 Play를 누른 뒤 controller를 실행한다.
- Headless SIM: controller가 TCP로 suite load를 보내고, `ActiveSuiteStatus`로 load 완료를 확인한 뒤 Play를 보낸다.

제어 루프는 두 모드 모두 UDP를 사용한다. TCP는 suite load, active status 확인, scenario play, fixed-step 같은 management 용도로만 사용한다.

이미 suite와 scenario가 실행 중이면 controller를 attach-only SUT로 실행할 수 있다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --save-data-each-step `
  --status-hz 5
```

이 명령은 suite load와 Scenario Play를 수행하지 않는다. 첫 Vehicle Info Ego 상태를
받으면 local `KATRI` MGeo에서 route를 생성하고 UDP Manual Control을 시작한다.

## 1. Suite 생성

단일 XOSC:

```powershell
.\tools\run_pdm_morai_xosc.ps1 `
  --xosc "data\PDM-Lite_Test_Suite_3\Untitled Project\CCRb_Ego_0.xosc" `
  --gui `
  --output-root "data\generated-suite"
```

XOSC 폴더 전체:

```powershell
.\tools\run_pdm_morai_xosc.ps1 `
  --xosc "data\PDM-Lite_Test_Suite_3\Untitled Project" `
  --gui `
  --output-root "data\generated-suite"
```

Headless용 suite 생성도 같은 구조를 쓴다. 차이는 다음 실행 명령 안내만 headless용으로 출력한다.

```powershell
.\tools\run_pdm_morai_xosc.ps1 `
  --xosc "data\PDM-Lite_Test_Suite_3\Untitled Project\CCRb_Ego_0.xosc" `
  --headless `
  --output-root "data\generated-suite"
```

파일 생성 없이 확인:

```powershell
.\tools\run_pdm_morai_xosc.ps1 `
  --xosc "data\PDM-Lite_Test_Suite_3\Untitled Project" `
  --gui `
  --dry-run
```
<!-- 
## 2. UI SIM 실행

1. MORAI UI에서 generated `.msuite`를 연다.
2. map과 entity가 보이는지 확인한다.
3. Network Interface에서 `Vehicle Info Ego` SEND, `Manual Control` RECEIVE가 활성화되어 있는지 확인한다.
4. UI에서 Play를 누른다.
5. controller를 실행한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --status-hz 2
``` -->

## 3. Headless SIM 실행

Headless SIM을 먼저 띄운다. 그 뒤 controller가 TCP로 suite load, active suite status 확인, scenario play를 수행한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --headless `
  --tcp-port 20000 `
  --status-hz 2
```

동작 순서:

```text
TCP LoadSuite
TCP ActiveSuiteStatus polling
active_suite_name 확인
TCP ScenarioControl Play
UDP Vehicle Info 수신 대기
UDP Manual Control 송신 시작
```

Load 확인 timeout을 늘릴 때:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --headless `
  --tcp-port 20000 `
  --headless-timeout 60 `
  --status-hz 2
```

## 4. Headless Fixed-step 실행

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --headless `
  --fixed-step `
  ----save-data-each-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --status-hz 2
```

## 5. 수신만 확인

제어를 보내지 않고 UDP 상태 수신만 확인한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --headless `
  --tcp-port 20000 `
  --no-send `
  --duration 10 `
  --status-hz 2
```

정상 로그:

```text
ego=Ego raw=... actors=... speed=... cmd=(thr=..., brk=..., steer=...)
```

비정상 로그:

```text
waiting for ego state... rx=0 chunks=0 invalid=0
```

이 경우 controller의 Ego 매핑 문제가 아니라 SIM에서 Vehicle Info UDP SEND가 controller까지 오지 않는 상태다.

## 6. 주요 옵션

| 옵션 | 의미 |
|---|---|
| `--suite` | 실행할 `.msuite` |
| `--headless` | TCP LoadSuite -> status 확인 -> Play 후 UDP 제어 |
| `--tcp-port` | MORAI TCP management port. 예: `20000` |
| `--headless-timeout` | active suite 확인 대기 시간 |
| `--fixed-step` | TCP FixedStep(1)로 tick 진행 |
| `--fixed-target-fps` | fixed-step FPS |
| `--physics-delta-time` | fixed-step delta time, ms |
| `--rx-port` | Vehicle Info UDP 수신 포트 |
| `--tx-port` | Manual Control UDP 송신 포트 |
| `--no-send` | 제어 계산만 하고 UDP 송신 안 함 |
| `--status-hz` | 상태 로그 출력 주기 |

## 7. 주의

- Headless에서 외부 SIM 프로세스를 자동 실행하는 기능은 아직 없다.
- `--run-headless`, `--headless-runner`, `--steps`는 아직 자동 실행용으로 사용하지 않는다.
- TCP LoadSuite/ScenarioPlay 성공이 Vehicle Info UDP 송신 시작을 보장하지는 않는다.
- UDP가 0이면 headless runtime의 Network Interface가 실제로 SEND 중인지 먼저 확인한다.
