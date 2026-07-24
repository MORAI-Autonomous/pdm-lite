# PDM-MORAI 실행 및 Suite 변환 매뉴얼

이 문서는 현재 정리된 PDM-MORAI UI SIM 기준 실행 절차를 설명한다. 현재 제어 루프는 UDP 기반이며, TCP는 suite load, scenario play, fixed-step 같은 저주파 management 용도로만 사용한다.

## 1. 전체 흐름

```text
XOSC 입력
  -> Ego/NPC/초기 pose/속도/map 정보 파싱
  -> generated suite/msuite/project/interface 생성
  -> MORAI UI SIM에서 suite load 또는 runner가 TCP LoadSuite 전송
  -> Scenario Play
  -> Vehicle Info UDP 수신
  -> PDM-MORAI controller가 Manual Control UDP 송신
```

## 2. XOSC를 Runnable Suite로 변환

### 단일 XOSC 변환

```powershell
.\tools\run_pdm_morai_xosc.ps1 `
  --xosc "data\PDM-Lite_Test_Suite_3\Untitled Project\CCRb_Ego_0.xosc" `
  --gui `
  --output-root "data\generated-suite"
```

### XOSC 폴더 전체 변환

```powershell
.\tools\run_pdm_morai_xosc.ps1 `
  --xosc "data\PDM-Lite_Test_Suite_3\Untitled Project" `
  --gui `
  --output-root "data\generated-suite"
```

### 파일 생성 없이 진단

```powershell
.\tools\run_pdm_morai_xosc.ps1 `
  --xosc "data\PDM-Lite_Test_Suite_3\Untitled Project" `
  --gui `
  --dry-run
```

Dry-run에서 확인할 항목:

```text
ego_entity
npc_entities
map / runner_map_name
initial_pose
initial_speed
planned_udp_vehicle_info
planned_manual_control
planned_logical_id_mapping
blocking_errors
```

## 3. Suite 변환 옵션

| 옵션 | 의미 |
|---|---|
| `--xosc` | 입력 XOSC 파일 또는 XOSC 폴더 |
| `--gui` | 현재 기본 검증 모드. UI SIM용 suite 생성 |
| `--headless` | headless-compatible suite 생성 및 headless controller 명령 안내 |
| `--output-root` | generated suite 출력 위치 |
| `--ego-entity` | Ego 자동 판별 실패 시 명시적으로 지정 |
| `--manual-control-entity` | Manual Control target override |
| `--rx-port` | Vehicle Info UDP 수신 포트. 기본 `9097` |
| `--tx-port` | Manual Control UDP 송신 포트. 기본 `9091` |
| `--feedback-hz` | Vehicle Info SEND 주기. 기본 `30` |
| `--control-hz` | Manual Control RECEIVE 기준 주기. 기본 `60` |
| `--dry-run` | 파일 생성 없이 파싱/매핑만 출력 |

## 4. 변환 결과물 구조

예시:

```text
data/generated-suite/CCRb_Ego_0/
  CCRb_Ego_0.msuite
  Untitled Project/
    Untitled Project.mproj
    CCRb_Ego_0.xosc
    Vehicle Info Ego.json
    Manual Control.json
    Vehicle Info.tmpl
    Vehicle Manual Control.tmpl
    MGeo/
```

핵심 매핑:

```text
ScenarioObject name="Ego" -> Ego/host vehicle
NPC_* -> NPC
Vehicle Info Ego.json -> UDP SEND, destination 127.0.0.1:9097
Manual Control.json -> UDP RECEIVE, host 127.0.0.1:9091
mproj interface + xosc metadata -> runner가 읽는 실행 바인딩
```

## 5. UI SIM에서 수동 Load/Play 후 Controller 실행

1. MORAI UI SIM에서 generated `.msuite`를 연다.
2. map과 entity가 표시되는지 확인한다.
3. Network Interface에서 `Vehicle Info Ego` SEND, `Manual Control` RECEIVE가 활성화되어 있는지 확인한다.
4. Scenario Play를 누른다.
5. 별도 PowerShell에서 controller를 실행한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --status-hz 2
```

### 이미 실행 중인 SIM에 SUT만 연결

GUI, coverage runner 또는 다른 프로세스가 suite load와 Scenario Play를 완료했다면
`--suite` 없이 controller만 연결할 수 있다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --pdm-step-delta-time 20 `
  --physics-delta-time 20 `
  --save-data-each-step `
  --status-hz 5
```

이 모드는 suite를 생성하거나 load하지 않고, Scenario Play도 보내지 않는다.
기본 UDP 포트는 Vehicle Info `9097`, Manual Control `9091`이며 Ego는 `Ego`,
그다음 `Car_1` 순서로 찾는다. 첫 Ego UDP pose를 받은 뒤
`data/morai/mgeo/KATRI`에서 nearest-link route를 생성한다.

다른 설정이 필요하면 다음 옵션을 명시한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --ego-id "HostVehicle" `
  --rx-port 9097 `
  --tx-port 9091 `
  --target-speed 10 `
  --mgeo-dir "data\morai\mgeo\KATRI" `
  --status-hz 5
```

시작 로그의 실행 모드는 다음과 같다.

```text
mode=active_sim_attach       # suite 없음, 현재 SIM에 연결
mode=suite_metadata_attach  # suite는 metadata/route 용도로만 읽고 load하지 않음
mode=standalone_load        # --load-suite 또는 --headless
```

### Attach-only의 actor ID 처리

일부 SIM 구성에서는 Ego와 NPC의 UDP raw id가 모두 `Car_1`로 전송될 수 있다.
suite metadata가 없는 상태에서 한 payload에 중복 raw id record가 여러 개 들어오면
runner는 generated suite의 Vehicle Info 순서 규칙에 따라 첫 record를 `Ego`,
이후 record를 `NPC_1`, `NPC_2` 순서로 보존한다.

```text
runtime_id_mapping: duplicate raw ids detected; using record order Ego,NPC_1..
```

외부 시스템이 정확한 logical ID 순서를 알고 있다면 직접 지정하는 편이 안전하다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --entity-ids "Ego,NPC_1,NPC_2,NPC_3,NPC_4" `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --status-hz 5
```

`--suite`를 제공하되 `--load-suite`를 생략하면 SIM lifecycle은 수정하지 않고
suite의 ID/route/interface metadata만 사용할 수 있다.

## 6. Runner가 Suite Load와 Play까지 보내는 실행

SIM TCP management port가 열려 있으면 runner가 suite load와 scenario play를 먼저 보낼 수 있다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --load-suite `
  --scenario-play `
  --tcp-port 20000 `
  --status-hz 2
```

동작 방식:

```text
TCP LoadSuite
TCP ScenarioControl Play
TCP close
UDP Vehicle Info 수신 대기
UDP Manual Control 송신 시작
```

주의:

```text
TCP LoadSuite / ScenarioPlay 성공이 Network Interface SEND 자동 시작을 보장하지 않는다.
Vehicle Info UDP가 0이면 SIM UI에서 Vehicle Info Ego SEND 상태를 확인해야 한다.
```

## 7. 상태 수신만 확인

제어 송신 없이 UDP 수신과 PDM 계산 전 상태를 확인한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --duration 10 `
  --status-hz 2 `
  --no-send
```

정상 로그 예:

```text
ego=Ego raw=Ego pos=(...) actors=...
speed=...mps
lead=...
cmd=(thr=..., brk=..., steer=...)
rx=... tx=...
```

비정상 로그:

```text
waiting for ego state... rx=0 chunks=0 invalid=0
```

이 경우는 controller의 Ego 매핑 문제가 아니라, SIM에서 Vehicle Info UDP가 controller까지 오지 않는 상태다.

## 8. UDP 포트 진단

기본 포트:

| 용도 | 포트 |
|---|---:|
| Manual Control RECEIVE | `9091` |
| Vehicle Info Ego SEND | `9097` |
| 검증용 상태 포트 후보 | `9001` |
| Step/time 관련 후보 | `9094` |
| Traffic light 후보 | `9095` |
| Ground entity 후보 | `9096` |

포트 override 예:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --rx-ip 0.0.0.0 `
  --rx-port 9097 `
  --status-hz 2 `
  --no-send
```

최근 진단 결론:

```text
127.0.0.1:9097, 0.0.0.0:9097, 9001, 9094, 9095, 9096 모두 rx=0이면
runner 문제가 아니라 SIM Network Interface SEND가 실제 송신되지 않는 상태로 판단한다.
```

## 9. Manual Control 단독 테스트

Vehicle Info 수신 없이 Manual Control UDP 송신만 확인한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --manual-test `
  --manual-throttle 0.10 `
  --manual-brake 0.00 `
  --manual-steer 0.00 `
  --duration 3
```

이 테스트가 성공해도 Vehicle Info 수신이 살아난 것은 아니다. 송신 제어와 상태 피드백은 별도 인터페이스다.

## 10. Fixed-step 실행

fixed-step은 재현성 검증용이다. realtime control은 여전히 UDP로 유지된다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --load-suite `
  --scenario-play `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --status-hz 2
```

30Hz 설정 예:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --load-suite `
  --scenario-play `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 30 `
  --physics-delta-time 33 `
  --status-hz 2
```

## 11. Controller 주요 옵션

| 옵션 | 의미 |
|---|---|
| `--suite` | 실행할 `.msuite` 또는 `.xosc` |
| `--duration` | 실행 시간. `0`이면 Ctrl+C까지 계속 |
| `--target-speed` | 목표 속도 override, 단위 m/s |
| `--ego-id` | logical Ego id override |
| `--entity-ids` | 수신 Vehicle Info record logical id 목록 |
| `--rx-ip`, `--rx-port` | Vehicle Info UDP 수신 endpoint |
| `--tx-ip`, `--tx-port` | Manual Control UDP 송신 endpoint |
| `--no-send` | 제어 계산만 하고 UDP 송신 안 함 |
| `--manual-test` | Vehicle Info 없이 고정 Manual Control 송신 |
| `--hold-until-play` | 첫 ego state 이후 sim timestamp가 변할 때까지 제어 hold |
| `--dry-run` | scenario/route만 로드하고 종료 |
| `--load-suite` | TCP로 suite load |
| `--scenario-play` | TCP로 scenario play |
| `--scenario-stop-only` | TCP로 active scenario Stop만 보내고 종료 |
| `--scenario-control-only` | TCP scenario menu 명령만 실행. `status`, `play`, `pause`, `stop` |
| `--scenario-name` | TCP ScenarioControl 대상 이름 override |
| `--no-stop-before-start` | load/play 전에 기존 active scenario Stop을 보내지 않음 |
| `--headless` | TCP LoadSuite -> ActiveSuiteStatus 확인 -> ScenarioPlay 후 UDP 제어 |
| `--headless-timeout` | headless active suite 확인 대기 시간 |
| `--udp-ready-timeout` | PDM 제어 시작 전 Ego Vehicle Info UDP 수신 대기 시간. `0`이면 비활성 |
| `--udp-ready-steps` | deprecated. UDP readiness는 `--udp-ready-timeout` 기준으로 대기 |
| `--tcp-port` | TCP management port. 예: `20000` |
| `--fixed-step` | TCP FixedStep(1)로 tick 진행 |
| `--fixed-target-fps` | fixed-step target FPS |
| `--physics-delta-time` | fixed-step delta time, ms |
| `--status-hz` | 상태 로그 출력 주기 |
| `--route-length` | Ego pose 기반 MGeo route 생성 길이 |
| `--route-points` | controller에 넘길 전방 route point 수 |
| `--rebuild-distance` | route 이탈 시 route 재생성 거리 |
| `--route-max-error` | 이 값 초과 시 neutral route guard |
| `--route-entity` | Ego AdvancedRouting만 허용. NPC route는 Ego route로 쓰지 않음 |

## 12. 현재 route 정책

Ego route는 XOSC에 명시되지 않는 경우가 일반적이므로 기본값은 다음과 같다.

```text
Ego initial WorldPosition
  -> local MGeo nearest link
  -> successor chain
  -> dense reference route
```

NPC `AdvancedRouting`은 NPC 자신의 주행용 시나리오 요소로 본다. Ego를 외부 Manual Control로 제어하는 PDM-MORAI 구조에서는 NPC AdvancedRouting을 Ego route로 자동 차용하지 않는다.

## 13. Map 정책

현재 local MGeo 기준은 `KATRI`로 통일한다.

UI SIM 표시용 map 이름은 source suite/project의 이름을 보존한다. 예를 들어 UI에서 필요한 map 이름이 `Hwaseong-si`이면 `.mproj`와 XOSC의 `moraiMap`은 해당 이름을 유지하고, runner 내부 MGeo lookup만 `KATRI` sidecar를 사용한다.

## 14. Headless 실행

Headless에서는 MORAI UI로 suite를 열거나 Play를 누를 수 없다. 따라서 controller가 TCP management API를 이용해 suite load, active status 확인, scenario play를 수행한 뒤 UDP 제어 루프로 들어간다.

Stop만 보내기:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --scenario-control-only stop `
  --tcp-port 20000
```

Scenario menu 직접 확인:

```powershell
.\tools\run_pdm_morai_control.ps1 --scenario-control-only status --tcp-port 20000
.\tools\run_pdm_morai_control.ps1 --scenario-control-only play   --tcp-port 20000
.\tools\run_pdm_morai_control.ps1 --scenario-control-only pause  --tcp-port 20000
.\tools\run_pdm_morai_control.ps1 --scenario-control-only stop   --tcp-port 20000
```

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --headless `
  --tcp-port 20000 `
  --status-hz 2
```

동작 순서:

```text
TCP ScenarioControl Stop으로 이전 Play 정리
TCP LoadSuite
TCP ActiveSuiteStatus polling
active_suite_name 확인
TCP ScenarioControl Play
UDP Vehicle Info Ego 수신 및 timestamp 전진 확인
UDP Manual Control 송신 시작
```

Load 확인 timeout 조정:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --headless `
  --tcp-port 20000 `
  --headless-timeout 60 `
  --status-hz 2
```

Headless fixed-step:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --headless `
  --save-data-each-step `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --status-hz 2
```

아직 외부 headless SIM 프로세스를 자동 실행하는 기능은 없다. `--run-headless`, `--headless-runner`, `--steps`는 자동 실행용 예약 옵션으로 남겨둔다.

## 15. 자주 보는 문제

### `load_suite`와 `scenario_play`는 성공했는데 `rx=0`

의미:

```text
SIM이 scenario는 play 중이지만 Vehicle Info UDP SEND가 송신되지 않음
```

확인:

```text
Network Interface -> Vehicle Info Ego -> SEND 활성화
destination port 9097
entityList에 Ego 포함
Manual Control RECEIVE는 별도 확인
```

이미 Play 중인 scenario에 `--scenario-play`를 다시 보내는 것은 scenario restart가
아니다. Play 상태인데 Vehicle Info SEND가 멈춰 있다면 먼저 Stop한 뒤 다시
controller를 실행한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --tcp-port 20000 `
  --scenario-control-only stop
```

### Headless import에서 BoundingBox/Axles 오류가 남

MORAI headless importer는 vehicle geometry validation이 엄격하다.

```text
ScenarioObject: Invalid Vehicle Bounding Box & Axles Value
FrontOverhang must be greater than 0
```

이 경우 원본 XOSC의 `BoundingBox Center.x`, `Dimensions length`, `FrontAxle positionX`, `RearAxle positionX` 조합이 MORAI 기준과 맞지 않는 것이다. 현재 suite 생성기는 generated XOSC에 한해 axle 기준으로 `BoundingBox Center.x`를 보정해서 front/rear overhang이 양수가 되도록 한다.

또한 Sun 각도는 MORAI가 radian을 기대하므로, degree처럼 보이는 `Sun azimuth/elevation` 값은 generated XOSC에서 radian으로 변환한다.

### Ego가 NPC처럼 보임

확인:

```text
XOSC ScenarioObject name="Ego"
Vehicle property AgentType=Ego
mproj vehicleAgentTypeList에서 Ego AgentType=1
Manual Control entityList target이 Ego
XOSC external_control entity 또는 Ego entity가 runner ego_id로 해석됨
```

### route가 이상하거나 시작 직후 조향이 튐

확인:

```text
dry-run start_route distance
dry-run heading_error
local MGeo가 최신 KATRI인지
XOSC initial pose가 실제 map lane 위인지
초기 속도가 과도하게 높지 않은지
```

권장 dry-run:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_0\CCRb_Ego_0.msuite" `
  --dry-run `
  --status-hz 2
```
## Runtime trace CSV

콘솔 상태 출력은 그대로 유지하면서 각 제어 업데이트를 SIM statelog 대응 CSV로 저장하려면
`--trace-log`를 추가한다. 지정한 파일은 실행 시작 시 덮어쓴다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_1\CCRb_Ego_1.msuite" `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --save-data-each-step `
  --status-hz 2 `
  --trace-log "logs\pdm_morai_trace.csv"
```

trace는 콘솔 문자열을 다시 파싱하지 않고 UDP Ego 상태를 직접 기록한다.
SIM statelog과 비교 가능한 time, position, velocity, acceleration, rotation,
applied throttle/brake/steer만 포함한다.
