# PDM-MORAI 작업 고찰 - 2026-06-15

## 1. 기존에 `--suite`를 명시했던 이유

초기 PDM-MORAI runner에서 suite는 단순히 SIM에 로드할 파일이 아니었다.
runner는 suite와 sidecar 파일에서 다음 정보를 함께 얻었다.

- Ego/NPC logical ID와 record 순서
- Vehicle Info UDP와 Manual Control UDP endpoint
- XOSC Ego 초기 pose와 초기 속도
- map/MGeo 및 route 생성 기준
- Manual Control 대상 entity
- headless 환경의 LoadSuite와 ScenarioControl 대상 이름

따라서 당시에는 suite를 생략하면 실행에 필요한 바인딩 정보까지 함께 사라졌다.
UI와 headless 실행을 하나의 runner가 모두 소유하던 구조에서는 `--suite` 필수화가
가장 단순하고 안전한 선택이었다.

## 2. 준비 단계와 SUT 실행을 분리한 이유

coverage 또는 UI 환경에서는 외부 시스템이나 사용자가 이미 suite를 로드하고
scenario를 Play할 수 있다. 이때 PDM-MORAI가 다시 suite를 생성하거나 로드하면
scenario lifecycle 충돌, 중복 Play, UDP interface 재초기화 문제가 발생할 수 있다.

현재 구조는 책임을 다음과 같이 나눈다.

| 구분 | 책임 |
|---|---|
| XOSC/suite 준비 | XOSC 파싱, Ego/NPC 매핑, msuite/project/interface 생성 |
| TCP management | 선택적으로 LoadSuite, Play/Stop/Pause, fixed-step |
| SUT runtime | Vehicle Info UDP 수신, PDM 계산, Manual Control UDP 송신 |

`--suite`가 없으면 `active_sim_attach`, suite만 제공하면
`suite_metadata_attach`, `--load-suite` 또는 `--headless`를 사용하면
`standalone_load`로 동작한다.

## 3. 분리 후에도 suite 명시가 유효한 경우

attach-only는 suite load가 필요 없다는 뜻이지 suite metadata가 항상 불필요하다는
뜻은 아니다. 다음 조건에서는 `--suite` 또는 `--entity-ids`가 더 안전하다.

- SIM UDP raw id가 실제 XOSC entity 이름과 다를 때
- 여러 actor가 동일한 raw id로 전송될 때
- 기본 포트 `9097/9091`이 아닌 suite를 사용할 때
- XOSC actor bbox, 초기 pose, target speed를 사용해야 할 때
- suite route 또는 profile의 logical ID 순서를 강제해야 할 때

외부 시스템이 scenario lifecycle을 소유하되 metadata만 활용하려면 suite를 지정하고
`--load-suite`, `--scenario-play`를 생략한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --suite "data\generated-suite\CCRb_Ego_2\CCRb_Ego_2.msuite" `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --save-data-each-step `
  --status-hz 5
```

## 4. 오늘 확인한 Manual Control 미적용 원인

사용한 attach-only 명령은 TCP fixed-step과 UDP 송신 자체는 정상 동작했다.
문제는 SIM이 Ego와 NPC 4대의 Vehicle Info record를 모두 raw id `Car_1`로 보낸
것이었다.

기존 코드는 state dictionary의 key로 raw id를 사용했기 때문에 같은 프레임의
5개 record가 서로 덮어써졌다. 마지막 record인 NPC_4만 남았고 controller는 이를
Ego로 오인했다.

결과적으로:

```text
상태 입력: NPC_4
Manual Control 대상: Ego
```

가 되어 command 송신 count는 증가하지만 Ego의 applied control은 변하지 않았다.

## 5. 수정 사항

### 중복 raw id 보존

suite metadata가 없는 attach-only에서 한 UDP payload에 여러 record가 있고 raw id가
전부 같으면 generated suite의 Vehicle Info 순서 규칙을 사용한다.

```text
record 0 -> Ego
record 1 -> NPC_1
record 2 -> NPC_2
...
```

이 매핑은 다음 로그로 명시된다.

```text
runtime_id_mapping: duplicate raw ids detected; using record order Ego,NPC_1..
```

명시적 `--entity-ids` 또는 suite metadata가 있으면 해당 순서를 우선한다.

### UDP 중복 listener 차단

Vehicle Info 수신 socket의 `SO_REUSEADDR`를 제거했다. 이제 다른 controller가 이미
같은 UDP port를 점유하면 두 프로세스가 packet을 나눠 받지 않고 bind 오류로 즉시
종료한다.

### 검증 결과

실제 MORAI SIM `CCRb_Ego_2`, fixed 50 Hz, 20 ms 조건에서 확인했다.

- TCP FixedStep 정상 진행
- UDP payload당 actor 5대 유지
- Ego pose `(-2729.24, 1614.05)` 선택
- Manual Control applied throttle `0.10 -> 0.96`
- Ego speed 약 `0 -> 1.57 m/s`
- 단위 테스트 21개 통과

## 6. `--scenario-play` 동작 주의사항

`--scenario-play`는 현재 active scenario에 Play 명령을 보내지만 이미 Play 중인
scenario를 재시작하지 않는다. Play 세션은 살아 있으나 Vehicle Info SEND가 멈춘
경우 Play를 다시 보내는 것만으로 UDP interface가 재활성화되지 않을 수 있다.

이 경우 먼저 Stop한 뒤 controller를 다시 실행한다.

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --tcp-port 20000 `
  --scenario-control-only stop
```

그 다음:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --fixed-step `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20 `
  --save-data-each-step `
  --status-hz 5 `
  --scenario-play
```

## 7. 남은 한계와 권장 방향

- record 순서 기반 추론은 현재 generated suite 규칙에는 맞지만 protocol 수준의
  stable entity ID는 아니다.
- 장기적으로 SIM Vehicle Info가 실제 entity id를 보내거나 별도 object stream이
  stable ID를 제공해야 한다.
- 외부 coverage 환경에서는 가능한 경우 `--entity-ids` 또는 suite metadata를
  제공해 추론 의존도를 낮추는 것이 안전하다.
- traffic light, pedestrian, stop line, static obstacle 정보는 현재 Vehicle Info
  UDP만으로 구현할 수 없다.
- realtime control은 계속 UDP를 사용하고 TCP ManualControlById를 기본 경로로
  사용하지 않는다.
