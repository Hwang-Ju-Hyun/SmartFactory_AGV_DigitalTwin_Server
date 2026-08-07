# Current project status

Last verified: 2026-08-07

Implementation base: `e193f64` on `old-new-combined` plus the physical-demo working tree

Purpose: Windows, WSL, 새 Codex 세션 사이의 공용 handoff

이 문서는 “지금 실제로 어디까지 됐는가”의 단일 기준이다. 설계 문서나 과거 포트폴리오 글과 상태가 다르면 이 문서와 최신 코드를 먼저 확인한다.

## 상태 표기

| 상태 | 의미 |
|---|---|
| 검증 완료 | 코드 실행 또는 사용자의 실차 시험으로 결과를 확인함 |
| 구현됨·재검증 필요 | 코드가 있으나 이번 기준 시점에 전체 실행을 다시 확인하지 않음 |
| 진행 중 | 일부 계층만 완성됐거나 서로 결합되지 않음 |
| 계획 | 아직 구현 또는 시험하지 않음 |

## 소프트웨어 상태

| 영역 | 상태 | 근거/메모 |
|---|---|---|
| C++20 CMake 구성 | 검증 완료 | `Shared`, `AGV_Server`, `FakeRobot`, `AGV_Client` target |
| Server-authoritative world | 검증 완료 | 기본 TESTCASE0 AGV 4대와 `--physical-demo` 단일 AGV mode |
| WHCA* 계열 route planning | 검증 완료 | smoke test에서 route 생성과 후속 route 재전송 확인 |
| 시간 기반 node/edge/goal reservation | 검증 완료 | server/FakeRobot smoke flow에서 사용 |
| 실행 시점 occupancy 검사 | 검증 완료 | simulator/controller 실행 flow에서 사용 |
| Unity map/replication protocol | 구현됨·재검증 필요 | Unity 프로젝트 자체는 CMake tree에 없음 |
| RobotProtocol v1 | 검증 완료 | HELLO_ACK, ROUTE_COMMAND, STATUS/ARRIVED flow 확인 |
| FakeRobot | 검증 완료 | localhost에서 AGV 1로 연결해 여러 route와 arrival 확인 |
| 자동화된 test target | 계획 | 현재는 build와 FakeRobot smoke test 중심 |
| native Windows server build | 계획 아님 | POSIX socket 의존성 때문에 Linux/WSL이 기본 환경 |

## 최근 software 검증

### 2026-08-07 motor-disabled physical demo

사용자 실물 시험에서 WSL Server와 ESP32의 연결이 복구돼 다음 흐름을 확인했다.

- ESP32 `HELLO_ACK accepted=1 agvID=1 error=0`
- 기존 자동 mode의 Server route `[1, 5, 9]` 수신
- firmware가 exact `[1, 2]`가 아닌 route를 거절하고 `STBY=LOW`, `PWM=0` 유지

이 결과를 기준으로 Server에 `--physical-demo` runtime mode를 추가했다.

- node 1에 AGV 1대만 생성
- `IDLE_READY`와 TaskManager 자동 배차 비활성화
- AGV 1의 HELLO 뒤 RoutePlanner가 exact `[1, 2]`인지 검증
- node/edge/goal 예약과 master plan을 만든 뒤 `ROUTE_COMMAND` 전송
- `MissionPurpose::NONE` route 완료 시 새 자동 임무 없이 IDLE 전환
- strict route에서 예상 밖 `ARRIVED` 또는 execution blocked가 오면 일반 재계획을 금지하고 `CANCEL_ROUTE`와 논리 safe stop 수행
- remote `ARRIVED`의 node sequence를 RoutePlanner가 먼저 검증한 뒤에만 occupancy와 AGV current node 갱신
- VS Code CMake Tools의 `AGV_Server` 벌레 버튼은 `--physical-demo`를 자동 전달

WSL에서 CMake configure/build 후 FakeRobot smoke test를 수행했다. `HELLO_ACK accepted=1`, `ROUTE routeID=1 nodes=2`, `ARRIVED node=2`를 확인했고, 도착 뒤 추가 route나 occupancy assertion은 발생하지 않았다. AGV 2 요청은 `UNKNOWN_AGV`로 거절됐다. 의도적으로 잘못 보낸 `ARRIVED node=3`에는 `CANCEL_ROUTE`를 보냈고, 뒤늦은 두 번째 잘못된 ARRIVED도 새 route나 occupancy 변경 없이 무시한 다음 재접속한 정상 `[1, 2]` route가 완료됐다. 인자 없이 실행한 기본 `AutomaticFleet`도 AGV 4대와 기존 자동 route 흐름을 유지했다. 실제 ESP32와 Unity는 새 mode로 아직 재검증하지 않았다.

### 2026-08-07 ESP32 HELLO/HELLO_ACK 진단

WSL Server와 Windows ESP32 사이의 motor-disabled Phase 2A 연결을 진단했다.

- Server와 ESP32 기준 commit은 각각 `120a600`, `640891d`였다.
- HELLO 19 byte와 HELLO_ACK 21 byte의 packet ID, protocol version, payload field, little-endian framing이 일치했다.
- WSL에서 CMake configure/build가 성공했다.
- localhost FakeRobot은 `HELLO_ACK accepted=1`, `ROUTE_COMMAND`, `STATUS`, `ARRIVED` 흐름을 통과했다.
- Windows `portproxy`의 ESP32 frontend TCP 연결 두 개는 AGV_Server 시작 전에 생성됐고, 대응하는 WSL backend 연결과 Server의 accept 로그가 없었다.
- ESP32 firmware에는 HELLO_ACK timeout이 없어 frontend 연결만 살아 있으면 HELLO를 재전송하거나 TCP를 다시 연결하지 않는다.

따라서 관찰된 `[TCP] Connected, sending HELLO` 정지는 Server/ESP32 wire format 불일치가 아니라 Server listener보다 먼저 만들어진 stale proxy 연결로 판별했다. 이후 Server를 먼저 실행하고 ESP32를 reset해 실제 `HELLO_ACK accepted=1`까지 확인했다.

FakeRobot 장시간 경로 실행 중에는 AGV 1이 AGV 3이 점유한 node 7에 진입하며 `OccupancyProvider::OccupyNode()` assertion이 발생했다. HELLO와 별개의 자동 route/실행 점유 blocker로 분리해 후속 진단한다.

### 2026-08-06 기준 검증

2026-08-06 WSL/Linux 환경에서 다음을 실행했다.

```bash
cmake -S . -B build
cmake --build build -j2
./build/Server/AGV_Server
./build/Server/FakeRobot 127.0.0.1:6666 1
```

결과:

- `Shared`, `AGV_Server`, `FakeRobot`, `AGV_Client` build 성공
- server가 `Shared/MapData.json`의 node 12개, link 52개 load
- TESTCASE0의 AGV 4대 world 생성
- FakeRobot `HELLO_ACK accepted=1`
- server가 active route와 후속 `ROUTE_COMMAND` 전송
- FakeRobot이 `ARRIVED`를 반복 보고하고 server가 다음 route 생성

이 검증에는 Unity viewer와 실제 ESP32 차체가 포함되지 않았다.

## 실물 AGV 상태

다음 항목은 2026-08-04 사용자 실차 기록 기준이다.

| 영역 | 상태 |
|---|---|
| 양쪽 motor와 encoder | 검증 완료 |
| 배터리 독립 전원 | 검증 완료 |
| 약 15 cm, 30 cm 직진 | 검증 완료 |
| 시계/반시계 제자리 90도 회전 | 검증 완료 |
| `30 cm -> 시계 90도 -> 30 cm` L자 경로 | 검증 완료 |
| 140 RPM PID 시험 | 검증 완료 |
| 후진 | 계획/미검증 |
| encoder odometry `x/y/heading` | 계획 |
| Wi-Fi 웹 조종 | 계획 |
| 실차와 서버의 end-to-end route 실행 | 진행 전 |
| 실차 pose의 Unity Digital Twin 반영 | 진행 전 |

## 반드시 구분할 두 ESP32 코드 계열

현재 두 계열이 아직 합쳐지지 않았다.

1. 실차 주행 검증본
   - motor, encoder, 정지 조건, L자 경로가 실차에서 검증됐다.
   - Wi-Fi와 RobotProtocol은 없다.
   - 기록에서 언급된 `AGV_Project_Record/final_l_route_main.cpp`는 현재 이 Git 저장소에 없다.

2. 저장소의 과거 network firmware 스냅샷
   - RobotProtocol의 HELLO/ROUTE/STATUS/ARRIVED 골격이 있다.
   - motor output이 비활성화돼 있고 encoder 대신 시간으로 route progress를 흉내 낸다.
   - `esp32.txt`, `agvallcode.txt` 형태라 독립적으로 build 가능한 firmware project가 아니다.

따라서 “ESP32 서버 연결”과 “실제 바퀴 구동”이 각각 존재한다고 해서 실제 서버 route로 차체가 주행한다고 표현하면 안 된다.

## 알려진 위험과 blocker

- `agvallcode.txt`에 실제 Wi-Fi 인증정보가 Git 추적 상태로 남아 있다. 값은 재사용하거나 문서에 옮기지 않는다.
- 2026-08-06 확인 결과 GitHub 원격 저장소는 public이다. 해당 credential은 이미 노출된 것으로 취급하고 공유기에서 즉시 변경해야 한다.
- 비밀번호를 변경한 뒤 현재 파일을 placeholder로 바꾸고 Git 과거 이력에서도 제거해야 한다. 정리가 끝나기 전 새 commit을 원격에 push하지 않는다.
- 과거 network firmware의 오른쪽 motor 방향 GPIO 순서가 실차 검증값과 반대다. 출력 활성화 전 `BIN1=33`, `BIN2=32`로 통일하고 바퀴를 띄워 방향을 다시 확인한다.
- 과거 문서 일부는 차체 도착 전 상태다. 현재 상태 판단에는 이 문서를 우선한다.
- `Shared/build/` 생성물이 과거 Git에 추적돼 있으나 현재 source of truth가 아니다.
- server reconnect cleanup, robot offline timeout과 실제 emergency-stop 송신 경로를 보강해야 한다.
- physical demo도 같은 AGV ID의 동시/교체 session을 아직 명시적으로 거절하지 않는다. 실차 시험 중 AGV 1로 FakeRobot을 함께 연결하지 않는다.

## 다음 우선 작업

1. `--physical-demo` Server를 먼저 실행하고 ESP32를 reset해 exact `[1, 2]` 수신과 motor lock 유지 확인
2. 같은 mode에 Unity viewer를 연결해 AGV 1대가 node 1에 보이는지 확인
3. ESP32에 HELLO_ACK timeout과 stale TCP reconnect를 추가
4. 노출된 Wi-Fi 비밀번호 변경과 저장소 secret 정리
5. 전원 분배단자, fuse, switch와 기판 고정
6. 후진 15 cm 시험으로 양쪽 encoder 부호 확정
7. 실차 GPIO와 모든 local safety를 network firmware에 옮기되 motor output은 끈 상태로 통신 검증
8. 바퀴를 띄운 상태에서 HELLO/STATUS/CANCEL/ESTOP 검증
9. odometry와 route executor를 추가하고 시간 기반 progress 제거
10. 낮은 속도로 단일 route를 실행한 뒤 Unity 오차 측정
11. FakeRobot에서 드러난 자동 fleet node 7 occupancy collision assertion을 별도 Server 작업으로 진단

상세 순서는 [physical-agv-integration.md](physical-agv-integration.md)에 있다.

## 환경 간 handoff 규칙

작업을 시작할 때:

```bash
git status --short --branch
git pull --ff-only
```

작업을 끝낼 때 다음 중 실제로 바뀐 내용만 이 문서에 반영한다.

- 마지막으로 검증한 명령/시험
- 새로 완료된 milestone
- 발견한 blocker 또는 안전 위험
- 다음 사람이 수행할 단일 우선 작업

그 뒤 변경 파일을 명시적으로 stage하고 commit/push한다. Codex 대화에만 남긴 결정은 다른 환경에 전달되지 않는다. 두 clone을 동시에 쓸 때의 branch 규칙은 [development-workflow.md](development-workflow.md)를 따른다.
