# Current project status

Last verified: 2026-09-02

Implementation base: Server `old-new-combined` 2026-08-25 working tree, ESP32 `668622cf`, Unity `b821d0c2`, VisionTracker `278cc431`

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
| C++20 CMake 구성 | 검증 완료 | `Shared`, `AGV_Server`, `FakeRobot`, `TrajectorySmokeTest`, `TrajectoryPreview`, `VisionObservationTest`, `PhysicalFleetCorrectionTest`, `AGV_Client` target |
| Server-authoritative world | 구현됨 | TESTCASE0 및 실제 AGV 1용 `--physical-fleet` |
| WHCA* 계열 route planning | 검증 완료 | smoke test에서 route 생성과 후속 route 재전송 확인 |
| 시간 기반 node/edge/goal reservation | 검증 완료 | server/FakeRobot smoke flow에서 사용 |
| 실행 시점 occupancy 검사 | 검증 완료 | simulator/controller 실행 flow에서 사용 |
| Unity map/replication protocol | 검증 완료 | physical-demo에서 AGV 1 생성과 실차 진행/도착 표시를 사용자 시험으로 확인 |
| RobotProtocol v1 | 검증 완료 | HELLO_ACK, ROUTE_COMMAND, STATUS/ARRIVED flow 확인 |
| metric trajectory 기반 | 진행 중 | 60 mm/unit `[1 -> 4]` preview와 ESP32 motor-disabled follower trace 통과 |
| FakeRobot | 검증 완료 | localhost에서 AGV 1로 연결해 여러 route와 arrival 확인 |
| Vision 관측 수신 기반 | 카메라 검증 완료·Server 재시작 필요 | 기준/AGV 태그 높이 140 mm, 측정된 `[0,0]` body offset의 pose contract `f84eb43ebb6cf7ff`, calibration `f1766a6f3a2d9a6d`; 카메라 재조정 후 Node 1 camera-only 오차 약 1.9 mm/0.2도, 새 HELLO 승인은 Server 재시작 뒤 확인 필요 |
| Vision 관측 Unity 중계 | wire E2E 검증 완료·실화면 검증 필요 | 별도 packet type 6, mm→map unit/radian 변환, 500 ms timeout LOST; authoritative pose와 분리 |
| Vision node 보정 제어 | 구현됨·실차 재검증 필요 | `--physical-fleet`와 Vision을 함께 켜면 coarse ARRIVED 뒤 fresh MEASURED pose로 제한된 회전/직진 보정 후에만 NODE_ARRIVED 확정 |
| 자동화된 test target | 일부 구현 | trajectory, Vision serializer/store/Unity relay, correction policy를 포함한 CTest 4개 통과; 전체 fleet TCP test framework는 없음 |
| native Windows server build | 계획 아님 | POSIX socket 의존성 때문에 Linux/WSL이 기본 환경 |

## 최근 software 검증

### 2026-08-31 Vision 기반 node 도착 보정 Server 구현

- `--physical-fleet`와 `--vision-observation`을 함께 사용할 때만 Vision 관측을 실차 node 도착 보정에 사용한다. 다른 mode의 planned pose와 기존 Unity 비교 marker 흐름은 그대로 분리된다.
- Server는 전체 계획 경로를 유지하되 ESP32에는 LINE 한 edge씩 final trajectory로 보낸다. ESP32의 coarse `ARRIVED`는 즉시 RoutePlanner로 전달하지 않고, 정지 뒤 들어온 더 새 `MEASURED + VERIFIED` 관측을 기다린다.
- 허용 오차는 위치 20 mm, 도착 heading 10도다. 위치 오차 200 mm 초과는 거절하며, 한 primitive는 직진 최대 120 mm 또는 회전 최대 90도, node당 최대 8회로 제한한다. 바닥 시험에서 8도 미세 회전이 정지 마찰로 STALL 난 기록을 반영해 그 범위는 회전시키지 않는다.
- `NODE_CORRECTION_COMMAND=103`과 `NODE_CORRECTION_REPORT=202`, capability bit 2를 추가했다. 명령과 결과는 route/node/command ID로 결합하며 payload는 각각 17 byte다.
- 시작 시에도 AGV 1이 node 1 중심 20 mm 이내이고 동쪽 heading 10도 이내인 fresh Vision pose를 확인한 뒤에만 첫 자동 route를 보낸다.
- 새 측정 대기 2.5초, primitive 결과 대기 10초를 넘기거나 Vision/robot identity·범위·결과가 맞지 않으면 active route를 취소하고 fail-stop한다.
- 첫 실제 바닥 보정 주행에서 Vision은 계속 `VERIFIED/MEASURED`였고 ESP32 primitive 1~6도 모두 완료됐지만, node 2에 위치 약 30 mm와 heading 약 30도가 남은 상태에서 기존 6회 상한을 소진해 safe-stop했다. 회복 순서에 필요한 짧은 직진과 최종 heading 보정을 허용하도록 Server와 ESP32 상한을 8회로 맞췄으며 9번째 명령은 계속 거절한다.
- 후속 node 6 시험에서는 위치 오차가 13.1 mm까지 수렴한 뒤 90도 도착 heading 보정 중 차체가 옆으로 밀려 위치 오차가 50.7 mm로 다시 증가했다. 위치와 heading 보정이 반복돼 8회 한도에서 safe-stop한 것이며 Vision reject나 통신 단절은 아니었다. node 위치가 20 mm 안에 한 번 들어오면 heading 정렬 단계로 고정해, point-turn 밀림 때문에 위치 보정으로 되돌아가지 않도록 변경했다. heading은 계속 10도 안까지 맞추되 이 단계의 위치 오차가 75 mm를 넘으면 fail-stop한다. 시작 node 1의 위치·동쪽 heading 검사는 유지되며 이 정책은 실차 재검증이 필요하다.
- heading-only 단계 고정 변경 뒤 Server CMake configure/build와 CTest 4개가 통과했다. 이 변경은 Server 전용이라 ESP32 재업로드는 필요 없으며, 실제 바닥에서 다음 edge까지 진행하는지는 재검증이 필요하다.

### 2026-08-29 Vision 관측의 Unity 비교 pose 중계 기반

- Server가 `VisionObservationStore`의 fresh MEASURED/HELD를 기존 replication과 분리된 `UPT_VISION_OBSERVATION=6`으로 Unity session에 전달한다.
- Vision local mm를 TestCase0의 `50 mm/map-unit`, node 1 원점 `(50,-36)`으로 변환하고 heading degree를 radian으로 변환한다.
- Server receive age 500 ms를 넘거나 명시적 LOST이면 같은 transport sequence로도 LOST 전환을 한 번 보내 marker를 숨길 수 있게 한다.
- 이 relay는 planned AGV pose, ARRIVED, 경로·예약·점유와 ESP32 명령을 읽거나 수정하지 않는다.
- CMake build와 CTest 3개가 통과했다.
- Windows Python sender가 WSL Server에 실제 TCP로 MEASURED `(350,350) mm, 90 deg`를 보낸 wire E2E에서 Unity protocol consumer가 `(57,-29)`, `pi/2 rad`와 501 ms timeout LOST를 순서대로 확인했다.
- 위 검증은 실제 Unity Editor 렌더링과 실카메라 동시 실행을 대신하지 않는다. Unity Play 화면과 reference/robot tag의 동일 높이는 별도 실물 확인이 필요하다.

### 2026-08-29 TestCase0 node pitch 350 mm 반영

- 사용자가 바닥 node marker 간격을 350 mm로 변경했다.
- `50 mm/map-unit`은 유지하고 격자 간격을 4 unit에서 7 unit으로 변경했다.
- 모든 44개 directed LINE link 길이는 7 unit이며 전체 node 격자는 약 `1400 x 700 mm`다.
- Vision map contract는 `67254eca75c55e5c`이며, robot heading 0도 보정을 반영한 pose contract는 `f84eb43ebb6cf7ff`다.
- Server map과 hardware-free test를 갱신했고 VisionTracker 실카메라 calibration과 TCP 관측 저장을 확인했다. 실차 350 mm 반복 주행은 재검증이 필요하다.

### 2026-08-25 VisionTracker observation-only Server 기반

- Windows VisionTracker `278cc431`의 TestCase0 좌표·pose 계약을 기준으로 `VISION_HELLO/ACK/OBSERVATION` field serializer와 client identity 고정을 추가했다.
- 기능은 기본 OFF이며 `--vision-observation --vision-calibration-id <ID>`에서만 켜진다. 실제 calibration ID는 카메라 보정 전이므로 placeholder를 코드에 넣지 않았다.
- 유효 관측은 planned/ESP32 상태와 분리된 `VisionObservationStore`에만 저장한다. AGV pose, ARRIVED, 경로·예약·TaskManager·ESP32 명령에는 사용하지 않는다.
- malformed/NaN/Inf, unknown AGV, identity·map·pose contract 불일치, stale/out-of-order/out-of-map/state-pose 오류를 거부한다.
- CMake build와 CTest 3개가 통과했다. 실제 카메라 TCP 송신과 Unity 실측 marker는 아직 구현·검증하지 않았다. 별도 가짜 Vision 송신기는 만들지 않았다.

### 2026-08-13 TestCase0 실제 LINE fleet

- 새 map은 node 15개, directed link 44개이며 모두 양방향 LINE이다.
- Unity에서 다시 export한 정사각 격자 map을 Server `Shared/MapData.json`에 반영했다.
- 모든 link 길이는 4 map-unit이고 heading은 0/±90/180도다.
- 실제 운용 scale `50 mm/map-unit`에서 link 하나는 200 mm, 전체 격자는 약 `800 x 400 mm`다.
- `--physical-fleet`은 node 1의 실제 AGV 1대만 만들고, COMMAND HELLO 뒤 실제 TaskManager/RoutePlanner 자동 배차를 시작한다.
- 자동 경로는 `50 mm/map-unit`, `80 mm/s`, LINE endpoint와 제자리 회전 waypoint로 변환된다.
- ESP32는 실제 node boundary마다 ARRIVED를 한 번 보고한다. 자동 바닥 주행과 Unity 연동은 사용자 실차에서 동작했지만 직진·제자리 회전 오차가 커서 제어 보정 전 반복 운용은 중단한 상태다.
- Unity 또는 portproxy 연결이 갑자기 끊겼을 때 Linux `SIGPIPE`로 Server가 종료되던 문제를 수정했다. TCP send는 `MSG_NOSIGNAL`과 전체-byte 전송을 사용하고, 끊어진 proxy/robot session을 명부에서 제거한다. localhost 강제 RST 재현에서 Server 생존과 session cleanup을 확인했다.

### 2026-08-11 TestCase03 map과 Bezier preview

Unity TestCase03에서 불필요한 중복 node 23을 제거한 뒤 다시 export한 JSON을 Server working tree의 `Shared/MapData.json`에 반영했다.

- node 30개, ID 고유
- directed link 88개, dangling link 없음
- LINE `type=0` 44개, cubic Bezier `type=1` 44개
- 현재 일반 자동 mode는 TESTCASE3 시작 node `{1, 2, 3, 4, 5}`를 사용
- 기존 `--physical-demo`는 AGV 1의 exact `[1 -> 2]`로 분리 유지

별도 `TrajectoryPreview` target으로 directed Bezier link `[1 -> 4]`를 TCP와 motor 없이 읽었다. 시험 공간을 확인한 뒤 `60 mm/map-unit`을 곡선 demo scale로 선택했다. geometry상 시작 접선 heading `pi`, 20 mm spacing에서 17 waypoint, 약 305.884 mm sampled length, 376-byte payload를 만들었다. sampled 최소 회전반경은 약 87.6 mm로 트레드 반폭 65 mm보다 크다. CMake build와 CTest 2개가 통과했다.

ESP32 `bffe8e59` motor-disabled follower trace에서 실제 payload 수신, `minRadius > 65 mm`, `reverse=0`을 확인했다.

Server에는 command-capable client만 허용하는 `--trajectory-raised-wheel` mode를 추가했다. `[1 -> 4]`를 `80 mm/s`로 보내며 CMake build를 통과했고, 실제 모터 시험은 아직 하지 않았다.

후속 Server working tree에 `--trajectory-preview`를 추가했다. 이 mode는 단일 AGV/node 1, 자동 배차 off 상태에서 preview-only capability를 가진 client에만 `[1 -> 4]`를 한 번 전송한다. 모든 waypoint target speed는 0이며 RoutePlanner plan·예약·`ROUTE_COMMAND`·`ARRIVED` 흐름을 만들지 않는다.

이전 25 mm/unit 시점의 FakeRobot TCP E2E도 통과했으며, 현재 60 mm/unit은 실제 ESP32의 motor-disabled trace까지 통과했다.

### 2026-08-10 physical-demo 실차·Unity E2E

사용자 실기 시험에서 다음 vertical slice를 완료했다.

- Server `ee3244f`의 `--physical-demo`가 exact `[1 -> 2]`를 전송
- ESP32 `efc9e191` raised-wheel profile이 encoder 목표, settling, `ARRIVED` gate를 통과
- 공중 시험에서 최종 encoder count `L=545`, `R=547`, `PWM=0`, `STBY=LOW`, `ARRIVAL_REPORTED` 확인
- 이어서 같은 단일 구간의 바닥 주행 성공을 사용자 확인
- Unity `b821d0c2` viewer가 Server map과 AGV 1의 주행/도착 상태를 표시

따라서 Server 명령, 실제 모터/encoder 실행, STATUS/ARRIVED, Server world 갱신, Unity 표시까지의 첫 end-to-end 경로는 검증 완료다. 이 결과는 `[1 -> 2]`를 실물 약 30 cm로 해석하는 임시 scale에 한정되며 임의 맵 경로의 metric 일치를 뜻하지 않는다.

후속 Server working tree에는 기존 `ROUTE_COMMAND`와 호환되는 선택적 `TRAJECTORY_COMMAND` 기반을 추가했다.

- legacy HELLO는 capability 0으로 계속 처리하고 capability 0 writer도 기존 7-byte payload를 유지
- full execution bit와 parse/store-only preview bit를 분리; preview client는 runtime trajectory dispatch 대상이 아님
- type 0 LINE과 type 1 cubic Bezier를 한 robot-local metric waypoint 배열로 변환
- robot-local frame은 신뢰된 실제 시작 heading을 사용하며, 없으면 build 실패
- 급격한 corner는 `STOP`과 node boundary가 없는 `ROTATE_IN_PLACE`, 접선이 연속인 curve는 연속 waypoint로 생성
- 64 waypoint를 넘으면 truncate하지 않고 실패
- format version, malformed/trailing payload, initial turn, synthetic `LINE -> BEZIER -> LINE`, sharp corner, serializer limit smoke test 통과

ESP32 `bffe8e59`는 format v1 수신과 motor-disabled follower trace까지 통과했다. `STATUS` world pose와 실제 곡선 motor follower는 아직 연결하지 않았다. 기존 runtime physical-demo는 계속 `ROUTE_COMMAND`만 보내므로 기존 직선 실차 동작은 바뀌지 않는다.

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
| 실차와 서버의 단일 route end-to-end 실행 | 검증 완료 |
| 실차 진행률의 Unity Digital Twin 반영 | 검증 완료 |
| encoder odometry의 실제 pose 반영 | 계획 |

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

- Vision node 보정의 실제 바닥 흐름과 fail-stop은 확인했지만, node 위치 20 mm 수렴 뒤 heading-only 단계로 고정하는 Server 정책은 아직 실차 재검증하지 않았다.
- 보정 실패 시 Server가 확정하는 node는 마지막 검증 node로 유지된다. 실제 차체는 coarse target 부근에 있을 수 있으므로 fail-stop 뒤 자동 재개하지 말고 사람이 위치를 다시 확인해야 한다.

- TestCase03 export는 Server working tree에 반영됐지만 아직 Server와 Unity 저장소의 기준 commit으로 함께 확정되지 않았다. Unity의 수정 scene/export도 별도 commit으로 보존해야 한다.
- `60 mm/map-unit`은 TestCase03 곡선 demo용 선택값이며 시설 전체의 최종 실측 calibration은 아니다.
- physical-demo가 초기화하는 heading 값은 trajectory의 신뢰된 radian start heading으로 사용할 수 없다. 실제 시작 자세의 좌표계와 단위를 확정하기 전 runtime dispatch를 켜지 않는다.
- `[1 -> 4]` motor-disabled trace는 통과했지만 raised-wheel 곡선 시험 전까지 motor dispatch는 활성화하지 않는다.

- `agvallcode.txt`에 실제 Wi-Fi 인증정보가 Git 추적 상태로 남아 있다. 값은 재사용하거나 문서에 옮기지 않는다.
- 2026-08-06 확인 결과 GitHub 원격 저장소는 public이다. 해당 credential은 이미 노출된 것으로 취급하고 공유기에서 즉시 변경해야 한다.
- 비밀번호를 변경한 뒤 현재 파일을 placeholder로 바꾸고 Git 과거 이력에서도 제거해야 한다. 정리가 끝나기 전 새 commit을 원격에 push하지 않는다.
- 과거 network firmware의 오른쪽 motor 방향 GPIO 순서가 실차 검증값과 반대다. 출력 활성화 전 `BIN1=33`, `BIN2=32`로 통일하고 바퀴를 띄워 방향을 다시 확인한다.
- 과거 문서 일부는 차체 도착 전 상태다. 현재 상태 판단에는 이 문서를 우선한다.
- `Shared/build/` 생성물이 과거 Git에 추적돼 있으나 현재 source of truth가 아니다.
- server reconnect cleanup, robot offline timeout과 실제 emergency-stop 송신 경로를 보강해야 한다.
- physical demo도 같은 AGV ID의 동시/교체 session을 아직 명시적으로 거절하지 않는다. 실차 시험 중 AGV 1로 FakeRobot을 함께 연결하지 않는다.

## 다음 우선 작업

1. 로봇을 node 1 동쪽에 다시 둔 뒤 새 Server binary로 한 번만 시험한다. 이번 변경은 Server 전용이므로 ESP32 재업로드는 필요 없다.
2. 성공 시 Server 로그의 `Node N accepted`와 다음 edge 전송을 확인하고, 실패 시 새 `Primitive limit exhausted` 로그의 남은 위치·heading 오차를 기록
3. Unity Editor에서 planned AGV와 Vision marker가 보정 전후 의도대로 표시되는지 확인
4. 노출된 Wi-Fi 비밀번호 변경과 저장소 secret 정리
5. 전원 분배단자, fuse, switch와 기판 고정

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
