# Current project status

Last verified: 2026-08-06

Implementation base: `c56ada3` on `old-new-combined`

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
| Server-authoritative world | 검증 완료 | 현재 TESTCASE0에서 AGV 4대 생성 |
| WHCA* 계열 route planning | 검증 완료 | smoke test에서 route 생성과 후속 route 재전송 확인 |
| 시간 기반 node/edge/goal reservation | 검증 완료 | server/FakeRobot smoke flow에서 사용 |
| 실행 시점 occupancy 검사 | 검증 완료 | simulator/controller 실행 flow에서 사용 |
| Unity map/replication protocol | 구현됨·재검증 필요 | Unity 프로젝트 자체는 CMake tree에 없음 |
| RobotProtocol v1 | 검증 완료 | HELLO_ACK, ROUTE_COMMAND, STATUS/ARRIVED flow 확인 |
| FakeRobot | 검증 완료 | localhost에서 AGV 1로 연결해 여러 route와 arrival 확인 |
| 자동화된 test target | 계획 | 현재는 build와 FakeRobot smoke test 중심 |
| native Windows server build | 계획 아님 | POSIX socket 의존성 때문에 Linux/WSL이 기본 환경 |

## 최근 software 검증

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
- 해당 비밀번호를 먼저 변경한 뒤 현재 파일을 placeholder로 바꾸고, 원격 노출 범위에 따라 Git 이력 정리를 결정해야 한다.
- 과거 network firmware의 오른쪽 motor 방향 GPIO 순서가 실차 검증값과 반대다. 출력 활성화 전 `BIN1=33`, `BIN2=32`로 통일하고 바퀴를 띄워 방향을 다시 확인한다.
- 과거 문서 일부는 차체 도착 전 상태다. 현재 상태 판단에는 이 문서를 우선한다.
- `Shared/build/` 생성물이 과거 Git에 추적돼 있으나 현재 source of truth가 아니다.
- server reconnect cleanup, robot offline timeout과 실제 emergency-stop 송신 경로를 보강해야 한다.

## 다음 우선 작업

1. 노출된 Wi-Fi 비밀번호 변경과 저장소 secret 정리
2. 실차 검증 코드를 `Firmware/ESP32_AGV/` 같은 buildable PlatformIO/Arduino project로 가져오기
3. `secrets.example.h`만 추적하고 실제 `secrets.h`는 ignore 처리
4. 전원 분배단자, fuse, switch와 기판 고정
5. 후진 15 cm 시험으로 양쪽 encoder 부호 확정
6. 실차 GPIO와 모든 local safety를 network firmware에 옮기되 motor output은 끈 상태로 통신 검증
7. 바퀴를 띄운 상태에서 HELLO/STATUS/CANCEL/ESTOP 검증
8. odometry와 route executor를 추가하고 시간 기반 progress 제거
9. 낮은 속도로 단일 route를 실행한 뒤 Unity 오차 측정

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
