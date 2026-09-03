# Physical ESP32 AGV integration

Last updated: 2026-09-01

Hardware verification record: 2026-08-04

## 현재 결론

단일 직선 `[1 -> 2]`에서는 Server RobotProtocol, motor·encoder, STATUS/ARRIVED와 Unity 표시까지 하나의 firmware로 결합해 바닥 시험을 완료했다. 일반 LINE/Bezier follower와 실제 odometry pose 연동은 아직 별도 단계다.

| 영역 | 상태 |
|---|---|
| 양쪽 motor 및 encoder | 실차 검증 완료 |
| 15 cm·30 cm 직진 | 실차 검증 완료 |
| 좌우 제자리 90도 | 실차 검증 완료 |
| `30 cm -> CW 90도 -> 30 cm` | 바닥 시험 완료 |
| ESP32 RobotProtocol 연결 | 실차 E2E 검증 완료 |
| 실제 motor control + RobotProtocol | exact `[1 -> 2]` 바닥 검증 완료 |
| AprilTag node 도착 보정 | Server 구현·hardware-free test 완료, 새 firmware 실차 미검증 |
| 후진 | 미검증 |
| encoder odometry | motor-locked preview 구현, 실차 pose 미검증 |
| 실제 pose의 Digital Twin 반영 | node/progress 완료, odometry pose 미완료 |

과거 network firmware의 시간 기반 진행 기록은 현재 실차 firmware의 근거가 아니다. 현재 완료 범위는 exact `[1 -> 2]`이고 임의 LINE/Bezier route는 아직 실행하지 않았다.

## 하드웨어

| 부품 | 확인된 사양 |
|---|---|
| MCU | ESP32-WROOM-32E |
| motor driver | TB6612FNG |
| motor | WHEELTEC MG310P20 7.4V encoder motor 2개 |
| 바퀴 | 지름 48 mm |
| track width | 약 130 mm |
| 배터리 | 보호회로 내장 18650 셀 2개, 2S 직렬 holder |
| buck converter | ZX-052 V2.0 / LM2596 계열, 출력 5.0V |

## GPIO 기준

| 기능 | GPIO |
|---|---:|
| 왼쪽 AIN1 | 25 |
| 왼쪽 AIN2 | 26 |
| 왼쪽 PWMA | 27 |
| TB6612 STBY | 13 |
| 오른쪽 BIN1 | 33 |
| 오른쪽 BIN2 | 32 |
| 오른쪽 PWMB | 14 |
| 왼쪽 encoder A / B | 19 / 18 |
| 오른쪽 encoder A / B | 17 / 16 |
| BOOT button | 0 |

중요: 과거 network firmware 스냅샷의 오른쪽 방향핀은 `32/33` 순서이고 실차 검증값은 `BIN1=33`, `BIN2=32`다. 두 코드를 합칠 때 실차값으로 통일한 뒤 바퀴를 띄우고 방향을 재검증하기 전에는 motor output을 활성화하지 않는다.

## 배선

Encoder:

- 파란색: ESP32 `3V3`
- 검은색: 공통 `GND`
- 흰색: A channel
- 노란색: B channel

Motor:

- 왼쪽 빨강/초록: `AO1/AO2`
- 오른쪽 빨강/초록: `BO1/BO2`

Power:

- 배터리 `+`: LM2596 `VIN+`와 TB6612 `VM`으로 분기
- 배터리 `-`: LM2596 `VIN-`와 TB6612 `GND`로 분기
- LM2596 `VOUT+ 5.0V`: ESP32 `5V`
- LM2596 `VOUT-`: ESP32 `GND`
- ESP32 `3V3`: TB6612 `VCC`와 양쪽 encoder VCC
- 모든 GND 공통

## 업로드 안전 절차

현재 임시 배선 기준 절차다.

1. 배터리 한 개를 제거한다.
2. 화면과 LED가 모두 꺼졌는지 확인한다.
3. LM2596 `VOUT+ -> ESP32 5V` 선을 분리한다.
4. `VIN+ -> TB6612 VM` 선을 분리한다.
5. USB를 연결하고 firmware를 upload한다.
6. USB를 제거하고 ESP32 LED가 꺼졌는지 확인한다.
7. 두 전원선을 다시 연결한다.
8. 배터리를 삽입하고 사람이 옆에서 짧게 시험한다.

전원 분배 구조가 정식으로 바뀌면 이 절차도 회로와 함께 다시 검토한다.

## Encoder와 보정값

현재 counting 방식은 encoder A channel의 `RISING` edge에서 B channel을 읽어 방향을 판정한다.

| 항목 | 실차 검증값 |
|---|---:|
| 바퀴 1회전 | 260 count |
| 거리/count | 약 0.580 mm |
| 약 15 cm | 260 count |
| 약 30 cm | 520 count |
| 제자리 90도 | 각 바퀴 176 count |
| 폐기할 값 | 352 count |

`352 count`는 한쪽 motor 선이 빠진 상태에서 얻은 잘못된 값이므로 사용하지 않는다.

방향을 보존해야 하므로 odometry 진행량에 무조건 `abs()`를 적용하지 않는다. 통합 후 아래 부호를 짧은 저속 시험으로 다시 확정한다.

```text
forward: left +, right +
reverse: left -, right -
CW turn: left -, right +
CCW turn: left +, right -
```

속도 기록:

- 고정 PWM 약 30%: 왼쪽 약 151 RPM, 오른쪽 약 140 RPM
- 목표 140 RPM PID 시험 성공
- 최종 L자 경로 시험은 가감속 PWM과 좌우 count 동기화를 사용

## Odometry 초안

초기 내부 단위는 `mm`, 각도는 `rad`로 통일한다.

```text
mmPerCount = pi * 48 / 260
dL = leftDeltaCount * mmPerCount
dR = rightDeltaCount * mmPerCount
dS = (dL + dR) / 2
dTheta = (dR - dL) / 130

x += dS * cos(heading + dTheta / 2)
y += dS * sin(heading + dTheta / 2)
heading += dTheta
```

RobotProtocol은 `x`, `z`, `heading`을 사용하고 heading은 radian이다. 실제 pose를 직접 server 좌표로 보내기 전 다음을 결정한다.

- 실제 원점과 `MapData.json` 원점
- 실제 X/Y와 server X/Z 축 대응
- 회전 부호
- mm와 map 단위의 scale
- 부팅 시 초기 node와 heading

현재 `RobotSession`은 유효한 `currentNodeID/currentLinkID/progress`가 있으면 map 보간 pose를 우선 만들 수 있다. 첫 통합에서는 이 방식으로 Unity를 움직이고 encoder odometry는 별도 로그로 비교하는 것이 안전하다.

## Server 통신 계약

Server는 TCP `6666`, RobotProtocol version `1`을 사용한다.

```text
ESP32 -> Server : HELLO
Server -> ESP32 : HELLO_ACK
Server -> ESP32 : ROUTE_COMMAND
ESP32 -> Server : STATUS (주기 보고)
ESP32 -> Server : ARRIVED (실제 node 도착 후)
Server -> ESP32 : NODE_CORRECTION_COMMAND (Vision 보정이 필요할 때)
ESP32 -> Server : NODE_CORRECTION_REPORT (primitive 안전 완료/거절/fault)
```

추가 packet으로 `CANCEL_ROUTE`, `PING/PONG`, `ERROR_PACKET`, `EMERGENCY_STOP`이 정의돼 있다. 정확한 field와 크기는 [CommunicationProtocol.md](CommunicationProtocol.md)와 `Shared/Protocol.hpp`를 기준으로 한다.

PhysicalFleet의 `WHEEL_MISMATCH`는 기존 `MOTOR_FAULT/detail=65539`로 먼저 fail-stop한 뒤, 같은 ERROR payload의 `detail`을 재사용한 5개 tagged record로 operation, motion mode/profile, 좌우 normalized encoder progress와 개별 target count를 전송한다. Server는 이를 한 snapshot 로그로 조립한다. 이 진단은 기존 wheel mismatch 임계값, fault latch, PWM zero와 `STBY=LOW` 안전 동작을 변경하지 않는다.

통합 원칙:

- Wi-Fi가 끊겨도 ESP32가 local stop 조건을 유지한다.
- `ARRIVED`는 timer가 아니라 encoder/도착 판정이 성공한 뒤에만 보낸다.
- server route node를 local `rotate -> drive -> stop` 단계로 변환한다.
- server는 PWM이나 wheel별 제어값을 지속 전송하지 않는다.
- protocol 처리와 motor control loop가 서로 장시간 block하지 않게 한다.
- correction은 final one-edge 도착과 안전 정지 뒤에만 받으며 route/node/command ID와 local 범위를 다시 검증한다.

### Vision node 도착 보정 계약

`--physical-fleet --vision-observation` 조합에서는 coarse encoder 도착을 곧바로 최종 node 도착으로 확정하지 않는다.

1. Server가 LINE edge 하나를 final `TRAJECTORY_COMMAND`로 보낸다.
2. ESP32가 정지·settling 뒤 STATUS와 ARRIVED를 보내고 다음 trajectory를 기다린다.
3. Server가 ARRIVED 이후의 새 `MEASURED + VERIFIED` AprilTag pose만 읽는다.
4. 최초 coarse pose가 위치 40 mm 이하이고 arrival heading 10도 이내면 위치 primitive 없이 승인한다. 위치 correction은 40 mm 초과에서만 시작하며 목표 방향으로 최대 90도 회전하거나 최대 120 mm 직진하고, 완료 report 뒤 다시 측정한다.
5. correction 중 위치가 35 mm 안에 들어오면 arrival-heading 단계로 전환한다. final turn 중심 이동은 위치 40 mm와 arrival heading 10도 이내까지 `NODE_ARRIVED`로 확정한다. heading 단계에서 위치가 40 mm를 넘으면 bounded position correction으로 복귀하고, 다시 35 mm 이내에 들어와야 heading 단계로 나간다.
6. primitive 목적 오차가 2회 연속 감소하지 않음, 같은 방향 회전 3회째, 누적 회전 360도 초과, 회전 한 번에 위치 오차 25 mm 초과 증가를 비수렴으로 처리한다. 이 조건과 일반 보정 중 200 mm 초과, node당 8회 초과, LOST/HELD/stale, identity 불일치, 측정 2.5초/report 10초 timeout에서는 route를 취소하고 멈춘다.
7. post-arrival pose가 허용 범위에 들면 node/occupancy/route step을 먼저 한 번 확정하고 다음 edge는 controller departure hold에 둔다. 이후 실제 dispatch하려는 모든 edge의 bearing을 출발 heading으로 계산한다. 현재 heading과 10도를 넘게 차이나면 같은 완료 route/node ID의 `NODE_CORRECTION_COMMAND` 회전을 최대 2회 수행하고, 각 완료 report 뒤 명령 전보다 새로운 fresh `MEASURED + VERIFIED` pose로 다시 검사한다. 정렬된 실제 heading을 anchor로 확정한 뒤에만 다음 one-edge trajectory를 보낸다.
8. pre-departure 단계에서는 HELD/LOST/stale, calibration/source session 불일치, robot disconnect, 40 mm 초과 위치 또는 정렬 실패가 모두 forward 미전송 safe-stop이다. 기존 primitive·누적 회전·동일 방향·비수렴 guard를 그대로 공유한다. `phase=PRE_DEPARTURE`, edge, heading, sequence, command/attempt와 `dispatchResult` 로그로 post-arrival 보정과 구분한다.
9. post-arrival과 pre-departure는 서로 다른 correction objective다. 전환 시 동일 방향 반복, 비감소와 이전 command 비교 이력은 초기화하지만 node당 primitive 총수와 누적 회전량은 유지한다. pre 실패 후에도 이미 검증된 도착 node는 confirmed node로 남으며 ARRIVED/pickup/drop 이벤트는 중복 발행하지 않는다.

Server 시작 시에도 AGV 1이 node 1 중심 20 mm 이내이고 동쪽 10도 이내인 fresh pose가 확인돼야 첫 자동 route를 보낸다. 현재 firmware는 `NODE_CORRECTION_COMMAND`를 완료된 route의 `NODE_WAIT`에서만 받으므로 최초 start node의 별도 pre-departure turn은 Server-only로 지원하지 않으며 기존 start gate를 유지한다. correction firmware를 실제 차체에 올리기 전에는 바퀴를 띄운 상태에서 방향·거리·report·BOOT E-stop을 먼저 확인한다.

실차의 제자리회전은 encoder상 완료돼도 바닥 마찰 때문에 차체 중심을 수 cm 이동시킬 수 있다. 따라서 target bearing과 incoming-edge arrival heading을 별도 오차로 계산하고, 35 mm exit/40 mm entry hysteresis로 위치와 arrival-heading 단계를 전환한다. point turn 뒤 위치가 40 mm를 넘으면 위치 보정으로 복귀하며, 기존 position spike와 반복·누적 회전 비수렴 guard는 그대로 fail-stop한다.

승인된 node의 다음 edge start heading은 마지막 승인 판단에 사용한 fresh `MEASURED + VERIFIED` Vision heading을 우선한다. Server는 같은 AGV와 node인지, calibration ID와 현재 source session이 일치하는지, sequence가 유효하고 수신 후 200 ms 이내인지 다시 검사한다. 실패하면 incoming edge의 nominal heading으로 fallback한다. `headingSource=VISION_ACCEPTED|NOMINAL_NODE`와 `visionSequence`로 선택 원인을 확인한다.

## 단계별 통합 순서

### 0. 보안과 source 정리

1. Git에 노출된 Wi-Fi 비밀번호를 변경한다.
2. 실차 검증 코드를 `Firmware/ESP32_AGV/` 같은 buildable project로 가져온다.
3. `secrets.example.h`만 추적하고 실제 `secrets.h`는 ignore한다.
4. 실차 L자 주행 기준점을 commit으로 보존한다.

### 1. 기구·전원 안전 완료

- 분배단자, fuse, power switch, 절연과 기판 고정
- motor 기동/정지 시 5V rail 확인
- 사람이 즉시 전원을 차단할 수 있는 상태 확보

### 2. 후진과 odometry 단독 검증

- 후진 15 cm로 encoder 부호 확정
- 좌우 직진, 회전 count 회귀 시험
- serial log로 `x/y/heading`만 확인
- network와 자율 route는 아직 연결하지 않음

### 3. Motor disabled network 검증

- network firmware에 실차 GPIO와 safety code 이식
- motor output은 비활성화
- Server를 `./build/Server/AGV_Server --physical-demo`로 실행해 AGV 1과 exact `[1 -> 2]`만 사용
- 같은 시험에서 ESP32와 FakeRobot을 동시에 AGV 1로 연결하지 않음
- 바퀴를 띄운 상태에서 HELLO/HELLO_ACK/STATUS/CANCEL/ESTOP 확인
- reconnect와 Wi-Fi 끊김 시 안전 상태 확인

2026-08-10 기준 Server physical demo는 FakeRobot뿐 아니라 실제 ESP32와 Unity까지 검증됐다. 기존 `[1 -> 2]` firmware는 해당 논리 구간을 약 30 cm로 해석한다. 새 trajectory follower는 이 하드코딩과 분리해 TestCase03 demo scale `60 mm/map-unit`을 사용하며, 이 경우 `[1 -> 2]`는 약 720 mm다.

### 4. RouteExecutor 연결

- node route를 회전·직진 leg로 변환
- time simulation progress를 encoder progress로 교체
- 실제 node 도착 후 STATUS와 ARRIVED 전송
- 처음에는 AGV 1대, 직선 1구간, 낮은 PWM으로 제한

단일 직선 `[1 -> 2]`에 대해서는 위 항목을 완료했다. 다음 executor 확장은 기존 경로를 다시 하드코딩하지 않고, Server가 LINE/BEZIER map link를 robot-local metric waypoint로 샘플링한 `TRAJECTORY_COMMAND`를 따르는 방식으로 진행한다. capability가 없는 기존 firmware에는 계속 `ROUTE_COMMAND`만 보내 기존 검증본을 보존한다.

현재 실제 TestCase0 단계는 Bezier를 제외한다. `--physical-fleet`에서 AGV 1을 node 1, 동쪽 방향으로 배치하고 COMMAND-capable firmware 연결 뒤에만 자동 임무를 시작한다. map scale은 `50 mm/unit`이고 정사각 격자의 각 link는 7 unit, 즉 350 mm다. 각 LINE node boundary에서 정지·ARRIVED 후 필요하면 제자리 회전한다.

PhysicalFleet의 correction 승인과 다음 one-edge trajectory의 초기 heading 검사는 같은 10도(`0.174532925` rad) 허용치를 사용한다. Server는 이 범위 안의 잔여 heading에는 불필요한 rotate waypoint를 만들지 않는다. ESP32 follower도 동일한 10도 validation 계약이어야 하며, 기존 0.08 rad firmware에서는 4.58~10도 구간을 계속 `INVALID_COMMAND`로 거절하므로 실차 재시험 전에 반드시 대조한다.

### 5. Digital Twin 비교

- server의 node/progress 기반 pose와 odometry pose를 함께 log
- 원점, axis, scale, heading 오차 보정
- 15 cm, 후진, 좌우 90도, L자 route 회귀 시험
- 그 뒤에만 다중 AGV/예약 route와 결합

node/progress 기반 Unity 이동은 physical-demo 실차와 함께 검증됐다. TestCase03 `[1 -> 4]` preview는 `60 mm/map-unit`에서 약 305.9 mm, sampled 최소 반경 약 87.6 mm다. ESP32 `bffe8e59` motor-disabled trace에서 실제 수신과 `reverse=0`을 확인했으며, 다음 단계는 raised-wheel 곡선 시험이다.

웹 조종은 진단 수단으로는 유용하지만, 본 시스템 연동은 이미 정의된 RobotProtocol을 먼저 완성한다.

### 6. Vision correction 승격

- Server/ESP32의 capability bit, packet ID와 17-byte payload 순서를 먼저 대조
- motor-locked 통신에서 unsupported client가 dispatch되지 않는지 확인
- 바퀴를 띄워 20 mm 직진과 최소 CW/CCW correction을 각각 한 번 확인
- `STATUS -> NODE_CORRECTION_REPORT` 순서와 새 Vision measurement gate 확인
- 저속 바닥에서 한 node만 보정하고, 성공 후에만 다음 edge 자동 전송 확인
- timeout·LOST·BOOT E-stop 중 하나를 의도적으로 만들어 route cancel과 safe output 확인

## 안전 핵심

- 배터리 원전압을 ESP32 `5V` 또는 `3V3`에 직접 연결하지 않는다.
- 전원이 들어온 상태에서 배선을 빼거나 나사를 조이지 않는다.
- 현재 임시 전원 구조에서는 USB와 LM2596 5V를 ESP32에 동시에 공급하지 않는다.
- 임시 배선은 사람이 옆에서 짧게만 시험한다.
- 장시간 또는 원격 주행 전에 fuse, switch, 절연, 고정과 물리적 긴급 정지를 갖춘다.
- motor output을 켜기 전에 방향 GPIO, encoder 부호, BOOT stop, timeout을 바퀴를 띄운 상태에서 확인한다.
- correction fail-stop 뒤에는 Server의 마지막 확정 node와 실제 차체 위치가 다를 수 있으므로 사람이 재배치하기 전 자동 재개하지 않는다.
