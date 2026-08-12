# Physical ESP32 AGV integration

Last updated: 2026-08-11

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
```

추가 packet으로 `CANCEL_ROUTE`, `PING/PONG`, `ERROR_PACKET`, `EMERGENCY_STOP`이 정의돼 있다. 정확한 field와 크기는 [CommunicationProtocol.md](CommunicationProtocol.md)와 `Shared/Protocol.hpp`를 기준으로 한다.

통합 원칙:

- Wi-Fi가 끊겨도 ESP32가 local stop 조건을 유지한다.
- `ARRIVED`는 timer가 아니라 encoder/도착 판정이 성공한 뒤에만 보낸다.
- server route node를 local `rotate -> drive -> stop` 단계로 변환한다.
- server는 PWM이나 wheel별 제어값을 지속 전송하지 않는다.
- protocol 처리와 motor control loop가 서로 장시간 block하지 않게 한다.

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

2026-08-12의 실제 TestCase0 단계는 Bezier를 제외한다. `--physical-fleet`에서 AGV 1을 node 1, 동쪽 방향으로 배치하고 COMMAND-capable firmware 연결 뒤에만 자동 임무를 시작한다. map scale은 `50 mm/unit`이며 각 LINE node boundary에서 정지·ARRIVED 후 필요하면 제자리 회전한다.

### 5. Digital Twin 비교

- server의 node/progress 기반 pose와 odometry pose를 함께 log
- 원점, axis, scale, heading 오차 보정
- 15 cm, 후진, 좌우 90도, L자 route 회귀 시험
- 그 뒤에만 다중 AGV/예약 route와 결합

node/progress 기반 Unity 이동은 physical-demo 실차와 함께 검증됐다. TestCase03 `[1 -> 4]` preview는 `60 mm/map-unit`에서 약 305.9 mm, sampled 최소 반경 약 87.6 mm다. ESP32 `bffe8e59` motor-disabled trace에서 실제 수신과 `reverse=0`을 확인했으며, 다음 단계는 raised-wheel 곡선 시험이다.

웹 조종은 진단 수단으로는 유용하지만, 본 시스템 연동은 이미 정의된 RobotProtocol을 먼저 완성한다.

## 안전 핵심

- 배터리 원전압을 ESP32 `5V` 또는 `3V3`에 직접 연결하지 않는다.
- 전원이 들어온 상태에서 배선을 빼거나 나사를 조이지 않는다.
- 현재 임시 전원 구조에서는 USB와 LM2596 5V를 ESP32에 동시에 공급하지 않는다.
- 임시 배선은 사람이 옆에서 짧게만 시험한다.
- 장시간 또는 원격 주행 전에 fuse, switch, 절연, 고정과 물리적 긴급 정지를 갖춘다.
- motor output을 켜기 전에 방향 GPIO, encoder 부호, BOOT stop, timeout을 바퀴를 띄운 상태에서 확인한다.
