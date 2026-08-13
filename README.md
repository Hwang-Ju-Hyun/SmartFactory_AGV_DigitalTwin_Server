# SmartFactory AGV Digital Twin Server

C++20/Linux 기반의 server-authoritative AGV 관제 서버다. 서버가 창고 맵, AGV 상태, 작업 배정, 경로 계획과 충돌 회피를 소유하고 Unity는 디지털 트윈 viewer, ESP32와 FakeRobot은 경로를 실행하고 상태를 보고하는 robot client 역할을 한다.

> 현재 서버와 FakeRobot용 RobotProtocol, Unity replication 코드가 있으며, `--physical-demo`의 `[1 -> 2]` 명령으로 ESP32 실차가 바닥에서 주행하고 Unity가 같은 AGV를 표시하는 첫 end-to-end vertical slice까지 검증됐다. Unity TestCase03 export는 Server working tree에 동기화됐고 의도한 첫 `[1 -> 4]` Bezier를 hardware-free metric waypoint로 변환했다. Server의 preview-only 전송 모드도 구현됐으며, 다음 단계는 motor-locked ESP32에서 실제 수신 로그를 확인하는 것이다. 최신 상태는 [docs/current-status.md](docs/current-status.md)를 기준으로 한다.

![AGV Fleet Control System Architecture](docs/assets/architecture-overview.svg)

## 핵심 구성

| 구성요소 | 역할 | 현재 저장소 상태 |
|---|---|---|
| `AGV_Server` | 중앙 world state, 작업 배정, 경로·예약, TCP 통신 | CMake 빌드 대상 |
| `Shared` | 소켓, 직렬화, 맵, PathFinder, ReservationTable, 이벤트 | 정적 라이브러리 |
| `FakeRobot` | ESP32와 같은 RobotProtocol을 사용하는 PC 에뮬레이터 | CMake 빌드 대상 |
| Unity | 서버의 맵과 AGV pose를 렌더링하는 viewer | 프로토콜 지원 코드가 서버에 있으며 Unity 프로젝트는 CMake 트리 밖에 있음 |
| ESP32 AGV | route 실행, 모터·엔코더·PID, STATUS/ARRIVED 보고 | 현재 루트의 텍스트 기록만 존재하며 정식 firmware 프로젝트는 아직 미수록 |
| `AGV_Client` | 초기 C++ client 자리표시자 | 실행 동작이 없는 stub |

서버는 저수준 PWM 명령을 보내지 않는다. `ROUTE_COMMAND`로 노드 순서와 예정 시간을 보내고, 실제 모터 제어와 안전 정지는 ESP32가 담당한다.

## 저장소 구조

```text
.
├── Server/                 # 서버, route/task orchestration, robot controllers
├── Shared/                 # 공통 네트워크·맵·경로·예약 코드
├── Client/                 # 현재 비어 있는 C++ client stub
├── docs/                   # 기준 문서, 상태, 연동 기록, 발표 자료
├── CMakeLists.txt
├── AGENTS.md               # Codex/에이전트 작업 규칙
└── README.md
```

루트의 `*allcode.txt`, `esp32.txt`, 날짜가 붙은 보고서는 과거 스냅샷 또는 참고 기록이다. CMake 빌드의 기준 소스가 아니다.

## 빌드

기본 검증 환경은 Linux/WSL이다. POSIX socket API를 직접 사용하므로 native Windows 빌드는 별도 포팅 없이 지원되지 않는다.

필요 조건:

- CMake 3.15 이상
- C++20을 지원하는 컴파일러
- GLM 헤더
- Linux/WSL의 POSIX socket 환경

```bash
cmake -S . -B build
cmake --build build -j
```

생성되는 주요 실행 파일:

```text
build/Server/AGV_Server
build/Server/FakeRobot
build/Server/TrajectorySmokeTest
build/Server/TrajectoryPreview
build/Client/AGV_Client
```

## 로컬 실행과 빠른 통신 확인

저장소 루트에서 서버를 실행해야 `Shared/MapData.json`을 안정적으로 찾을 수 있다.

터미널 1:

```bash
./build/Server/AGV_Server
```

터미널 2:

```bash
./build/Server/FakeRobot 127.0.0.1:6666 1
```

서버는 TCP `0.0.0.0:6666`에서 Unity legacy protocol과 RobotProtocol을 구분해 처리한다. FakeRobot에서 `HELLO_ACK`, `ROUTE`, `STATUS`, `ARRIVED` 로그가 이어지는지 확인한다.

### 실제 TestCase0 LINE fleet

```bash
./build/Server/AGV_Server --physical-fleet
```

AGV 1대가 node 1에서 동쪽(`+X`, `0 rad`)을 향한 상태로 시작한다. COMMAND-capable ESP32가 연결된 뒤에만 TaskManager 자동 배차가 시작된다. Server가 계산한 임의 LINE 경로를 `50 mm/map-unit`, `80 mm/s` trajectory로 변환하며 특정 node sequence를 하드코딩하지 않는다.

LINE/Bezier 혼합 trajectory sampler와 wire serializer의 hardware-free 회귀시험:

```bash
./build/Server/TrajectorySmokeTest
```

현재 TestCase0의 LINE `[1 -> 2]` geometry를 TCP와 motor 없이 확인하는 preview:

```bash
./build/Server/TrajectoryPreview \
  --scale-mm-per-unit 50 \
  --spacing-mm 50 \
  --speed-mm-s 80 \
  --start-heading-rad 0 \
  --track-width-mm 130 \
  1 2
```

현재 TestCase0 실제 운용 scale은 `50 mm/map-unit`이며 `[1 -> 2]`는 200 mm다. 전체 격자는 약 `800 x 400 mm`다. Bezier preview 기록은 `docs/current-status.md`에 과거 단계로 남겨 둔다.

### Motor-locked trajectory network preview

```bash
./build/Server/AGV_Server --trajectory-preview
```

이 mode는 AGV 1대만 node 1에 만들고 자동 배차와 RoutePlanner 실행을 끈다. `PREVIEW` capability만 광고하고 실행 capability는 광고하지 않은 client에만 `[1 -> 4]` `TRAJECTORY_COMMAND`를 한 번 보내며, 모든 target speed는 0이다. `ROUTE_COMMAND`와 `ARRIVED` 흐름은 만들지 않는다. 실제 scale과 시작 heading을 확정한 실행 mode가 아니다.

곡선 공중시험 전용 실행 mode:

```bash
./build/Server/AGV_Server --trajectory-raised-wheel
```

`COMMAND` capability client에만 `[1 -> 4]`, `60 mm/map-unit`, `80 mm/s`를 전송한다. 자동 배차는 꺼지며 ESP32의 local BOOT 승인과 안전 제한이 우선한다.

실제 ESP32나 기본 6666 listener와 분리한 localhost FakeRobot 검증:

```bash
# 터미널 1
./build/Server/AGV_Server \
  --trajectory-preview \
  --listen 127.0.0.1:16666

# 터미널 2
./build/Server/FakeRobot \
  127.0.0.1:16666 1 \
  --expect-trajectory-preview 1 4 \
  --deadline-ms 3000
```

성공 시 FakeRobot은 `zeroSpeed=1`과 `TRAJECTORY_PREVIEW_PASS`를 출력하고 종료한다.

### Physical demo

실물 ESP32와 첫 단일 구간을 확인할 때는 physical demo mode를 사용한다.

```bash
./build/Server/AGV_Server --physical-demo
```

이 mode는 AGV 1대만 node 1에 만들고 자동 작업 배정을 끈다. AGV 1의 `HELLO`를 승인한 뒤 RoutePlanner가 정확히 `[1 -> 2]`인지 확인하고 예약한 route만 전송한다. 현재 ESP32 firmware는 이 논리 구간을 실물 30 cm로 임시 해석한다. TestCase03 map에서 1→2는 12.0 map unit이지만 이 대응을 전체 맵의 확정 scale로 간주하지 않는다.

이 option 자체는 ESP32 motor output을 켜지 않는다. 실제 출력 허용 여부는 ESP32 firmware의 build profile과 local BOOT/countdown/safety gate가 결정한다. 초기 motor-disabled 검증 뒤 별도 raised-wheel profile로 바퀴 공중 시험과 바닥 직진 시험을 완료했다. 저장소의 VS Code 설정은 CMake Tools에서 `AGV_Server` target의 벌레 버튼을 누를 때 이 option을 자동 전달한다. 다른 target, 특히 `FakeRobot`은 위 터미널 명령으로 별도 실행한다.

Physical demo 중에는 ESP32와 FakeRobot을 동시에 AGV 1로 연결하지 않는다. 현재 server는 같은 AGV의 이전 session 정리를 완성하지 않았으므로 재접속 시험은 motor-disabled 상태에서만 하고, 이상한 재전송이 보이면 server와 ESP32 연결을 모두 종료한 뒤 server부터 다시 시작한다.

## 문서 읽는 순서

1. [현재 구현 상태와 다음 작업](docs/current-status.md)
2. [전체 아키텍처](docs/architecture.md)
3. [통신 프로토콜](docs/CommunicationProtocol.md)
4. [실물 ESP32 AGV 연동](docs/physical-agv-integration.md)
5. [WHCA* 계열 경로 계획 코드 해설](docs/whca-code-walkthrough.md)
6. [Windows/WSL/Git 작업 흐름](docs/development-workflow.md)

문서 전체의 성격과 기준 우선순위는 [docs/README.md](docs/README.md)에 정리되어 있다.

## Windows와 WSL에서 함께 작업할 때

Codex 대화 내용과 커밋하지 않은 변경은 환경 사이에서 자동 공유되지 않는다. 두 환경이 공유하는 기준은 Git에 커밋하고 원격에 push한 파일이다.

- 가장 단순한 방식은 WSL clone 하나를 두고 Windows IDE가 Remote WSL로 여는 것이다.
- Windows와 WSL에 clone을 각각 둘 경우, 동시에 같은 branch와 같은 파일을 수정하지 않는다.
- 작업 시작 전 pull, 작업 종료 전 `docs/current-status.md` 갱신, commit, push를 한 묶음으로 수행한다.
- 자세한 명령과 충돌 방지 규칙은 [개발 작업 흐름](docs/development-workflow.md)을 따른다.

비밀번호, Wi-Fi 인증정보, API key는 Git에 저장하지 않는다. 현재 GitHub 원격은 public이며 과거 ESP32 스냅샷에 실제 인증정보가 추적돼 있으므로 이미 노출된 credential로 취급한다. 비밀번호 변경과 Git 이력 정리가 끝나기 전에는 push하지 않는다.
