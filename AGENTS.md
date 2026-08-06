# Repository instructions

이 지침은 저장소 전체에 적용된다. 하위 디렉터리에 더 구체적인 `AGENTS.md`가 생기면 해당 범위에서는 하위 지침을 함께 따른다.

## 작업 전 필수 확인

1. `README.md`
2. `docs/README.md`
3. `docs/current-status.md`
4. 변경 영역에 맞는 기준 문서
   - 시스템 경계: `docs/architecture.md`
   - wire format: `docs/CommunicationProtocol.md`
   - 실차: `docs/physical-agv-integration.md`
   - 경로 계획: `docs/whca-code-walkthrough.md`

항상 `git status --short --branch`로 사용자 변경과 현재 branch를 먼저 확인한다. 기존 변경은 사용자 소유이므로 덮어쓰거나 되돌리지 않는다.

## 확인된 시스템 경계

- 기본 실행 환경은 Linux/WSL, 언어 표준은 C++20이다.
- `AGV_Server`가 world state, 작업, 경로와 예약의 source of truth다.
- Unity는 서버 state를 렌더링하는 viewer이며 AGV 계획의 source of truth가 아니다.
- ESP32/FakeRobot은 `RobotProtocol` client다. 서버는 route를 보내고 robot은 `STATUS`와 `ARRIVED`를 보고한다.
- 현재 저장소에는 WPF/HMI나 실제 `TrafficControlManager` 클래스가 없다. TODO나 계획을 현재 구현처럼 문서화하지 않는다.
- `Client/ClientMain.cpp`는 빈 stub이다. 완성된 client로 간주하지 않는다.
- 정식 Unity 프로젝트와 buildable ESP32 firmware 프로젝트는 현재 CMake 소스 트리에 없다.

## 설계 불변식

- `PathFinder`는 예약 상태를 조회하며 경로 후보를 계산하지만 예약을 확정 기록하지 않는다.
- `RoutePlanner`가 경로 수명주기와 transactional reservation 확정을 조정한다.
- `ReservationTable`은 미래 시간 구간의 node/edge/goal 예약을 관리한다.
- `OccupancyProvider`는 실제 실행 시점의 현재 node/edge 점유를 관리한다. 예약과 실제 점유를 혼동하지 않는다.
- robot 구현 차이는 `IRobotController` 경계 뒤에 둔다. 경로 계획이 Unity/ESP32/FakeRobot 세부사항에 의존하지 않게 유지한다.
- 시스템 이벤트는 기존 `EventManager` 흐름을 우선 사용한다.
- RobotProtocol의 실제 기준은 `Shared/Protocol.hpp`와 `Shared/PacketSerializer.*`다. 프로토콜 변경 시 server, FakeRobot, ESP32 쪽 호환성과 `docs/CommunicationProtocol.md`를 함께 검토한다.
- 맵의 기준 데이터는 `Shared/MapData.json`이다. link는 방향성이 있으므로 양방향 이동에는 양방향 link가 필요하다.

## 코드와 문서 작업 규칙

- 요청 범위 밖의 대규모 이름 변경이나 아키텍처 재편을 하지 않는다.
- 현재 C++ naming과 주변 코드 스타일을 보존하고, 새 소유권에는 RAII와 명확한 lifetime을 우선한다.
- 프로토콜 정수 폭, byte order, frame size를 추측하지 말고 serializer 구현을 확인한다.
- 하드웨어 동작은 사용자 시험 기록과 코드 근거를 구분한다. 실차에서 재검증하지 않은 항목을 “완료”로 표시하지 않는다.
- architecture, protocol, hardware milestone이 바뀌면 관련 기준 문서와 `docs/current-status.md`를 갱신한다.
- Markdown 링크는 저장소 상대경로를 사용한다. 사용자 홈의 절대경로를 문서에 넣지 않는다.
- 비밀번호, SSID 인증정보, token, API key를 출력·문서화·커밋하지 않는다. 예시는 placeholder만 사용한다.
- commit이나 push는 사용자가 요청한 범위에서만 수행한다. push 전에는 staged 파일과 secret 포함 여부를 확인한다.

## 기준이 아닌 파일

- 루트의 `*allcode.txt`, `esp32.txt`, 날짜형 보고서 파일은 과거 스냅샷/기록이다. 명시적 요청 없이 수정하거나 현재 빌드 기준으로 사용하지 않는다.
- `build/`는 생성물이다.
- `Shared/build/`는 과거에 추적된 생성물이며 현재 소스의 기준이 아니다. 명시적 정리 작업이 아니면 수정하지 않는다.
- `docs/PortfolioGuide.md`와 `docs/velog-*.md`는 발표·게시용 또는 과거 상태 자료다. 현재 구현 판단에는 `docs/current-status.md`와 코드가 우선한다.

## 검증 명령

소스 변경의 기본 검증:

```bash
cmake -S . -B build
cmake --build build -j
```

통신 또는 robot controller 변경 시 가능한 경우 저장소 루트에서 다음 smoke test를 수행한다.

```bash
./build/Server/AGV_Server
./build/Server/FakeRobot 127.0.0.1:6666 1
```

자동화된 test target은 현재 없다. 실행 검증을 못 했다면 이유와 미검증 범위를 결과에 명시한다. native Windows server 빌드가 된다고 가정하지 않는다.

## 환경 간 인수인계

Codex 세션 자체는 Windows와 WSL 사이에서 공유되지 않는다. 장기 기억은 Git에 추적되는 문서로 남긴다.

- 작업 시작 시 `docs/current-status.md`의 “다음 우선 작업”과 알려진 위험을 확인한다.
- 의미 있는 milestone, blocker, 검증 결과가 바뀐 경우에만 해당 문서를 갱신한다.
- 진행 중인 세부 작업은 별도 branch와 commit으로 전달하고, 대화에만 남겨두지 않는다.
- 동일 파일을 두 환경에서 동시에 수정해야 하면 먼저 작업 소유권과 branch를 나눈다.
