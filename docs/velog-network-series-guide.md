# AGV 네트워크 Velog 시리즈 가이드

## 추천 시리즈 구성

네트워크 글은 한 편에 모두 넣으면 너무 길어진다. 아래처럼 나누는 것을 추천한다.

## 0편. 네트워크 서버 구조부터 다시 보기

주제:

- 서버 전체 아키텍처
- 왜 Linux server process로 분리했는가
- 왜 네트워크가 Digital Twin의 핵심 경계가 되는가
- 현재 구현은 epoll이 아니라 select 기반이라는 점
- ClientProxy의 역할
- TCPSession과 packet size framing
- NetworkManagerServer가 Unity protocol과 RobotProtocol을 분기하는 방식
- ROS를 바로 쓰지 않은 이유

대표 그림:

- `docs/assets/server-architecture.svg`

대표 코드:

- `ServerMain.cpp` accept/select loop
- `SocketUtil::Select`
- `NetworkManagerServer::OnClientAccepted`
- `ClientProxy` 생성자 callback
- `TCPSession::ProcessIncomingData`
- `NetworkManagerServer::ProcessPacket`
- `NetworkManagerServer::UpdateWorld`

핵심 문장:

> 서버를 Unity 내부 로직이 아니라 독립적인 Linux TCP process로 분리하면서, Unity와 ESP32가 서로 다른 역할의 client로 붙을 수 있는 기반을 만들었다.

## 1편. C++ Server와 Unity를 TCP로 연결하기

주제:

- 왜 Unity를 단순 simulation 주체로 두지 않았는가
- 왜 Server-authoritative 구조로 갔는가
- 왜 Replication을 썼는가
- TCP packet size framing
- NetworkID / ClassID / ReplicationAction
- Unity에서 GameObject를 어떻게 생성/갱신하는가

대표 그림:

- `docs/assets/unity-replication-flow.svg`
- `docs/assets/unity-protocol-frame.svg`

대표 코드:

- `TCPSession::SendPacket`
- `TCPSession::ProcessIncomingData`
- `ReplicationManagerServer::Write`
- `NetworkManagerServer::SendOutgoingReplicationPackets`
- Unity `HandleReplicatePacket_Recv`
- Unity `RenderManager::UpdateObjectPosition`

핵심 문장:

> Unity가 AGV를 움직이는 것이 아니라, Server가 가진 AGV world state를 Unity가 복제해서 보여주는 구조로 만들었다.

## 2편. Unity 다음은 ESP32, RobotProtocol 분리

주제:

- 왜 Unity protocol과 RobotProtocol을 분리했는가
- ESP32는 왜 replication client가 아닌가
- HELLO / HELLO_ACK
- ROUTE_COMMAND / STATUS / ARRIVED
- 왜 Server가 forward/turn/PWM을 직접 보내지 않는가
- FakeRobot을 만든 이유

대표 그림:

- `docs/assets/architecture-overview.svg`
- `docs/assets/responsibility-split.svg`
- `docs/assets/packet-frame.svg`
- `docs/assets/protocol-sequence.svg`

대표 코드:

- `RobotProtocol::PacketID`
- `RobotProtocol::PacketHeader`
- `NetworkManagerServer::TryProcessRobotProtocolPacket`
- `NetworkManagerServer::HandleRobotHelloPacket`
- `RobotSession::SendRoute`
- `RobotSession::SendRobotPacket`

핵심 문장:

> Unity 대신 ESP32가 붙는 것이 아니라, Unity와 ESP32가 서로 다른 책임을 가진 client로 같은 Server에 붙는 구조를 만들었다.

## 3편 후보. ESP32 연결 디버깅과 실제 하드웨어 전환 준비

아직 차체가 완성되기 전이라면 3편은 나중에 작성하는 편이 좋다.

추천 주제:

- WSL에서 서버를 돌리고 Windows/ESP32에서 접속하기
- `Connection reset by peer` 원인 분석
- AGV ID mismatch로 `UNKNOWN_AGV`가 났던 문제
- FakeRobot과 ESP32의 차이
- 현재 ESP32 route progress simulation의 한계
- 차체 도착 후 바뀔 부분: motor, encoder, battery, PID

대표 그림:

- Windows / WSL / ESP32 네트워크 연결 그림
- 현재 임시 simulation과 실제 주행 구조 비교 그림

핵심 문장:

> 하드웨어가 없을 때는 FakeRobot과 ESP32 임시 firmware로 protocol loop를 먼저 검증하고, 차체가 도착하면 motor/encoder layer만 교체하는 방식으로 진행했다.

## 글쓰기 원칙

Velog 글에서는 구현 결과보다 "왜 이렇게 설계했는가"를 먼저 써야 한다.

추천 순서:

1. 문제 제기
2. 처음 생각했던 단순한 방법
3. 그 방법의 한계
4. 선택한 구조
5. 핵심 코드
6. 이 구조가 이후 단계에 준 장점
7. 남은 한계

## 코드 넣는 기준

코드는 길게 넣기보다, 설계 의도가 드러나는 부분만 넣는다.

좋은 코드:

- packet frame을 만드는 코드
- packet type으로 분기하는 코드
- replication command를 쓰는 코드
- HELLO 검증 코드

피하는 코드:

- include 전체
- 생성자 전체
- 긴 business logic 전체
- 이미 설명한 내용이 반복되는 코드

한 코드 블록은 가능하면 20-40줄 안쪽으로 줄이는 것이 좋다.

## 다이어그램 넣는 기준

각 글에는 최소 1개의 대표 그림이 있으면 좋다.

1편:

- Server -> TCP -> Unity replication flow

2편:

- Server / Unity / ESP32 / FakeRobot architecture
- RobotProtocol packet frame
- HELLO / ROUTE / STATUS sequence

이미지는 Velog에 올릴 때 SVG를 그대로 올리거나, PNG로 변환해서 업로드한다.  
Markdown의 상대 경로는 로컬 문서용이므로 Velog에 붙여넣을 때는 Velog 이미지 URL로 교체하면 된다.

## 톤

너의 기존 글처럼 다음 톤을 유지하면 좋다.

- "처음에는 이렇게 생각했다"
- "하지만 실제로 구현해보니 문제가 있었다"
- "그래서 이런 구조를 선택했다"
- "이 방식의 장점은..."
- "아직 남은 한계는..."

이 톤이 좋은 이유는 단순 결과물이 아니라 설계 과정이 보이기 때문이다.
