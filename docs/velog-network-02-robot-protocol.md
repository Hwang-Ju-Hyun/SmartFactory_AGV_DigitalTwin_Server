# [AGV 프로젝트] Unity 다음은 ESP32 - 실제 로봇을 붙이기 위한 RobotProtocol 설계

## 0. 들어가며

이전 글에서는 C++ Server와 Unity를 연결하고, Server world state를 Unity에 복제하는 Replication 구조를 정리했다.

하지만 실제 AGV를 붙이려면 문제가 하나 더 생긴다.

> Unity와 ESP32를 같은 클라이언트처럼 다뤄도 될까?

처음에는 둘 다 TCP client니까 비슷하게 보였다.  
하지만 역할이 완전히 달랐다.

```text
Unity:
  Server 상태를 화면에 보여주는 viewer

ESP32:
  실제 로봇의 motor, encoder, battery 상태를 다루는 physical client
```

그래서 Unity replication protocol과 별도로, 실제 로봇용 통신 규칙인 **RobotProtocol**을 만들었다.

## 1. 전체 아키텍처

현재 목표 구조는 다음과 같다.

![AGV Fleet Control System Architecture](assets/architecture-overview.svg)

중요한 점은 이것이다.

```text
Unity 대신 ESP32가 붙는 것이 아니다.
Unity도 붙고, ESP32도 붙는다.
```

서버는 중앙 관제 서버이고, Unity와 ESP32는 각각 다른 역할의 client다.

```text
Server
  - TaskManager
  - RoutePlanner
  - ReservationTable
  - AGV world state

Unity
  - Digital Twin rendering

ESP32
  - Route execution
  - Motor control
  - Encoder reading
  - STATUS / ARRIVED report
```

## 2. 왜 기존 Unity packet을 그대로 쓰지 않았는가

처음 Unity 통신에는 이런 packet들이 있었다.

```cpp
enum UnityPacketType : uint8_t
{
    UPT_REPLICATION = 0,
    UPT_MAZE_DATA = 1,
    UPT_HELLO = 2,
    UPT_DISCONNECTED = 3,
    UPT_READY_MAP = 4,
    UPT_READY_OBJECT = 5
};
```

이 구조는 Unity에는 맞다.

- 맵 데이터를 받아야 한다.
- 서버 object를 생성해야 한다.
- GameObject 위치를 갱신해야 한다.

하지만 ESP32에는 맞지 않는다.

ESP32는 Unity prefab도 모르고, RenderManager도 없고, GameObject도 없다.  
ESP32가 필요한 것은 "화면에 object를 어떻게 그릴지"가 아니라 "어떤 route를 실행해야 하는지"다.

그래서 packet을 분리했다.

```text
Unity protocol:
  Server world를 화면에 복제하기 위한 통신

RobotProtocol:
  실제 로봇 등록, 경로 명령, 상태 보고를 위한 통신
```

## 3. 짚고 넘어갈 개념: 서버가 모터를 직접 제어해야 할까?

가장 많이 헷갈린 부분이 이것이었다.

Server가 ESP32에게 이렇게 보내는 구조도 가능하다.

```text
FORWARD 0.5m
TURN_LEFT 90도
PWM 120
STOP
```

하지만 이 프로젝트에서는 그렇게 하지 않았다.

대신 Server는 다음처럼 보낸다.

```text
ROUTE_COMMAND
  node 12
  node 13
  node 17
  node 21
```

이유는 Server와 Robot의 책임이 다르기 때문이다.

![Server and Robot Responsibility Split](assets/responsibility-split.svg)

Server는 여러 AGV를 같이 봐야 한다.

- 어떤 AGV에게 어떤 작업을 줄지
- 어떤 경로가 충돌이 적은지
- 어떤 노드/링크를 어느 시간에 예약할지
- 막힌 경로가 있으면 재계획할지

반면 ESP32는 자기 몸만 정확히 제어하면 된다.

- 바퀴가 실제로 얼마나 굴렀는지
- 모터 출력이 부족하지 않은지
- 배터리가 낮지 않은지
- encoder 값이 이상하지 않은지
- WiFi가 잠깐 끊겨도 안전하게 멈출지

그래서 Server는 route를 결정하고, ESP32는 route를 실제 motion으로 바꾸는 구조가 더 맞다고 판단했다.

## 4. RobotProtocol packet format

RobotProtocol도 TCP 위에서 동작한다.

그래서 Unity protocol과 마찬가지로 맨 앞에는 `packetSize`를 둔다.

![RobotProtocol TCP Frame Format](assets/packet-frame.svg)

현재 header는 다음과 같이 설계했다.

```cpp
#pragma pack(push, 1)
struct PacketHeader
{
    uint16_t packetSize;
    uint16_t packetID;
    uint32_t agvID;
    uint32_t sequence;
};
#pragma pack(pop)
```

각 field의 역할은 다음과 같다.

```text
packetSize : TCP stream에서 packet 경계를 찾기 위한 전체 크기
packetID   : HELLO, STATUS, ROUTE_COMMAND 같은 packet 종류
agvID      : 어떤 AGV에 대한 packet인지
sequence   : 로그 추적과 디버깅용 번호
```

실제 서버 코드에서는 `TCPSession`이 앞의 `packetSize`를 제거하고 body만 넘긴다.  
그래서 body header는 따로 존재한다.

```cpp
struct PacketBodyHeader
{
    uint16_t packetID;
    uint32_t agvID;
    uint32_t sequence;
};
```

## 5. PacketID

RobotProtocol packet은 `uint16_t` 기반으로 관리한다.

```cpp
enum class PacketID : uint16_t
{
    ROUTE_COMMAND = 100,
    CANCEL_ROUTE = 101,

    STATUS = 200,
    ARRIVED = 201,

    PING = 300,
    PONG = 301,

    HELLO = 400,
    HELLO_ACK = 401,

    ERROR_PACKET = 500,
    EMERGENCY_STOP = 501
};
```

초기 Unity packet은 `uint8_t`였지만, RobotProtocol은 새로 분리하면서 `uint16_t`를 사용했다.  
나중에 packet 종류가 늘어나도 여유를 두기 위해서다.

## 6. HELLO handshake

ESP32가 서버에 연결되면 가장 먼저 `HELLO`를 보낸다.

```cpp
struct HelloPayload
{
    uint16_t protocolVersion = kProtocolVersion;
    ClientType clientType = ClientType::UNKNOWN;
    uint32_t requestedAgvID = 0;
};
```

이 packet에는 세 가지 의미가 있다.

- protocol version이 서버와 맞는지 확인
- 접속한 client가 ESP32인지 FakeRobot인지 구분
- 몇 번 AGV로 연결할지 요청

Server는 이것을 받고 `HELLO_ACK`를 돌려준다.

```cpp
struct HelloAckPayload
{
    uint16_t protocolVersion = kProtocolVersion;
    uint8_t accepted = 0;
    uint32_t assignedAgvID = 0;
    ErrorCode errorCode = ErrorCode::NONE;
};
```

실제 서버 처리 흐름은 다음과 같다.

```cpp
void NetworkManagerServer::HandleRobotHelloPacket(
    ClientProxy* _proxy,
    const RobotProtocol::PacketBodyHeader& _header,
    InputMemoryStream& _stream)
{
    RobotProtocol::HelloPayload hello;
    RobotProtocol::ReadHelloPayload(_stream, hello);

    const bool isRobotClient =
        hello.clientType == RobotProtocol::ClientType::ESP32 ||
        hello.clientType == RobotProtocol::ClientType::FAKE_ROBOT;

    uint32_t assignedAgvID =
        hello.requestedAgvID != 0 ? hello.requestedAgvID : _header.agvID;

    RobotProtocol::HelloAckPayload ack;
    ack.protocolVersion = RobotProtocol::kProtocolVersion;
    ack.assignedAgvID = assignedAgvID;

    const bool versionOK =
        hello.protocolVersion == RobotProtocol::kProtocolVersion;

    ObjectPtr agvObject =
        assignedAgvID != 0
            ? m_LinkingContext->GetObject(assignedAgvID)
            : nullptr;

    const bool agvOK =
        agvObject && agvObject->GetClassID() == ClassID::OBJ_AGV;

    RobotSessionPtr robotSession =
        std::make_shared<RobotSession>(_proxy->GetSession(), assignedAgvID);

    if (!versionOK)
    {
        ack.accepted = 0;
        ack.errorCode = RobotProtocol::ErrorCode::PROTOCOL_MISMATCH;
        robotSession->SendHelloAck(ack);
        return;
    }

    if (!agvOK)
    {
        ack.accepted = 0;
        ack.errorCode = RobotProtocol::ErrorCode::UNKNOWN_AGV;
        robotSession->SendHelloAck(ack);
        return;
    }

    ack.accepted = 1;
    ack.errorCode = RobotProtocol::ErrorCode::NONE;

    m_AgvIdToRobotSessionMap[assignedAgvID] = robotSession;
    m_ProxyToRobotSessionMap[_proxy] = robotSession;

    RobotManager::GetInstance().RegisterRobot(
        assignedAgvID,
        std::make_unique<ESP32RobotController>(robotSession)
    );

    robotSession->SendHelloAck(ack);
}
```

이렇게 하면 ESP32가 잘못된 AGV ID로 접속했을 때 서버가 바로 거절할 수 있다.

## 7. 연결 이후 sequence

전체 흐름은 다음과 같다.

![RobotProtocol Lifecycle](assets/protocol-sequence.svg)

텍스트로 보면 이렇게 된다.

```text
ESP32 -> Server : HELLO
Server -> ESP32 : HELLO_ACK

Server -> ESP32 : ROUTE_COMMAND
ESP32 -> Server : STATUS
ESP32 -> Server : ARRIVED

Server -> Unity : Replication update
```

여기서 재미있는 점은 ESP32가 Unity에 직접 말하지 않는다는 것이다.

```text
ESP32 -> Unity
```

가 아니라,

```text
ESP32 -> Server -> Unity
```

이다.

이렇게 해야 Unity는 항상 서버 기준의 world state를 보게 된다.

## 8. ROUTE_COMMAND

서버가 ESP32에게 route를 보낼 때는 `RobotSession::SendRoute()`를 사용한다.

```cpp
void RobotSession::SendRoute(const RoutePacket& routePacket)
{
    RobotProtocol::RouteCommandPayload payload;
    payload.routeID = m_NextRouteID++;
    payload.nodes = routePacket.nodes;

    OutputMemoryStream payloadStream;
    RobotProtocol::WriteRouteCommandPayload(payloadStream, payload);

    SendRobotPacket(
        RobotProtocol::PacketID::ROUTE_COMMAND,
        routePacket.agvID,
        payloadStream);
}
```

실제 packet body header는 공통 함수에서 붙인다.

```cpp
void RobotSession::SendRobotPacket(
    RobotProtocol::PacketID packetID,
    uint32_t agvID,
    OutputMemoryStream& payloadStream)
{
    OutputMemoryStream bodyStream;

    RobotProtocol::WritePacketBodyHeader(
        bodyStream,
        packetID,
        agvID,
        NextSequence());

    bodyStream.Write(payloadStream.GetBuffer(), payloadStream.GetLength());

    m_TCPSession->SendPacket(bodyStream);
}
```

그리고 `TCPSession::SendPacket()`이 마지막에 `packetSize`를 붙여서 실제 TCP socket으로 전송한다.

## 9. STATUS와 ARRIVED

ESP32는 route를 실행하면서 계속 `STATUS`를 보낸다.

```cpp
struct StatusPayload
{
    uint32_t currentNodeID = 0;
    uint32_t currentLinkID = 0;
    float progress = 0.0f;
    float x = 0.0f;
    float z = 0.0f;
    float heading = 0.0f;
    float velocity = 0.0f;
    float battery = 100.0f;
    RobotState state = RobotState::UNKNOWN;
};
```

현재는 실제 차체가 아직 오기 전이라 ESP32 코드에서 route progress를 임시로 증가시키는 방식으로 테스트했다.  
이건 실제 주행 코드가 아니라, 서버와 ESP32 사이의 protocol loop를 먼저 검증하기 위한 단계다.

나중에 차체, TB6612FNG, encoder가 연결되면 이 부분은 실제 센서 기반 값으로 바뀐다.

```text
현재:
  progress를 임시 계산
  STATUS 전송

차체 연결 후:
  encoder로 거리 계산
  heading 보정
  motor PWM 제어
  실제 STATUS 전송
```

## 10. FakeRobot을 만든 이유

FakeRobot은 UnityRobotController와 다르다.

```text
UnityRobotController:
  서버 내부에서 Unity simulation을 움직이던 controller

FakeRobot:
  서버 밖에서 TCP로 접속하는 ESP32 대체 프로그램
```

FakeRobot을 둔 이유는 하드웨어가 없어도 네트워크를 검증하기 위해서다.

- HELLO / HELLO_ACK 확인
- ROUTE_COMMAND 수신 확인
- STATUS 전송 확인
- ARRIVED 전송 확인
- Server -> Unity replication 확인

즉 FakeRobot은 나중에 사라지는 "가짜 기능"이라기보다, 하드웨어 문제와 서버 통신 문제를 분리해주는 테스트 도구다.

## 11. 현재 구조에서 얻은 것

이 구조를 만들면서 프로젝트가 단순 Unity simulation에서 실제 AGV system에 가까워졌다.

```text
Before:
  Server -> Unity
  Unity에서 가상 AGV가 움직임

After:
  Server -> ESP32/FakeRobot -> STATUS -> Server -> Unity
  Unity는 서버가 복제한 상태를 화면에 그림
```

즉 최종적으로는 Unity가 로봇을 움직이는 것이 아니라, 실제 로봇 상태를 보여주는 디지털 트윈이 된다.

## 12. 정리

RobotProtocol을 따로 만든 이유는 다음과 같다.

- Unity와 ESP32의 역할이 다르다.
- 실제 로봇은 GameObject가 아니라 route command와 status report가 필요하다.
- TCP stream 특성 때문에 size-first frame이 필요하다.
- HELLO handshake로 client type과 AGV ID를 명확히 한다.
- Server는 fleet-level 판단을 하고, ESP32는 hardware-level 제어를 한다.
- FakeRobot으로 하드웨어 없이도 protocol을 검증할 수 있다.

아직 실제 차체 주행은 남아 있다.  
하지만 네트워크 관점에서는 이미 중요한 구조가 잡혔다.

> Server가 중앙 관제 역할을 하고, Unity와 ESP32가 각각 다른 client로 붙는 구조

이 구조가 만들어졌기 때문에, 차체가 도착하면 다음 단계는 서버를 다시 설계하는 것이 아니라 ESP32 내부의 motor/encoder 제어를 채우는 일이 된다.
