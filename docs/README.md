# Project documentation

이 디렉터리는 설계 기준, 현재 상태, 실차 기록, 발표용 자료를 함께 보관한다. 모든 문서가 같은 권위를 갖는 것은 아니다.

## 처음 읽는 순서

1. [현재 상태](current-status.md)
2. [아키텍처](architecture.md)
3. [통신 프로토콜](CommunicationProtocol.md)
4. [실물 AGV 연동](physical-agv-integration.md)
5. [Windows/WSL/Git 작업 흐름](development-workflow.md)
6. 필요할 때 [WHCA* 코드 해설](whca-code-walkthrough.md)

## 문서 분류

| 문서 | 성격 | 용도 |
|---|---|---|
| [current-status.md](current-status.md) | Current | 구현·실차 검증 상태와 다음 우선 작업의 단일 기준 |
| [architecture.md](architecture.md) | Canonical | 시스템 경계, 핵심 책임과 실행 흐름 |
| [CommunicationProtocol.md](CommunicationProtocol.md) | Canonical | Unity/RobotProtocol frame과 payload 설계 |
| [physical-agv-integration.md](physical-agv-integration.md) | Current | 실차 배선·보정·안전·통합 순서 |
| [development-workflow.md](development-workflow.md) | Current | Windows/WSL 간 Git 동기화와 인수인계 |
| [whca-code-walkthrough.md](whca-code-walkthrough.md) | Canonical/Guide | WHCA* 계열 경로 계획 구현의 상세 해설 |
| [PortfolioGuide.md](PortfolioGuide.md) | Historical/Publishing | 차체 완성 전 작성된 포트폴리오 초안 |
| `velog-network-*.md` | Historical/Publishing | 게시용 네트워크 글과 시리즈 구성안 |
| [assets/](assets/) | Supporting | 문서와 발표용 SVG 자산 |

Historical/Publishing 문서와 일부 SVG에는 “차체 도착 전” 상태가 남아 있다. 현재 구현 판단에는 사용하지 않고, 발표 자료를 다시 만들 때 최신 상태에 맞춰 갱신한다.

## Source of truth 우선순위

정보가 서로 다르면 다음 순서로 확인한다.

1. 현재 빌드되는 source code와 사용자가 실차에서 검증한 기록
2. protocol source인 `Shared/Protocol.hpp`, `Shared/PacketSerializer.*`
3. 이 디렉터리의 Canonical 문서
4. `current-status.md`의 진행·검증 현황
5. 포트폴리오, Velog, 루트 텍스트 스냅샷

문서와 코드가 다르면 조용히 추측하지 말고 차이를 기록한 뒤 어느 쪽을 갱신할지 결정한다.

## 유지보수 규칙

| 변경 | 함께 확인할 문서 |
|---|---|
| module 책임 또는 실행 흐름 | `architecture.md`, `current-status.md` |
| packet ID, payload, framing | `CommunicationProtocol.md`, 실물 firmware 호환성 |
| GPIO, 배선, 보정값, 안전 절차 | `physical-agv-integration.md`, `current-status.md` |
| milestone 또는 blocker | `current-status.md` |
| Windows/WSL branch 운용 | `development-workflow.md` |

- 현재 상태는 여러 문서에 복사하지 않고 `current-status.md`에 모은다.
- 문서 링크는 저장소 상대경로로 쓴다. 특정 PC의 `/home/...` 또는 `C:\...` 경로를 넣지 않는다.
- `build/`, IDE 설정, 개인 설정, 인증정보는 문서 공유 대상이 아니다.
- 중요한 변경은 코드와 관련 문서를 같은 branch/commit 흐름으로 전달한다.

Last verified: 2026-08-06, implementation base commit `c56ada3` on `old-new-combined`.
