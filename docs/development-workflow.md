# Windows, WSL and Git workflow

## 무엇이 공유되고 무엇이 공유되지 않는가

| 항목 | 다른 환경에 자동 공유되는가? |
|---|---|
| Codex/채팅 세션의 대화 기억 | 아니오 |
| commit하지 않은 로컬 변경 | 아니오, 별도 clone끼리는 보이지 않음 |
| `build/`, IDE cache, serial log | 아니오, Git에 넣지 않음 |
| commit 후 원격에 push한 source와 문서 | 예 |
| 원격 branch와 pull한 commit history | 예 |

이 프로젝트에서는 `README.md`, `AGENTS.md`, `docs/current-status.md`가 새 환경의 공용 시작점이다.

## 권장 구성

### 방법 A: WSL clone 하나 사용

서버 개발에는 이 방식이 가장 단순하다.

- 저장소는 WSL filesystem에 둔다.
- Windows의 VS Code/IDE는 Remote WSL로 같은 저장소를 연다.
- build와 server 실행은 WSL에서 한다.
- 두 Codex/IDE가 같은 working tree의 같은 파일을 동시에 수정하지 않는다.

이 방식은 파일이 즉시 같지만, 동시에 편집하면 서로의 미완성 변경을 덮을 수 있다.

### 방법 B: Windows와 WSL에 clone을 각각 둔다

Git 원격을 동기화 지점으로 쓴다. 동시에 작업할 때는 환경 또는 작업별 branch를 나눈다.

예:

```text
work/wsl-server-docs
work/windows-unity-viewer
```

native Windows에서는 현재 C++ server가 POSIX socket 때문에 바로 빌드되지 않는다. Windows clone은 Unity/문서 작업에 쓰거나, server build는 WSL에서 수행한다.

## 작업 시작

먼저 현재 변경을 확인한다.

```bash
git status --short --branch
git fetch origin
git pull --ff-only
```

다른 환경이 이미 같은 기능을 수정 중이면 같은 branch에서 동시에 시작하지 말고 새 작업 branch를 만든다.

```bash
git switch -c work/wsl-<topic>
```

Windows 쪽이라면 이름을 `work/windows-<topic>`처럼 구분할 수 있다. branch 이름 자체보다 한 branch의 작업 범위를 작게 유지하는 것이 중요하다.

## 작업 종료와 handoff

1. build 또는 해당 영역의 검증을 수행한다.
2. milestone, blocker, 다음 작업이 바뀌었다면 `docs/current-status.md`를 갱신한다.
3. 의도한 파일만 stage한다.
4. staged diff와 secret 포함 여부를 확인한다.
5. commit하고 원격 branch로 push한다.

```bash
git status --short
git add <변경한-파일>
git diff --cached --check
git diff --cached --stat
git commit -m "<type>: <변경 요약>"
git push -u origin HEAD
```

다른 환경에서는 해당 branch를 받아 이어서 작업한다.

```bash
git fetch origin
git switch <branch-name>
git pull --ff-only
```

## 충돌을 줄이는 규칙

- 같은 source 파일을 두 환경에서 동시에 수정하지 않는다.
- 한쪽에서 push하기 전에 다른 쪽의 미완성 변경을 억지로 pull하지 않는다.
- 임시 상태를 공유해야 하면 깨진 코드를 기본 branch에 올리지 말고 작은 작업 branch와 명확한 commit 메시지를 쓴다.
- 큰 텍스트 덤프 대신 실제 source tree와 짧은 문서를 공유한다.
- `.gitattributes`의 line-ending 규칙을 유지해 CRLF/LF만 바뀐 diff를 막는다.
- `build/`, `.vs/`, IDE cache, 개인 설정은 공유하지 않는다.

## 문서 인수인계 형식

`docs/current-status.md`에는 장문의 작업 일지를 쌓지 않는다. 아래 네 가지가 바뀔 때만 현재 값을 갱신한다.

```text
Last verified:
Completed milestone:
Known blocker:
Next single task:
```

세부 설계 결정이 오래 유지돼야 하면 `architecture.md` 또는 해당 기술 문서에 옮긴다. 일회성 console log는 Git에 넣지 않는다.

## Secret 처리

실제 SSID 비밀번호, token, API key, 인증서 private key는 어떤 branch에도 commit하지 않는다.

commit 전에는 값이 아니라 의심 파일 이름만 찾는 방식으로 확인할 수 있다.

```bash
git grep -Il -E 'password|passwd|api[_-]?key|secret|ssid'
```

검색 결과는 사람이 확인하되 비밀값을 채팅이나 issue에 붙여 넣지 않는다. 실제 firmware에는 다음 형태를 사용한다.

```text
secrets.example.h   # placeholder만 포함, Git 추적
secrets.h           # 실제 값, .gitignore
```

비밀값이 이미 commit된 경우 현재 파일에서 지우는 것만으로 끝나지 않는다.

1. 해당 credential을 먼저 변경하거나 폐기한다.
2. 현재 source를 placeholder 방식으로 바꾼다.
3. 원격 저장소 공개 범위와 clone 사용자를 확인한다.
4. 필요하면 별도 승인 후 Git history를 정리하고 모든 clone을 다시 동기화한다.

history rewrite는 기존 commit hash와 다른 환경의 branch를 바꾸므로 독단적으로 수행하지 않는다.
