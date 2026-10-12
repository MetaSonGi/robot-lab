# 효기 Robot Lab — 웹 로봇 제어·통신 실습실 v0.1

실제 MuJoCo 물리 계산을 웹에서 조작하는 로컬 프로그램입니다. 공식 로봇 모델 네 종류와 로봇별 미션, 관절 목표값 제어, 응답 그래프, WebSocket 명령·상태 모니터를 포함합니다. 화면에서 관절을 움직이면 Python 시뮬레이터가 계산한 위치를 Three.js가 표시합니다.

## 바로 실행 — Windows

1. ZIP을 압축 해제합니다. ZIP 안의 `robot-lab` 폴더를 사용하세요.
2. Python **3.12 또는 3.13, 64비트**를 설치합니다. 3.14만 있으면 `start.bat`가 실행을 거부합니다(여러 버전이 있으면 3.13 → 3.12 순으로 자동 선택).
3. `start.bat`를 더블클릭합니다. 처음에는 인터넷으로 Python 패키지를 설치합니다.
4. 터미널에 출력된 `브라우저 주소`를 엽니다. 기본은 **http://127.0.0.1:8000**이며, 8000번이 사용 중이면 8010 → 8080 → 8888 → 9000 순으로 빈 포트를 자동 선택합니다. 포트를 고정하려면 `set ROBOT_LAB_PORT=8123` 후 실행하세요.
5. 종료하려면 터미널에서 Ctrl+C를 누릅니다.

## 외부 접속 · 웹버전 (GitHub Pages)

웹버전 주소: https://metasongi.github.io/robot-lab/ — 화면은 GitHub Pages가, 물리 계산은 이 PC의 Python 서버가 맡고 Cloudflare 임시 터널로 연결합니다.

- **서버 없이 열면 데모 모드**: 4개 로봇의 미션 자동 풀이를 미리 녹화해 재생합니다. 로봇 버튼으로 전환, 실행/일시정지/초기화로 재생 제어, 통신 로그에 당시 보낸 JSON 명령(TX)과 응답(ACK)이 시간에 맞춰 표시됩니다. 서버 설정의 **데모 보기**로 언제든 전환할 수 있습니다.
- **실시간 조작**:
  1. 서버 PC에서 `start_public.bat`를 실행합니다(`cloudflared` 필요: `winget install --id Cloudflare.cloudflared`). 실행할 때마다 터널 주소가 저장소 `tunnel` 브랜치의 `server.json`에 자동 게시됩니다(토큰은 게시하지 않음).
  2. 웹버전 **서버 설정**에 토큰만 한 번 입력하면 이 브라우저에 저장되고, 이후에는 열 때마다 자동 연결됩니다. 서버 주소는 선택 입력(다른 서버를 쓸 때만)입니다.
  3. 창에 출력되는 `토큰 포함 링크`를 공유하면 토큰 입력 없이 바로 연결됩니다(토큰은 주소창에서 바로 지워짐).

- 서버가 꺼져 있거나 토큰이 없으면 자동으로 데모를 재생합니다. 터널 주소는 실행마다 바뀌지만 자동 게시되므로 신경 쓸 필요가 없습니다. 토큰은 `.lab_token`에 저장되어 유지되며, 파일을 지우면 새로 발급됩니다(Git에는 올라가지 않음).
- 미션·모델을 바꿨다면 `.venv\Scripts\python.exe tools\record_demos.py`로 데모를 다시 녹화합니다(약 1분, `static/demo/*.json`).
- 서버는 토큰이 틀리면 접속을 거절하고(4401), 허용 출처는 같은 주소와 `https://metasongi.github.io`뿐입니다(`ROBOT_LAB_ALLOWED_ORIGINS`로 변경). 접속마다 시뮬레이션을 따로 계산하므로 동시 접속은 기본 4명입니다(`ROBOT_LAB_MAX_SESSIONS`).
- 로봇 선택 시 모델 데이터(14~20MB)를 받으므로 외부망에서는 첫 로딩이 몇 초 걸립니다.
- `static/`을 수정했다면 커밋 후 `deploy_pages.bat`로 웹버전을 갱신합니다.

Cursor에서는 압축 해제한 폴더를 열고 터미널에서 `start.bat`를 실행하면 됩니다. 브라우저는 Chrome 또는 Edge의 WebGL 하드웨어 가속을 켜세요. 별도 Node.js 설치는 필요 없습니다. Python 설치·인터넷 연결은 사용자 PC에서 필요합니다.

## Linux / Ubuntu / WSL

Python 3.12/3.13과 venv가 준비된 환경에서:

```bash
cd robot-lab
bash start.sh
```

Ubuntu에서 venv가 없다면 `sudo apt install python3-venv`로 설치합니다. WSL도 Windows 브라우저에서 `http://localhost:8000`을 열면 됩니다. 실제 GPU 렌더링은 브라우저에서 하고 서버는 MuJoCo 동역학 계산만 하므로 NVIDIA GPU가 필수는 아닙니다. 성능은 PC와 모델에 따라 달라집니다.

Docker가 이미 설치되어 있으면 `docker compose up --build`로 실행할 수 있습니다. Docker 구성은 제공하지만 이번 환경에서 Docker 빌드는 검증하지 않았습니다. Docker 이미지는 의존성 설치 시 인터넷이 필요하며 로컬 포트에만 바인딩합니다.

## 포함 기능

| 항목 | 지원 범위 |
|---|---|
| Go2 | 12개 모터 관절, PD 토크 제어 |
| RB-Y1 (Rainbow Robotics) | 한 몸통에 양팔이 달린 상반신형. 몸통 6 · 팔 7×2 · 그리퍼 2 · 머리 2 = 24축 위치 서보. 바퀴(속도 구동기)는 정지 유지 |
| G1 | 29개 구동 관절, 모델의 위치 서보 |
| Allegro Hand (Wonik Robotics) | 손가락 4개(검지·중지·약지·엄지) × 4관절 = 16축 위치 서보. 손 펴기·주먹·집기·V 자세 프리셋 |
| 미션 | Go2 목표 지점 이동, RB-Y1 컨베이어 피킹, G1 상자 창고 운반, Allegro 물체 쥐기. 자동 판정·점수 |
| 물리 | MuJoCo 중력, 접촉, 관절·모터 제한 |
| 화면 | 공식 모델 메시 기반 3D, 회전·확대, 관절 조작 |
| 제어 | 실행, 일시정지, 정지, 초기화, 자유 몸체 고정/해제 |
| 관찰 | 목표/실제 응답, 속도, 구동력, 접촉점, 모델 센서 배열 |
| 통신 | WebSocket JSON 명령, ack/error, 약 30Hz 목표 상태 전송 |
| 통신 실험 | 클라이언트 명령에 0~1000ms 지연, 0~80% 임의 누락 |
| 기록 | 최근 500개 통신 로그와 현재 상태 JSON 다운로드 |
| 데이터 수집 | 로봇별 실행 데이터 누적 저장(30 Hz 상태 CSV + 명령 JSONL), 웹 패널·API·ZIP 내보내기 |
| 예제 | Python 제어 클라이언트, 미션 자동 풀이 클라이언트, 선택적 ROS 2 JointState 브리지 |

### 구현 범위와 한계

- 미션용 제어기(트롯 보행 생성기, 감쇠 최소제곱 역기구학, 가상 대차)는 **실습용 단순 구현**입니다. 학습된 보행 정책이나 전신 균형 제어기가 아닙니다.
- 미션의 **보조 기능**(Go2 균형 보조 힘, RB-Y1/G1 파지 흡착)은 물리 밖의 도움입니다. 화면의 `보조` 체크로 끄면 순수 물리만 남으며, 이 경우 미션 난도가 크게 올라갑니다. Allegro 쥐기는 보조 없이 실제 접촉으로 판정합니다.
- 실제 Unitree DDS/SDK2 제어는 아직 연결하지 않았습니다. 현재 JSON은 이 프로그램의 실습 프로토콜이며 제조사 프로토콜과 호환되지 않습니다.
- ROS 2 예제는 표준 `sensor_msgs/JointState`를 사용하지만 선택적 코드입니다. ROS 2가 없는 구축 환경에서는 구문 검사만 했습니다.
- 모델은 MuJoCo Menagerie의 시뮬레이션용 정의입니다. 실제 로봇과 완전히 동일한 동특성·펌웨어 동작을 보증하지 않습니다.
- 기본 몸체 고정은 매 스텝 자유 몸체 위치·속도를 유지하는 학습용 구속입니다. 물리적인 고정 지그 모델은 아닙니다. 미션 물체(큐브·상자 등)는 고정 대상이 아닙니다. G1은 고정을 풀면 균형 제어기가 없어서 넘어질 수 있습니다.
- G1/RB-Y1/Allegro의 서보 게인은 모델에 들어 있습니다. `gains` 명령은 Go2의 PD 제어에 적용됩니다(기본 kp 70, kd 3).
- 모델에 정의된 센서만 전송합니다. 센서 배열이 비어 있으면 해당 모델에 센서가 없는 것입니다. 구동력은 모터 전류나 외부 힘 센서 측정값이 아닙니다.
- 지연/누락은 **브라우저 명령 큐의 교육용 모사**이며 실제 네트워크 UDP/DDS 패킷 손실을 재현하지 않습니다. 정지·초기화·로봇 전환은 실험 지연을 우회하고 대기 명령을 취소합니다.
- 정지는 이 가상 세션의 계산을 멈춥니다. 실제 하드웨어 비상정지 장치가 아닙니다.
- 각 브라우저/클라이언트 연결은 독립된 로봇 세계입니다. Python 예제를 돌리면 웹 화면과 다른 세계를 제어합니다. 공동 제어 세션은 후속 기능입니다.
- 브라우저에서 임의 Python 코드를 실행하지 않습니다. Cursor/터미널에서 예제와 별도 Python 파일을 실행하세요.
- 웹버전(GitHub Pages)은 화면만 제공합니다. 물리 계산은 항상 `start_public.bat`를 실행한 PC가 하므로, 그 PC가 꺼지거나 창을 닫으면 접속이 끊깁니다.

## 첫 실습

1. Go2를 선택합니다. 기본 자세는 공식 모델의 home 키프레임입니다.
2. 몸체 고정을 켠 채 `FL_hip_joint` 목표를 `0.1 rad` 정도로 바꿉니다.
3. 실행을 누르고 선택 관절의 목표/실제 그래프를 관찰합니다.
4. JSON 명령으로 `{"type":"gains","kp":20,"kd":2}`를 보냅니다. 응답을 비교합니다.
5. 명령 지연을 500ms로 올리고 목표값을 다시 바꿉니다. QUEUE/TX/ACK/RX를 비교합니다.
6. 누락률을 올리고 DROP 로그와 목표값이 갱신되지 않는 경우를 관찰합니다.
7. Escape 또는 정지 버튼으로 가상 세션을 정지합니다. 초기화로 자세와 시간을 리셋합니다.

## 미션

로봇을 고르면 오른쪽 패널에 미션 카드(목표·진행 상황·빠른 명령·단계·JSON 예시)가 나옵니다. 빠른 명령 버튼은 JSON 명령을 그대로 보내므로 통신 로그에서 보낸 내용을 확인할 수 있습니다. 상태 메시지의 `mission` 필드에 `success`, `summary`와 미션별 값이 들어갑니다.

| 로봇 | 미션 | 성공 조건 |
|---|---|---|
| Go2 | 초록 목표 지점까지 보행 | 몸통이 목표 반경 0.3m 안 |
| RB-Y1 | 컨베이어로 지나가는 큐브를 집어 수거함에 넣기 | 3개 수거 |
| G1 | 팔레트 위 상자를 양손으로 들어 창고 박스에 넣기 | 상자가 박스 안 바닥에 놓임 |
| Allegro | 손바닥 위 물체를 손가락으로 감싸 쥐기 | 엄지 포함 3손가락 이상 접촉 1초 유지 |

미션 명령(공통 명령에 추가):

```json
{"type":"assist","enabled":false}
{"type":"goal","x":2.5,"y":1.0}
{"type":"gait","vx":0.4,"wz":0.3}
{"type":"conveyor","speed":0.05}
{"type":"ee_target","arm":"right","pos":[0.42,-0.25,0.95],"point_down":true}
{"type":"grasp","arm":"right","enabled":true}
{"type":"base","x":0.22,"y":0,"yaw":0,"height":0.56}
```

- `assist`: 미션 보조 켜기/끄기(Allegro 제외).
- `goal`, `gait` (Go2): 목표 좌표 ±4m, 전진 속도 `vx` -0.3~0.6 m/s, 회전 `wz` -1~1 rad/s. `gait`를 보내면 몸체 고정이 자동으로 풀립니다. 상태의 `goal_local`은 로봇 기준 목표 위치(앞 x, 왼쪽 y)입니다.
- `conveyor`, `ee_target`, `grasp` (RB-Y1): 벨트 속도 0~0.2 m/s. `ee_target`은 그립점 목표(월드 좌표, m)를 역기구학으로 관절 목표로 바꾸고 `ik_error_m`, `reachable`을 응답합니다. 오른팔은 벨트 높이까지 y≈-0.35~-0.15 구간에서 닿습니다. `grasp`는 그립점 5cm 안의 큐브를 잡으며, 다른 팔이 든 큐브면 넘겨받습니다(`handover`).
- `base`, `ee_target`, `grasp` (G1): 몸체 고정 상태에서 골반을 가상 대차처럼 이동합니다(`height` 낮추면 쪼그려 앉기). 이동·회전 중에는 다리가 발을 번갈아 드는 겉보기 스텝을 하고(상태 `mission.stepping`), 멈추면 선 자세로 돌아옵니다. 고정 중에는 대차가 체중을 지탱하므로 발-바닥 마찰을 낮춰 다리가 비틀린 채 남지 않게 합니다. `grasp`는 양손이 상자 옆면에 붙어 있어야 성공합니다.
- G1 주의: 반드시 `{"type":"play"}` 후 명령해야 움직입니다(정지 상태면 ack에 `warning`이 붙음). 고정을 풀면(`anchor:false`) 균형 제어기가 없어 넘어집니다. 팔레트(x≈0.5~0.76) 쪽으로 너무 가면 다리가 팔레트에 걸립니다.

자동 풀이 예제(서버 실행 중, 포트는 서버 출력값):

```bash
.venv\Scripts\python.exe examples\missions.py --robot rby1 --port 8010
.venv\Scripts\python.exe examples\missions.py --robot go2 --goal -1.5 2 --port 8010
```

## Python 통신 공부

서버를 실행한 상태에서 다른 터미널을 열고:

```bash
# Linux / WSL
.venv/bin/python examples/control.py --robot go2

# Windows
.venv\Scripts\python.exe examples\control.py --robot go2
```

`--robot rby1`, `--robot g1`, `--robot allegro`도 지원합니다. 서버가 8000이 아닌 포트로 떴다면 `--port 8010`처럼 지정하세요(ROS 2 브리지는 `-p port:=8010`). 메시지를 JSON으로 인코딩하고, 서버의 `model → state/ack/error` 응답을 파싱하는 예제입니다.

### WebSocket 명령

연결 주소: `ws://127.0.0.1:8000/ws`. 처음에는 Go2 모델과 상태가 옵니다. 첫 model 메시지는 메시 데이터를 포함해 약 14~20MB이므로 Python 클라이언트는 `max_size=64*1024*1024`를 사용합니다.

```json
{"type":"select","robot":"go2"}
{"type":"joint","joint":1,"target":0.1}
{"type":"play"}
{"type":"pause"}
{"type":"stop"}
{"type":"reset"}
{"type":"anchor","enabled":false}
{"type":"gains","kp":35,"kd":3}
```

로봇별 명령 설명은 웹 오른쪽 **JSON 명령 설명**(누르면 입력창에 채워짐)과 `model` 메시지의 `commands` 배열(`{"json": 예시, "desc": 설명}`)로 제공됩니다. 관절 ID는 로봇별로 다릅니다. 반드시 `model.joints`의 `id`, `name`, `min`, `max`, `unit`를 확인하세요. 목표 범위, NaN/무한대, 없는 관절 ID는 오류로 거절합니다. 관절의 각도는 rad, 속도는 rad/s, 회전 관절의 구동력은 N·m이며 직선 관절은 m, m/s, N입니다.

`state`에는 `seq`, `time`, `running`, `anchored`, `contacts`, `joints`, `positions`, `rotations`, `sensors`가 들어갑니다. 시간은 시뮬레이션 시간입니다. 전송 주기는 목표치이며 CPU/연결 속도에 따라 달라집니다. 현재 각 반복의 물리 시간은 모델 timestep에 맞춰 반올림하므로 실제 벽시계 시간과 일치하지 않을 수 있습니다.

## 실행 데이터 누적 수집

서버는 모든 세션(웹·Python 예제·ROS 2 브리지)의 시뮬레이션을 **실행(▶) 중일 때만** 로봇별로 쌓습니다. 한 실행은 로봇 선택(또는 초기화)부터 다음 선택·초기화·라벨 변경·접속 종료까지입니다.

```
data/<로봇>/<run_id>/
  meta.json       로봇·라벨·시작/종료 시각·프레임 수·시뮬레이션 시간·미션 성공 여부·최종 요약
  states.csv      30 Hz: frame, time, 몸체 자세(base_x..qz), 관절별 _q/_dq/_target/_torque, 센서, mission_success/summary/json
  commands.jsonl  받은 명령 + 결과/오류, frame(=states.csv 행 번호)과 시뮬레이션 시간
```

- 웹: 오른쪽 **04 / DATA** 패널에서 기록 상태, 로봇별 누적(실행 수·시간·성공), 최근 실행 CSV/명령 다운로드, 전체 ZIP을 받습니다. 라벨을 넣으면 그 이후가 새 실행으로 묶입니다.
- 명령: `{"type":"record","enabled":false}`로 이 세션 기록 끄기, `{"type":"record","enabled":true,"label":"kp20_지연500"}`로 라벨 붙이기.
- API(공개 모드는 헤더 `X-Lab-Token` 필요): `GET /api/data/summary`, `GET /api/data/runs?robot=g1&limit=50`, `GET /api/data/runs/<로봇>/<run_id>/states.csv|commands.jsonl|meta.json`, `GET /api/data/export?robot=g1`(ZIP).
- 분석 예제: `.venv\Scripts\python.exe examples\analyze_runs.py --server http://127.0.0.1:8010`(공개 서버는 `--token` 추가). 로봇별 누적 요약과 최근 실행의 관절 추종 RMS 오차를 출력하고, `--download data.zip`으로 전체를 받습니다. CSV는 utf-8-sig라 엑셀·pandas에서 바로 열립니다.
- 환경변수: `ROBOT_LAB_RECORD=0`(기본 끄기), `ROBOT_LAB_DATA_DIR`(저장 폴더, 기본 `data/`), `ROBOT_LAB_DATA_MAX_MB`(전체 한도, 기본 2000), `ROBOT_LAB_RUN_MAX_MINUTES`(실행 하나 최대 길이, 기본 30). 한도에 닿으면 기록만 멈추고 시뮬레이션은 계속됩니다.
- `data/`는 Git에 올라가지 않습니다. 공개 모드에서는 토큰을 가진 접속자 모두가 데이터를 조회할 수 있습니다.

## 선택적 ROS 2 실습

ROS 2 환경을 source한 터미널에서 `rclpy`, `sensor_msgs`, `websockets`를 사용할 수 있어야 합니다. 시스템 ROS Python과 일반 venv가 호환되지 않을 수 있으므로 ROS 환경의 Python을 사용하세요.

```bash
python3 examples/ros2_bridge.py --ros-args -p robot:=go2
ros2 topic echo /robot_lab/joint_states
ros2 topic pub --once /robot_lab/joint_targets sensor_msgs/msg/JointState "{name: ['FL_hip_joint'], position: [0.1]}"
```

브리지는 별도 세션을 만들고 실행합니다. `/robot_lab/joint_states`는 실제 계산된 상태, `/robot_lab/joint_targets`는 명령입니다. 실물 로봇에는 연결되지 않습니다. 이 예제의 런타임 검증은 사용자 ROS 환경에서 추가로 필요합니다.

## 개발 구조

- `app.py`: 모델 로더, 관절 제어, 물리 루프, WebSocket/API
- `static/`: 웹 화면, 로컬 Three.js 모듈
- `vendor/menagerie/`: 공식 MJCF·메시·라이선스 원본
- `missions.py`: 미션 장면 조립(MjSpec), 판정, 보행 생성기·역기구학·흡착 보조
- `examples/control.py`: Python 통신 예제
- `examples/missions.py`: 4개 미션 자동 풀이(WebSocket)
- `examples/ros2_bridge.py`: 선택적 ROS 2 브리지
- `recorder.py`, `examples/analyze_runs.py`: 실행 데이터 누적 수집과 조회·분석 예제
- `tests/test_data_recording.py`: 실행 단위 기록·끄기·API·토큰·경로 보호·용량 한도 검증
- `tests/test_lab.py`: 물리 응답·프로토콜 검증
- `tests/test_missions.py`, `tests/mission_helpers.py`: 미션 끝까지 수행 검증
- `requirements.txt`: 검증한 주요 패키지 버전
- `start.bat`, `start.sh`: 실행 파일
- `start_public.bat`, `public.py`: 외부 공개 실행(서버 + Cloudflare 터널 + 토큰)
- `deploy_pages.bat`: `static/`을 gh-pages 브랜치(웹버전)로 배포
- `tools/record_demos.py`, `static/demo/`: 웹버전 데모 녹화 도구와 녹화 파일
- `tests/test_demo_files.py`: 데모 파일이 현재 모델과 맞는지 검증
- `tests/test_public_access.py`: 토큰·허용 출처·동시 접속 제한 검증
- `Dockerfile`, `compose.yaml`: 로컬 컨테이너 구성

Cursor에서 `app.py`와 `static/app.js`를 중심으로 확장하면 됩니다. 첫 확장 목표는 브라우저와 Python의 공유 세션, 그다음 공식 unitree_mujoco DDS 브리지 연결입니다. 실물 전환 전에는 모델 관절 순서·단위·제한 및 지원 메시지를 각각 검증해야 합니다.

## 검증

Windows 11, Python 3.13, MuJoCo 3.15.0에서 네 로봇 로드와 실제 관절 응답, 초기화·일시정지, 잘못된 명령 거절, 독립 세션, 메시 인덱스, HTTP/WebSocket 통신, 4개 미션 성공(Go2는 보조 끄고도)과 양팔 큐브 넘겨주기를 자동 검사했습니다. `examples/missions.py`로 실행 중인 서버에서도 4개 미션 성공을 확인했습니다. G1 이동 중 양발 교대 들기와 정지 후 다리 복귀, 실행 데이터 기록·조회 API·용량 한도도 자동 검사합니다. ROS 2 런타임, Docker 빌드는 검증하지 않았습니다.

테스트 재실행:

```bash
.venv/bin/python -m pip install pytest httpx
.venv/bin/python -m pytest -q
```

## 출처·라이선스

모델 원본: https://github.com/google-deepmind/mujoco_menagerie

Go2/G1 모델은 BSD-3-Clause, RB-Y1은 Apache-2.0, Allegro Hand는 BSD 계열(SimLab)이며 각 폴더 LICENSE를 따릅니다. 원본 README 및 LICENSE를 패키지에 포함했습니다. 정확한 소스 커밋은 `MODEL_SOURCE.txt`를 확인하세요. Three.js는 MIT 라이선스이며 `static/lib/LICENSE`에 포함했습니다. 메시 데이터는 로컬 번들에 들어 있어 모델 다운로드 과정 없이 실행됩니다. 첫 Python 의존성 설치 이후에는 외부 CDN 요청 없이 사용할 수 있습니다.

공식 후속 자료:
- MuJoCo: https://github.com/google-deepmind/mujoco
- Unitree SDK2 Python: https://github.com/unitreerobotics/unitree_sdk2_python
- Unitree MuJoCo: https://github.com/unitreerobotics/unitree_mujoco
