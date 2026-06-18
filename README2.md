# `omx_hand_teleop.launch.py` 구조 상세 설명

OMX-F 손 추적(hand-tracking) 텔레오퍼레이션 + 5종 고수준 스킬을 **하나의 launch로** 띄우는
원샷 런치 파일의 내부 구조를, 무엇을 불러오고 각각이 어떻게 구동되는지까지 정리한 문서입니다.

> 대상 파일: [open_manipulator_playground/launch/omx_hand_teleop.launch.py](open_manipulator_playground/launch/omx_hand_teleop.launch.py)

---

## 0. 한눈에 보기

```
ros2 launch open_manipulator_playground omx_hand_teleop.launch.py
        │
        ├─ (t=0초)   bringup        : omx_f.launch.py        → 실제 하드웨어 + ros2_control 컨트롤러
        ├─ (t=5초)   control        : main.py                → OmxMoveLBridge(브리지) + JointTrajectoryRelay(relay)
        └─ (t=20초)  cyclo_movel    : omx_controller.launch.py (controller_type=movel) → MoveL IK 컨트롤러
```

이 launch는 **직접 노드를 거의 만들지 않습니다.** 대신 3개의 큰 블록을 시간차를 두고 띄우고,
그중 실제 "이 리포가 책임지는 제어 로직"은 `control` 블록(= `main.py`) 하나입니다.

데이터 흐름(엔드 투 엔드):

```
Reachy Mini(외부 Pi)
   │  HTTP POST / WebSocket (JSON: delta·absolute·gripper·enable·skill·...)
   ▼
[main.py] OmxMoveLBridge ── MoveL ─────────────▶ [cyclo MoveL 컨트롤러]
   │                                                  │  IK
   │                                                  ├─ /leader/joint_trajectory ─┐
   │                                                  └─ /omx_movel_controller/current_pose
   │                                                                                │
   │                       [main.py] JointTrajectoryRelay ◀──────────────────────────┘
   │                                  │  /arm_controller/joint_trajectory
   │                                  ▼
   │                          [ros2_control] arm_controller ──▶ 실제 팔(다이나믹셀)
   │
   └─ GripperCommand action ─▶ [ros2_control] gripper_controller ──▶ 실제 그리퍼
```

---

## 1. 런치 최상위 구조

[omx_hand_teleop.launch.py](open_manipulator_playground/launch/omx_hand_teleop.launch.py)의
`generate_launch_description()`는 다음 3개를 반환 리스트에 담아 실행합니다
([L173-176](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L173-L176)):

| 순번 | 블록 | 실행 시점 | 정의 위치 |
|---|---|---|---|
| 1 | `bringup` | **즉시(t=0)** | [L57-70](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L57-L70) |
| 2 | `control` (= `main.py`) | `bridge_delay` **기본 5초 뒤** | [L85-129](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L85-L129) |
| 3 | `cyclo_movel` | `cyclo_delay` **기본 20초 뒤** | [L72-83](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L72-L83) |

`TimerAction`으로 2·3번을 지연 실행합니다([L174-175](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L174-L175)).

> ⚠️ **순서 주의:** 브리지(`control`, 5초)가 MoveL 컨트롤러(`cyclo_movel`, 20초)보다 **먼저** 뜹니다.
> 그래서 시작 후 약 15초 동안 브리지는 `/omx_movel_controller/current_pose`를 못 받아
> `No /current_pose yet; is cyclo_control + bringup running?` 경고를 주기적으로 찍습니다(정상 동작).
> cyclo가 올라오면 자동으로 해소됩니다.

---

## 2. 블록 ① `bringup` — 실제 하드웨어 + ros2_control

[open_manipulator_bringup/launch/omx_f.launch.py](open_manipulator_bringup/launch/omx_f.launch.py)를
include 하며, 다음 인자를 넘깁니다([L65-69](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L65-L69)):

- `port_name` : 기본 `/dev/ttyACM0`
- `start_rviz` : 기본 `false` → RViz 안 띄움
- `init_position` : **`false` (고정)** → 초기자세 이동 노드를 띄우지 않음

`omx_f.launch.py`가 실제로 띄우는 노드들:

| 노드 | 패키지/실행파일 | 역할 | 조건 |
|---|---|---|---|
| `control_node` | `controller_manager` / `ros2_control_node` | URDF(xacro) + `hardware_controller_manager.yaml`로 실제 다이나믹셀 하드웨어 인터페이스 구동 | `use_sim=false`일 때만([L149](open_manipulator_bringup/launch/omx_f.launch.py#L149)) |
| `robot_controller_spawner` | `controller_manager` / `spawner` | **`joint_state_broadcaster`, `arm_controller`, `gripper_controller`** 3개 컨트롤러 스폰(타임아웃 60초) | 항상([L152-166](open_manipulator_bringup/launch/omx_f.launch.py#L152-L166)) |
| `robot_state_publisher_node` | `robot_state_publisher` | URDF로 TF 발행 | 항상 |
| `joint_trajectory_executor` | `open_manipulator_bringup` | 초기자세 이동 | `init_position=true`일 때만 → **이 launch에선 `false`라 안 뜸**([L180](open_manipulator_bringup/launch/omx_f.launch.py#L180)) |
| `rviz_node` | `rviz2` | 시각화 | `start_rviz=true`일 때만 → **안 뜸** |

여기서 만들어지는 핵심 인터페이스:
- **`arm_controller`** : `/arm_controller/joint_trajectory` 토픽을 받아 팔을 움직임 (relay의 최종 목적지).
- **`gripper_controller`** : `/gripper_controller/gripper_cmd` **GripperCommand 액션 서버** (브리지가 그리퍼 명령을 보내는 곳).

> URDF는 [omx_f.urdf.xacro](open_manipulator_description/urdf/omx_f/omx_f.urdf.xacro)를 xacro로 생성하며,
> `port_name`/`use_mock_hardware`/`ros2_control_type=omx_f` 등이 xacro 인자로 전달됩니다
> ([L92-120](open_manipulator_bringup/launch/omx_f.launch.py#L92-L120)).

---

## 3. 블록 ② `control` — `main.py` (이 리포의 핵심)

[L85-89](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L85-L89)에서
`open_manipulator_playground` 패키지의 **`main.py`**를 노드로 실행하고,
[L89-128](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L89-L128)에서 수십 개의 파라미터를 주입합니다
(`http_port`, `control_mode`, 워크스페이스 `x/y/z_min/max`, 보간/스무딩 파라미터, `skill_pose_config` 등).

### 3.1 `main.py`가 불러오는 것

[main.py](open_manipulator_playground/scripts/main.py)는 **한 프로세스 안에서 2개의 노드**를 생성합니다
([L33-34](open_manipulator_playground/scripts/main.py#L33-L34)):

```python
bridge = OmxMoveLBridge()          # omx_movel_bridge.py
relay  = JointTrajectoryRelay()    # omx_joint_trajectory_relay.py
```

그리고 **2개의 SingleThreadedExecutor를 각각 별도 스레드에서** 돌립니다
([L37-49](open_manipulator_playground/scripts/main.py#L37-L49)):

| Executor | 노드 | 스레드 | 이유 |
|---|---|---|---|
| `relay_executor` | `relay` | 전용 데몬 스레드 | 100 Hz 조인트 명령 스트림을 격리 |
| `bridge_executor` | `bridge` | 메인 스레드 | 텔레옵 타이머/HTTP가 조인트 스트림을 지연시키지 못하게 |

→ 두 노드는 내부적으로 각자 단일 스레드라 별도 락이 필요 없고, 한쪽이 바빠도 다른 쪽 스트림이 끊기지 않습니다.

### 3.2 `OmxMoveLBridge` 내부 구동 방식

[omx_movel_bridge.py](open_manipulator_playground/scripts/omx_movel_bridge.py)는 한 노드 안에서
**3종류의 실행 컨텍스트**가 협력합니다:

1. **HTTP/WebSocket 서버 (데몬 스레드)** — [L586-600](open_manipulator_playground/scripts/omx_movel_bridge.py#L586-L600)
   - stdlib만으로 구현(`ThreadingHTTPServer` + 직접 짠 RFC 6455 WebSocket 프레이밍).
   - 포트는 `http_port`(이 launch 기본값 **18001**).
   - Reachy가 보내는 JSON 한 건을 `ingest_payload()`([L645](open_manipulator_playground/scripts/omx_movel_bridge.py#L645))로 파싱.
   - **HTTP 스레드는 절대 ROS publish를 하지 않습니다.** 공유 상태에 값을 접거나(델타/절대/그리퍼/enable),
     스킬·시퀀스·프리미티브는 큐에 enqueue만 합니다.
   - 메시지 타입: `delta`, `absolute`/`workspace`/`pose`, `gripper`, `enable`, `skill`, `primitive`, `sequence`, `stop`.

2. **ROS 타이머 (`bridge_executor` 스레드, `publish_rate`=20 Hz)** — `_on_timer()` [L851](open_manipulator_playground/scripts/omx_movel_bridge.py#L851)
   - 공유 상태(델타/절대 목표/그리퍼 요청/enable)를 락으로 스냅샷.
   - 절대 목표를 스무딩·보간([L953-990](open_manipulator_playground/scripts/omx_movel_bridge.py#L953-L990))한 뒤 **MoveL 메시지를 발행**.
   - 그리퍼 요청이 있으면 `_send_gripper()`로 GripperCommand 액션 전송.
   - 스킬이 팔을 점유 중(`_skill_active`)이면 텔레옵 입력은 버리고, 끝나면 현재 포즈로 재시드.

3. **스킬 워커 스레드 (데몬)** — `_skill_worker()` [L1132](open_manipulator_playground/scripts/omx_movel_bridge.py#L1132)
   - 큐에서 작업을 꺼내 **블로킹 방식으로** 스킬/시퀀스/프리미티브를 실행.
   - **고수준 스킬이 MoveL/그리퍼를 publish하는 유일한 곳.** (타이머=텔레옵, 워커=스킬)
   - 등록된 스킬 11종([L556-568](open_manipulator_playground/scripts/omx_movel_bridge.py#L556-L568)):
     `move_to_home`, `pick_and_handover`, `pick_pointed_object`, `point_to_detected_object`,
     `approach_detected_object`, `push_detected_object`, `pick_detected_object`,
     `pick_and_place_detected_object`, `sort_detected_object`, `place_to_zone`, `push_object`.

**ROS 인터페이스**([L570-577](open_manipulator_playground/scripts/omx_movel_bridge.py#L570-L577)):

| 종류 | 이름(기본값) | 방향 | 상대 |
|---|---|---|---|
| Publisher | `/omx_movel_controller/movel` (`MoveL`) | 송신 | cyclo MoveL 컨트롤러 |
| Subscription | `/omx_movel_controller/current_pose` (`PoseStamped`) | 수신 | cyclo MoveL 컨트롤러 |
| Action Client | `/gripper_controller/gripper_cmd` (`GripperCommand`) | 송신 | bringup의 gripper_controller |
| Publisher | `~/skill_status` (`String`) | 송신 | 상태 모니터링 |

> 스킬 포즈/타이밍/그리퍼 값은 `skill_pose_config`로 주입되는
> [config/omx_skill_poses.yaml](open_manipulator_playground/config/omx_skill_poses.yaml)에서 옵니다
> (그리퍼 `closed=0.0`, `open=1.1` — OMX 규약).

### 3.3 `JointTrajectoryRelay`

[omx_joint_trajectory_relay.py](open_manipulator_playground/scripts/omx_joint_trajectory_relay.py)는 매우 단순합니다:

- `/leader/joint_trajectory`(cyclo가 발행) 구독 → `/arm_controller/joint_trajectory`로 **그대로 재발행**
  ([L25-36](open_manipulator_playground/scripts/omx_joint_trajectory_relay.py#L25-L36)).
- `ros-jazzy-topic-tools`에 의존하지 않으려고 직접 만든 토픽 릴레이.

---

## 4. 블록 ③ `cyclo_movel` — MoveL IK 컨트롤러 (외부 패키지)

[L72-83](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L72-L83)에서
**`cyclo_motion_controller_ros`** 패키지의 `omx_controller.launch.py`를 `controller_type:=movel`로 include 합니다.

> ⚠️ 이 패키지는 **이 리포지토리에 포함되어 있지 않은 외부 의존성**입니다(워크스페이스에 별도 설치 필요).

브리지 관점에서 이 컨트롤러의 역할:
- `/omx_movel_controller/movel`(브리지가 보낸 절대 EE 목표)을 받아 **IK를 풀고**,
- 결과 조인트 궤적을 `/leader/joint_trajectory`로 발행(→ relay가 `arm_controller`로 전달),
- 현재 EE 포즈를 `/omx_movel_controller/current_pose`로 발행(→ 브리지가 시드/재시드에 사용).

---

## 5. 기동 타임라인

```
t=0s    bringup 시작
        ├─ ros2_control_node (하드웨어)
        ├─ robot_state_publisher
        └─ spawner → joint_state_broadcaster / arm_controller / gripper_controller (최대 60s 대기 가능)
t=5s    main.py 시작
        ├─ OmxMoveLBridge: HTTP/WS 서버(:18001), 20Hz 타이머, 스킬 워커 스레드
        └─ JointTrajectoryRelay: /leader/joint_trajectory → /arm_controller/joint_trajectory
        (이 시점~t≈20s: current_pose 없어 "No /current_pose yet" 경고 — 정상)
t=20s   cyclo_movel 시작
        └─ /omx_movel_controller/current_pose 발행 시작 → 브리지 시드 완료, 텔레옵 가능
```

---

## 6. 주요 런치 인자(기본값)

[L131-172](open_manipulator_playground/launch/omx_hand_teleop.launch.py#L131-L172) 발췌:

| 인자 | 기본값 | 의미 |
|---|---|---|
| `http_port` | `18001` | 브리지 HTTP/WS 포트 |
| `enable_websocket` | `true` | WebSocket 업그레이드 허용 |
| `control_mode` | `absolute` | Reachy 절대 좌표 모드 |
| `publish_rate` | `20.0` | 브리지 타이머 주파수(Hz) |
| `x_min/x_max` | `0.08 / 0.28` | 워크스페이스 X 한계(m) |
| `y_min/y_max` | `-0.16 / 0.16` | 워크스페이스 Y 한계(m) |
| `z_min/z_max` | `0.05 / 0.28` | 워크스페이스 Z 한계(m) |
| `port_name` | `/dev/ttyACM0` | 하드웨어 시리얼 포트 |
| `skill_pose_config` | `config/omx_skill_poses.yaml` | 스킬 포즈/타이밍/그리퍼 설정 |
| `start_rviz` | `false` | RViz 표시 여부 |
| `cyclo_delay` | `20.0` | cyclo 컨트롤러 시작 지연(초) |
| `bridge_delay` | `5.0` | main.py 시작 지연(초) |

(나머지 보간/스무딩/입력매핑 인자는 브리지 파라미터로 그대로 전달됩니다.)

---

## 7. 실행

```bash
ros2 launch open_manipulator_playground omx_hand_teleop.launch.py
# 포트/시리얼 변경 예시
ros2 launch open_manipulator_playground omx_hand_teleop.launch.py \
  http_port:=8000 port_name:=/dev/ttyACM1 start_rviz:=true
```

**의존성 체크포인트**
- `cyclo_motion_controller_ros` 패키지가 워크스페이스에 빌드/설치되어 있어야 함(외부 의존).
- `robotis_interfaces`(MoveL 메시지)가 설치되어 있어야 브리지가 임포트됨.
- 실제 하드웨어가 `port_name`에 연결되어 있어야 bringup의 ros2_control_node가 정상 동작.
