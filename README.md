# Doosan E0509 Robot Arm Controller

ROS2 기반 두산 E0509 로봇암 제어 시스템

## 프로젝트 개요

PyQt5 GUI를 통해 사용자로부터 목표 좌표를 입력받아 ROS2와 MoveIt2를 활용하여 두산 E0509 로봇암의 말단(End-Effector)을 제어하는 시스템입니다.

### 주요 기능

- 다중 목표점 순차 이동
- 절대/상대 좌표 모드
- 속도/가속도 사용자 설정
- 실시간 로봇 상태 모니터링
- 관절 각도 및 말단 위치 표시
- 긴급 정지 기능
- RViz2 시각화 연동

---

## 개발 환경

- **OS**: Ubuntu 22.04 LTS (WSL2 지원)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **주요 라이브러리**:
  - PyQt5 (GUI)
  - MoveIt2 (Motion Planning)
  - rclpy (ROS2 Python Client)

---

## 빠른 설치 가이드 (처음부터 끝까지)

### 사전 준비

- Ubuntu 22.04 (WSL2 또는 Native)
- 인터넷 연결
- 약 10GB 디스크 여유 공간

### 1단계: 프로젝트 클론

```bash
# 홈 디렉토리로 이동
cd ~

# 프로젝트 클론
git clone -b main https://github.com/yumijipsaa/ros2_ws.git

# 워크스페이스로 이동
cd ros2_ws
```

### 2단계: ROS2 Humble 설치

```bash
# 시스템 업데이트
sudo apt update

# 로케일 설정
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 저장소 추가
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치 (시간 소요: 약 5-10분)
sudo apt update
sudo apt install ros-humble-desktop-full -y

# 개발 도구
sudo apt install ros-dev-tools -y

# 환경 설정 (.bashrc에 자동 추가)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3단계: 필수 패키지 설치

```bash
# MoveIt2
sudo apt install ros-humble-moveit -y

# PyQt5 (GUI)
sudo apt install python3-pyqt5 -y

# 추가 ROS2 패키지
sudo apt install ros-humble-joint-state-publisher -y
sudo apt install ros-humble-joint-state-publisher-gui -y
sudo apt install ros-humble-robot-state-publisher -y
sudo apt install ros-humble-rviz2 -y
sudo apt install ros-humble-xacro -y

# Python 도구
sudo apt install python3-pip -y
pip3 install numpy

# Poco 라이브러리 (Doosan Robot용)
sudo apt install libpoco-dev libprotobuf-dev protobuf-compiler -y
```

### 4단계: WSL GUI 설정 (WSL 사용자만)

```bash
# GUI 렌더링을 위한 환경 변수 설정
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc
source ~/.bashrc
```

### 5단계: Doosan Robot 패키지 설치

```bash
# src 디렉토리로 이동
cd ~/ros2_ws/src

# Doosan Robot2 패키지 클론
git clone -b humble-devel https://github.com/DoosanRobotics/doosan-robot2.git

# 워크스페이스로 돌아가기
cd ~/ros2_ws
```

### 6단계: 의존성 설치

```bash
# rosdep 초기화 (처음 한 번만)
sudo rosdep init
rosdep update

# 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y
```

### 7단계: 빌드

```bash
cd ~/ros2_ws

# 전체 빌드 (시간 소요: 약 3-5분)
colcon build

# 환경 설정
source install/setup.bash
```

---

## 실행 방법

### 방법 A: 2개 터미널 사용 (권장)

#### 터미널 1: 시뮬레이터 실행

```bash
cd ~/ros2_ws
source install/setup.bash

# 환경 변수 설정 (WSL)
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:0

# 시뮬레이터 실행
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch dsr_moveit_config_e0509 start.launch.py mode:=virtual gui:=true model:=e0509
```

#### 터미널 2: 컨트롤러 GUI 실행

```bash
cd ~/ros2_ws
source install/setup.bash

# 환경 변수 설정 (WSL)
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:0
export QT_X11_NO_MITSHM=1

# GUI 실행
ros2 run doosan_controller robot_control_node
```

### 방법 B: 빠른 실행 스크립트

#### 시뮬레이터 실행 스크립트 생성

```bash
nano ~/run_simulator.sh
```

다음 내용 입력:

```bash
#!/bin/bash
cd ~/ros2_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:0
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual host:=127.0.0.1 model:=e0509
```

실행 권한 부여:

```bash
chmod +x ~/run_simulator.sh
```

#### GUI 실행 스크립트 생성

```bash
nano ~/run_gui.sh
```

다음 내용 입력:

```bash
#!/bin/bash
cd ~/ros2_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
ros2 run doosan_controller robot_control_node
```

실행 권한 부여:

```bash
chmod +x ~/run_gui.sh
```

**실행**:

```bash
# 터미널 1
~/run_simulator.sh

# 터미널 2
~/run_gui.sh
```

---

## 📖 사용 방법

### 1. 시스템 시작

1. **시뮬레이터 실행** (터미널 1)
   - RViz2 창이 열리고 로봇 모델이 표시됨
   - 초기 로딩에 10-20초 소요

2. **GUI 실행** (터미널 2)
   - Doosan E0509 Robot Arm Controller 창이 열림

3. **연결 확인**
   - GUI 우측 상단 **Connection Status**가 "Connected" (녹색)으로 변경되면 준비 완료
   - 약 5초 소요

### 2. 좌표 입력 및 실행

#### 제어 패널 (GUI 왼쪽)

1. **Coordinate Mode 선택**
   - `Absolute Coordinates`: 로봇 베이스 기준 절대 좌표
   - `Relative Coordinates`: 현재 위치 기준 상대 좌표

2. **Target Position 입력** (단위: mm)
   ```
   X: 400
   Y: 0
   Z: 500
   ```

3. **Add Point** 버튼 클릭
   - 테이블에 좌표 추가됨
   - 여러 개의 목표점을 순차적으로 추가 가능

4. **Motion Parameters** (선택사항)
   - ☑ Set Velocity (m/s): 0.1 ~ 1.0
   - ☑ Set Acceleration (m/s²): 0.5 ~ 2.0

5. **Execute Motion** 버튼 클릭
   - 로봇이 순차적으로 목표점으로 이동
   - RViz에서 로봇 움직임 확인

### 3. 상태 모니터링

#### 상태 패널 (GUI 오른쪽)

- **Connection Status**: 
  - 🟢 Connected: 정상 연결
  - 🔴 Disconnected: 연결 끊김

- **Status**: 
  - Idle: 대기 중
  - Moving: 이동 중

- **Joint Angles**: 6개 관절의 현재 각도 (degrees)
  - Joint 1-6의 실시간 각도 표시

- **Current Position**: 말단 위치 (Base Frame 기준, mm)
  - X, Y, Z 좌표 실시간 표시

- **Real-time Log**: 시스템 동작 로그
  - 시간별 동작 기록
  - 에러 메시지 표시

### 4. 긴급 정지

- **Emergency Stop** 버튼 (빨간색) 클릭
- 로봇이 즉시 정지
- 다시 실행하려면 새로운 좌표 입력 필요

---

## 💡 사용 예시

### 예시 1: 단일 포인트 이동

```
1. Absolute Coordinates 선택
2. 좌표 입력:
   X: 400
   Y: 0
   Z: 500
3. Add Point 클릭
4. Execute Motion 클릭
→ 로봇이 (400, 0, 500) 위치로 이동
→ RViz에서 확인 가능
```

### 예시 2: 다중 포인트 순차 이동

```
1. Absolute Coordinates 선택
2. 여러 좌표 추가:
   Point 1: X:400, Y:0, Z:500 → Add Point
   Point 2: X:300, Y:100, Z:450 → Add Point
   Point 3: X:500, Y:-100, Z:600 → Add Point
   
3. Set Velocity 체크, 0.2 입력
4. Set Acceleration 체크, 0.8 입력
5. Execute Motion 클릭
→ 로봇이 1→2→3 순서로 이동
→ 각 포인트 도달 시 로그에 표시
```

### 예시 3: 상대 좌표 이동

```
1. Relative Coordinates 선택
2. 좌표 입력:
   X: 50
   Y: 0
   Z: 100
3. Add Point 클릭
4. Execute Motion 클릭
→ 현재 위치에서 (+50, 0, +100) 이동
```

### 예시 4: 사각형 궤적

```
1. Absolute Coordinates
2. 4개 포인트 추가:
   Point 1: (400, 100, 500)
   Point 2: (400, -100, 500)
   Point 3: (300, -100, 500)
   Point 4: (300, 100, 500)
3. Execute Motion
→ 사각형 궤적으로 이동
```

---

## 📁 프로젝트 구조

```
ros2_ws/
├── src/
│   ├── doosan-robot2/              # Doosan 공식 패키지
│   │   ├── dsr_bringup2/           # 런처 및 설정
│   │   ├── dsr_controller2/        # 로봇 컨트롤러
│   │   ├── dsr_description2/       # URDF 모델
│   │   ├── dsr_hardware2/          # 하드웨어 인터페이스
│   │   ├── dsr_moveit2/            # MoveIt 설정
│   │   ├── dsr_moveit_config_e0509/# E0509 MoveIt 설정
│   │   └── ...
│   │
│   └── doosan_controller/          # 본 프로젝트 ⭐
│       ├── package.xml             # 패키지 메타데이터
│       ├── setup.py                # Python 설정
│       ├── setup.cfg               # 설치 설정
│       ├── resource/               # 리소스 파일
│       │   └── doosan_controller
│       ├── launch/                 # 런치 파일 (선택)
│       ├── config/                 # 설정 파일 (선택)
│       └── doosan_controller/      # Python 모듈
│           ├── __init__.py
│           ├── robot_controller.py # ROS2 노드 및 메인
│           ├── gui_window.py       # PyQt5 GUI
│           └── motion_executor.py  # 모션 실행 스레드
│
├── build/                          # 빌드 파일 (자동 생성)
├── install/                        # 설치 파일 (자동 생성)
└── log/                            # 로그 파일 (자동 생성)
```

---

## 주요 모듈 설명

### 1. robot_controller.py (메인 노드)

```python
class RobotControllerNode(Node):
    """ROS2 메인 노드"""
    
    # 주요 기능:
    - /joint_states 구독 (로봇 상태 수신)
    - /dsr_moveit_controller/joint_trajectory 발행 (명령 전송)
    - 역기구학 계산 (send_cartesian_move)
    - GUI와 로봇 중재
```

**주요 토픽**:
- 구독: `/joint_states` (sensor_msgs/JointState)
- 발행: `/dsr_moveit_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)

### 2. gui_window.py (사용자 인터페이스)

```python
class RobotControlGUI(QMainWindow):
    """PyQt5 GUI"""
    
    # 구성:
    - 제어 패널 (create_control_panel)
    - 상태 패널 (create_status_panel)
    - 100ms 타이머로 실시간 업데이트
```

### 3. motion_executor.py (모션 실행)

```python
class MotionExecutor:
    """별도 스레드에서 모션 실행"""
    
    # 기능:
    - 다중 목표점 순차 처리
    - 이동 시간 추정
    - 긴급 정지 지원
```

---

## 기술 상세

### 역기구학 (Inverse Kinematics)

E0509 로봇의 2D 평면 역기구학:

```python
# 링크 길이
L1 = 0.409 m  # 베이스 높이
L2 = 0.367 m  # 상완
L3 = 0.124 m  # 전완

# Joint 1: 베이스 회전
j1 = atan2(y, x)

# Joint 2, 3: 평면 2R 로봇
reach = sqrt(r² + z_adj²)
cos_j3 = (reach² - L2² - L3²) / (2*L2*L3)
j3 = acos(cos_j3)

alpha = atan2(z_adj, r)
beta = atan2(L3*sin(j3), L2 + L3*cos(j3))
j2 = alpha - beta

# Joint 4, 5, 6: 손목 (고정)
j4 = j5 = j6 = 0.0
```

### ROS2 통신 구조

```
[GUI] ─(좌표)→ [Controller Node] ─(IK 계산)→ [JointTrajectory]
                        ↓
                [/dsr_moveit_controller/joint_trajectory]
                        ↓
                [Doosan Controller] → [Simulator/Robot]
                        ↓
                [/joint_states] → [Controller Node] → [GUI 업데이트]
```

### 멀티스레딩 구조

- **메인 스레드**: PyQt5 이벤트 루프 (GUI 렌더링)
- **ROS2 스레드**: `executor.spin()` (토픽 수신/발행)
- **모션 스레드**: 순차적 목표점 이동 (MotionExecutor)

---

## 문제 해결

### 1. GUI가 열리지 않음

**증상**: "cannot connect to X server" 에러

**해결**:
```bash
# 환경 변수 확인
echo $DISPLAY

# 설정되지 않았다면
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1

# .bashrc에 영구 추가
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```

### 2. 로봇이 연결되지 않음

**증상**: Connection Status가 계속 "Disconnected"

**해결**:
```bash
# 새 터미널에서 토픽 확인
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep joint_states

# joint_states가 없다면 시뮬레이터 재시작
```

### 3. 빌드 에러

**증상**: `colcon build` 실패

**해결**:
```bash
# 클린 빌드
cd ~/ros2_ws
rm -rf build install log

# Poco 라이브러리 재설치
sudo apt install libpoco-dev libprotobuf-dev protobuf-compiler -y

# 재빌드
colcon build
source install/setup.bash
```

### 4. "Package 'doosan_controller' not found"

**증상**: `ros2 run` 실행 시 패키지를 찾을 수 없음

**해결**:
```bash
# 환경 설정 재로드
cd ~/ros2_ws
source install/setup.bash

# 패키지 확인
ros2 pkg list | grep doosan_controller
```

---

## 성능 및 제약사항

### 시스템 요구사항

- **메모리**: 최소 4GB, 권장 8GB
- **디스크**: 최소 10GB 여유 공간
- **CPU**: 멀티코어 권장 (빌드 시간 단축)

### 작동 범위

- **도달 거리**: 약 0.9m (E0509 최대 도달 거리)
- **권장 좌표 범위**:
  - X: 200 ~ 600 mm
  - Y: -300 ~ 300 mm
  - Z: 300 ~ 700 mm

**범위 밖 좌표 입력 시**: 자동으로 제한되며 로그에 경고 표시

### 성능 특성

- **응답 시간**: 명령 후 0.1초 이내 시작
- **위치 정확도**: ±5mm (시뮬레이션 기준)
- **GUI 업데이트**: 100ms (10Hz)

---

## 개발 노트

### 구현 과정

1. **WSL 환경 구축** → GUI 렌더링 설정
2. **ROS2 패키지 구조 설계** → setup.py, package.xml
3. **PyQt5 GUI 구현** → 코드로 직접 작성 (Qt Designer 미사용)
4. **역기구학 구현** → 2D 평면 IK
5. **토픽 통신 연결** → Doosan 컨트롤러 연동
6. **스레드 분리** → GUI/ROS2/모션 독립 실행

### 해결한 주요 문제

1. **WSL GUI 렌더링**: `LIBGL_ALWAYS_SOFTWARE=1` 설정
2. **토픽 찾기**: `/dsr_moveit_controller/joint_trajectory` 발견
3. **메시지 형식**: `velocities` 필드 필수임을 확인
4. **좌표계 변환**: mm ↔ m 변환 주의

## 라이선스

Apache-2.0

---

## 참고 자료

### 공식 문서

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [Doosan Robotics GitHub](https://github.com/DoosanRobotics/doosan-robot2)
- [PyQt5 Documentation](https://www.riverbankcomputing.com/static/Docs/PyQt5/)

### 유용한 명령어

```bash
# 토픽 모니터링
ros2 topic list
ros2 topic echo /joint_states
ros2 topic hz /joint_states

# 노드 확인
ros2 node list
ros2 node info /doosan_controller_node

# 빌드 및 실행
colcon build --packages-select doosan_controller
source install/setup.bash
ros2 run doosan_controller robot_control_node
```

