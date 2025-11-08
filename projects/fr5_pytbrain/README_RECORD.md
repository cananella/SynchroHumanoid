# FR5-Quest LeRobot Dataset Recording

Quest 컨트롤러로 FR5 로봇을 텔레오퍼레이션하며 LeRobot 데이터셋을 수집합니다.

## 📋 준비사항

### 1. 하드웨어
- ✅ FR5 로봇 (IP: 192.168.58.2)
- ✅ Quest 헤드셋 (Quest 앱 실행)
- ✅ 카메라 2개:
  - 손목 카메라: `/dev/video0`
  - 정면 카메라: `/dev/video2`

### 2. 소프트웨어
```bash
# Conda 환경 활성화
conda activate lerobot

# 필요한 패키지 확인
python -c "import cv2; from lerobot.datasets.lerobot_dataset import LeRobotDataset; print('OK')"
```

### 3. HuggingFace 로그인 (최초 1회만)

데이터셋을 HuggingFace Hub에 업로드하려면 먼저 로그인이 필요합니다.

#### 방법 1: CLI로 로그인 (권장)
```bash
huggingface-cli login
```
- 프롬프트가 나오면 HuggingFace token 입력
- Token은 https://huggingface.co/settings/tokens 에서 생성 가능
- **Write** 권한이 있는 token 필요

#### 방법 2: 환경 변수로 설정
```bash
export HF_TOKEN=your_token_here
```

#### Token 생성 방법:
1. https://huggingface.co/settings/tokens 접속
2. "New token" 클릭
3. Token 이름 입력 (예: "lerobot-dataset-upload")
4. **Type: Write** 선택 (중요!)
5. "Generate a token" 클릭
6. 생성된 token 복사

#### 로그인 확인:
```bash
huggingface-cli whoami
```
- 사용자명이 표시되면 로그인 성공

---

## ⚙️ 설정

### `record_config.yaml` 파일 수정

```yaml
dataset:
  name: "fr5-pick-cube"           # 데이터셋 이름 (변경 필수!)
  hf_username: "your_username"    # HuggingFace 사용자명 (변경 필수!)
  local_root: "/home/stream/vla/datasets"
  fps: 30
  use_videos: true

recording:
  num_episodes: 10                # 녹화할 에피소드 개수
  episode_time_s: 60              # 한 에피소드 시간 (초)
  reset_time_s: 30                # 에피소드 간 휴식 시간 (초)
  task_description: "Pick the red cube and place it in the box"  # Task 설명 (변경 필수!)

robot:
  ip: "192.168.57.2"
  robot_type: "fr5"

cameras:
  wrist:
    device_id: 0  # /dev/video0
    width: 640
    height: 480
    fps: 30
  front:
    device_id: 1  # /dev/video1
    width: 640
    height: 480
    fps: 30

quest:
  port: 5454
  hz: 72
```

**주요 설정 항목:**
- `dataset.name`: 데이터셋 이름 (디렉토리명 및 HF repo 이름)
- `dataset.hf_username`: HuggingFace 사용자명
- `recording.task_description`: Task 설명 (모든 에피소드에 동일하게 적용)
- `recording.num_episodes`: 녹화할 에피소드 개수

---

## 🎬 녹화 실행

### 1. 로봇 및 Quest 준비
```bash
# 1. FR5 로봇 전원 ON 및 자동 모드 설정
# 2. Quest 앱 실행 (PC IP로 연결)
# 3. 카메라 연결 확인
ls /dev/video*
```

### 2. 스크립트 실행

**방법 A: Config 파일 사용 (권장)**
```bash
conda activate lerobot
cd /home/stream/vla/fr5_sst
python record_fr5_quest_dataset.py --config record_config.yaml
```

**방법 B: CLI로 설정 오버라이드**
```bash
python record_fr5_quest_dataset.py \
    --config record_config.yaml \
    --dataset-name fr5-pick-cube \
    --num-episodes 5 \
    --task "Pick the red cube"
```

### 3. 녹화 프로세스

1. **Quest 연결 대기**
   - Quest 앱에서 PC IP로 연결
   - 자동으로 연결 감지

2. **Quest 동기화**
   - 5초 동안 컨트롤러를 가만히 두세요
   - 기준점 설정

3. **에피소드 녹화** (반복)
   - `Press Enter to start recording...` → Enter 키 입력
   - 60초 동안 Quest로 로봇 제어
   - 데이터 자동 저장
   - 30초 휴식 (환경 리셋)
   - 다음 에피소드로 자동 진행

4. **완료**
   - 모든 에피소드 녹화 완료
   - 로컬에 자동 저장: `/home/stream/vla/datasets/{dataset_name}/`

---

## 📤 HuggingFace Hub 업로드

### 1. 데이터셋 업로드

> **참고:** 준비사항 섹션에서 HuggingFace 로그인을 먼저 완료해야 합니다.
```bash
# 예시: fr5-pick-cube 데이터셋 업로드
huggingface-cli upload your_username/fr5-pick-cube \
    /home/stream/vla/datasets/fr5-pick-cube \
    --repo-type=dataset
```

**파라미터:**
- `your_username/fr5-pick-cube`: HuggingFace repo ID (config의 `{hf_username}/{dataset.name}`)
- `/home/stream/vla/datasets/fr5-pick-cube`: 로컬 데이터셋 경로
- `--repo-type=dataset`: 데이터셋 타입 지정

### 2. 업로드 확인
업로드 완료 후 다음 URL에서 확인:
```
https://huggingface.co/datasets/{your_username}/{dataset_name}
```

---

## 📊 데이터셋 구조

```
/home/stream/vla/datasets/fr5-pick-cube/
├── meta/
│   ├── info.json           # 데이터셋 메타정보
│   ├── tasks.json          # Task 설명
│   ├── stats.json          # 통계 (학습용)
│   └── episodes/           # 에피소드 메타데이터
│       └── chunk-000/
│           └── file-000.parquet
├── data/
│   └── chunk-000/
│       └── file-000.parquet  # 조인트, action 데이터
└── videos/
    └── chunk-000/
        ├── observation.images.wrist/
        │   ├── episode_000000.mp4
        │   ├── episode_000001.mp4
        │   └── ...
        └── observation.images.front/
            ├── episode_000000.mp4
            ├── episode_000001.mp4
            └── ...
```

**데이터 features:**
- `observation.state`: 현재 조인트 값 (6,)
- `observation.images.wrist`: 손목 카메라 영상
- `observation.images.front`: 정면 카메라 영상
- `action`: 목표 조인트 값 (6,) - 절대값
- `task`: Task 설명 (문자열)

---

## 🔧 트러블슈팅

### Quest 연결 안 됨
```bash
# Quest 앱에서 PC IP 확인
ifconfig

# 포트 5454가 열려있는지 확인
netstat -tuln | grep 5454
```

### 카메라 인식 안 됨
```bash
# 카메라 디바이스 확인
ls -l /dev/video*

# 카메라 테스트
python -c "import cv2; cap = cv2.VideoCapture(0); print(cap.read()[0])"
```

### 로봇 연결 실패
```bash
# 로봇 IP 핑 테스트
ping 192.168.57.2

# 네트워크 설정 확인 (같은 서브넷인지)
```

### 녹화 중 끊김
- FPS를 30에서 20으로 낮춰보기
- `use_videos: false`로 설정 (이미지로 저장)
- 카메라 해상도 낮추기 (640x480 → 320x240)

---

## 💡 팁

### 효율적인 녹화
1. **에피소드 시간 조절**
   - 간단한 task: 30초
   - 복잡한 task: 60-120초

2. **휴식 시간 활용**
   - 환경 리셋 (물체 배치)
   - 로봇 홈 포지션 복귀
   - 다음 에피소드 준비

3. **데이터 품질**
   - 각 에피소드마다 성공적으로 task 완료
   - 다양한 초기 상태에서 시작
   - 일관된 동작 패턴 유지

### 다양한 Task 수집
```bash
# Task 1: Pick red cube
python record_fr5_quest_dataset.py \
    --dataset-name fr5-pick-red-cube \
    --task "Pick the red cube"

# Task 2: Place in box
python record_fr5_quest_dataset.py \
    --dataset-name fr5-place-in-box \
    --task "Place the cube in the box"

# Task 3: Stack cubes
python record_fr5_quest_dataset.py \
    --dataset-name fr5-stack-cubes \
    --task "Stack the blue cube on top of red cube"
```

---

## 📚 다음 단계

### 1. 데이터셋 확인
```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(repo_id="your_username/fr5-pick-cube")
print(f"Episodes: {dataset.num_episodes}")
print(f"Frames: {dataset.num_frames}")
print(f"Features: {dataset.features}")
```

### 2. 학습
```bash
# LeRobot으로 policy 학습 (예: Diffusion Policy)
lerobot-train \
    policy=diffusion \
    dataset_repo_id=your_username/fr5-pick-cube \
    ...
```

### 3. Inference
학습된 모델로 로봇 제어 (별도 스크립트 필요)

---

## 📞 문의

문제가 발생하면 다음을 확인하세요:
1. 모든 하드웨어가 연결되어 있는지
2. Conda 환경이 활성화되어 있는지
3. Config 파일 설정이 올바른지
4. 로봇이 자동 모드인지
