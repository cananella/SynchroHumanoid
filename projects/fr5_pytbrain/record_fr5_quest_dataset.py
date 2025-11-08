#!/usr/bin/env python3
"""
FR5-Quest LeRobot Dataset Recording

기존 Quest teleoperation 코드를 활용하여 LeRobot 데이터셋 포맷으로 저장합니다.

Usage:
    python record_fr5_quest_dataset.py --config record_config.yaml

    또는 CLI로 직접 설정:
    python record_fr5_quest_dataset.py \
        --dataset-name fr5-pick-cube \
        --hf-username your_username \
        --num-episodes 10 \
        --task "Pick the red cube"
"""

import sys
import time
import threading
import argparse
import yaml
from pathlib import Path

import numpy as np
import cv2

from sdk_paths import setup_paths
setup_paths()

from third_party.fairino_python_sdk.linux.fairino import Robot
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from new_quest_fr5_teleop_full_6dof import QuestFR5TeleopAsync


class FR5QuestRecorder:
    def __init__(self, config):
        """
        FR5-Quest 데이터 수집기

        Args:
            config: 설정 딕셔너리 (YAML에서 로드)
        """
        self.config = config

        # 데이터셋 경로 설정
        dataset_name = config['dataset']['name']
        hf_username = config['dataset']['hf_username']
        local_root = Path(config['dataset']['local_root'])

        self.repo_id = f"{hf_username}/{dataset_name}"
        self.dataset_root = local_root / dataset_name

        print("=" * 70)
        print("🎮 FR5-Quest LeRobot Dataset Recording")
        print("=" * 70)
        print(f"📦 Dataset: {self.repo_id}")
        print(f"💾 Local path: {self.dataset_root}")
        print(f"📊 FPS: {config['dataset']['fps']}")
        print(f"🎬 Episodes: {config['recording']['num_episodes']}")
        print(f"⏱️  Episode duration: {config['recording']['episode_time_s']}s")
        print("=" * 70)

        # 데이터셋 생성 또는 로드
        resume = config['dataset'].get('resume', False)

        if resume and self.dataset_root.exists():
            print(f"\n📂 Resuming existing dataset...")
            self.dataset = LeRobotDataset(
                repo_id=self.repo_id,
                root=self.dataset_root,
            )
            print(f"✅ Loaded existing dataset with {self.dataset.num_episodes} episodes")
        else:
            # 동적으로 카메라 features 생성
            # Joint names for FR5 robot
            joint_names = [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6"
            ]

            features = {
                "observation.state": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": joint_names
                },
                "action": {
                    "dtype": "float32",
                    "shape": (6,),
                    "names": joint_names
                },
            }

            # 모든 카메라에 대해 features 추가
            for cam_name, cam_cfg in config['cameras'].items():
                features[f"observation.images.{cam_name}"] = {
                    "dtype": "video",
                    "shape": (cam_cfg['height'], cam_cfg['width'], 3),
                    "names": ["height", "width", "channels"]
                }

            print("\n📁 Creating new LeRobot dataset...")
            self.dataset = LeRobotDataset.create(
                repo_id=self.repo_id,
                root=self.dataset_root,
                fps=config['dataset']['fps'],
                features=features,
                robot_type=config['robot']['robot_type'],
                use_videos=config['dataset']['use_videos'],
            )
            print("✅ Dataset created!")

        # Quest 제어 시스템 초기화 (기존 코드 활용)
        print("\n🤖 Initializing FR5-Quest teleoperation...")
        self.teleop = QuestFR5TeleopAsync()
        self.robot = self.teleop.robot

        # 카메라 초기화 (동적)
        print("\n📷 Initializing cameras...")
        self.cameras = {}

        for cam_name, cam_cfg in config['cameras'].items():
            cap = cv2.VideoCapture(cam_cfg['device_id'])
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg['height'])
            cap.set(cv2.CAP_PROP_FPS, cam_cfg['fps'])

            # 카메라 확인
            ret, _ = cap.read()
            if not ret:
                raise RuntimeError(f"❌ Failed to read from camera '{cam_name}' (/dev/video{cam_cfg['device_id']})")

            self.cameras[cam_name] = cap
            print(f"✅ {cam_name} camera: /dev/video{cam_cfg['device_id']}")

        # Quest 스레드 (데이터 수신용)
        self.quest_thread = None
        self.robot_thread = None

    def start_teleoperation(self):
        """Quest teleoperation 스레드 시작 (기존 코드 활용)"""
        print("\n🥽 Starting Quest receiver...")
        self.teleop.running = True

        # Quest 수신 스레드 시작
        self.quest_thread = threading.Thread(
            target=self.teleop.quest_receiver_thread,
            daemon=True
        )
        self.quest_thread.start()

        # 로봇 제어 스레드 시작 (중요!)
        self.robot_thread = threading.Thread(
            target=self.teleop.robot_control_thread,
            daemon=True
        )
        self.robot_thread.start()

        # Quest 연결 대기 (무한 대기, Ctrl+C로 중단 가능)
        print("   Waiting for Quest connection... (Press Ctrl+C to cancel)")
        while not self.teleop.quest_connected:
            time.sleep(0.1)

        print("✅ Quest connected!")

    def move_to_home(self):
        """홈 포지션으로 이동"""
        HOME_JOINTS = [-136.0, -128.0, -90.0, 40.0, 90.00, 0.0]

        print("\n🏠 Moving to home position...")

        error = self.robot.MoveJ(
            joint_pos=HOME_JOINTS,
            desc_pos=[0, 0, 0, 0, 0, 0],
            tool=0,
            user=0,
            vel=30.0,
            acc=30.0,
            ovl=100.0,
            blendT=-1.0,
            offset_flag=0,
            offset_pos=[0, 0, 0, 0, 0, 0]
        )

        if error == 0:
            print("✅ Home position reached!")
            time.sleep(0.5)
        else:
            print(f"⚠️  Failed to move to home (error: {error})")

    def calibrate_quest(self):
        """Quest 동기화 (기존 코드 활용)"""
        print("\n📍 Ready to calibrate Quest")
        print("   Press Enter to move robot to home position first...")
        input()

        # 홈 위치로 이동
        self.move_to_home()

        print("\n   Now press Enter to start Quest calibration (hold controller still for 2.5s)...")
        input()

        print("🔄 Calibrating... Keep controller still!")
        if not self.teleop.calibrate(calibration_time=2.5):
            raise RuntimeError("❌ Quest calibration failed!")
        print("✅ Quest calibrated!")

    def record_episode(self, episode_index, task_description):
        """
        한 에피소드 녹화

        Args:
            episode_index: 에피소드 번호
            task_description: Task 설명 (모든 프레임에 동일하게 적용)
        """
        config = self.config
        episode_time_s = config['recording']['episode_time_s']
        fps = config['dataset']['fps']
        dt = 1.0 / fps

        print(f"\n{'='*70}")
        print(f"🎬 Episode {episode_index + 1}/{config['recording']['num_episodes']}")
        print(f"📝 Task: {task_description}")
        print(f"⏱️  Duration: {episode_time_s}s")
        print(f"{'='*70}")

        input("Press Enter to start recording...")

        # 에피소드 버퍼 생성
        self.dataset.episode_buffer = self.dataset.create_episode_buffer()

        # 임시 버퍼: observation.state만 저장 (나중에 action 계산용)
        episode_observations = []

        # 녹화 루프
        start_time = time.time()
        frame_count = 0

        print("\n🔴 Recording...")

        while time.time() - start_time < episode_time_s:
            loop_start = time.time()

            # 1. 현재 로봇 조인트 읽기 (observation)
            error, current_joints = self.robot.GetActualJointPosDegree()
            if error != 0:
                print(f"⚠️  Warning: Failed to read joints (error {error})")
                continue

            # 2. 카메라 프레임 읽기 (동적)
            camera_frames = {}
            all_frames_ok = True

            for cam_name, cap in self.cameras.items():
                ret, frame = cap.read()
                if not ret:
                    print(f"⚠️  Warning: Failed to read from camera '{cam_name}'")
                    all_frames_ok = False
                    break

                # 이미지 해상도 확인 및 리사이즈
                cam_cfg = config['cameras'][cam_name]
                if frame.shape[:2] != (cam_cfg['height'], cam_cfg['width']):
                    frame = cv2.resize(frame, (cam_cfg['width'], cam_cfg['height']))

                camera_frames[cam_name] = frame

            if not all_frames_ok:
                continue

            # observation.state 임시 저장
            obs_state = np.array(current_joints, dtype=np.float32)
            episode_observations.append(obs_state)

            # 임시 action (나중에 채워짐)
            # action[t] = observation.state[t+1]이 되도록 처리
            action_joints = np.zeros(6, dtype=np.float32)

            # 프레임 데이터 구성
            frame_data = {
                "observation.state": obs_state,
                "action": action_joints,
                "task": task_description,
            }

            # 모든 카메라 이미지 추가
            for cam_name, frame in camera_frames.items():
                frame_data[f"observation.images.{cam_name}"] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # 프레임 저장
            self.dataset.add_frame(frame_data)

            frame_count += 1

            # 카메라 화면 표시 (display_data 옵션)
            if config['dataset'].get('display_data', False):
                # 모든 카메라 이미지를 나란히 표시
                display_frames = list(camera_frames.values())
                combined = np.hstack(display_frames)

                # 상태 정보 오버레이
                cv2.putText(combined, f"Frame: {frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined, f"Episode: {episode_index + 1}/{config['recording']['num_episodes']}",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 카메라 이름 표시
                cam_names = " | ".join(camera_frames.keys())
                cv2.imshow(f'FR5 Recording ({cam_names})', combined)
                cv2.waitKey(1)

            # 진행 상황 표시 (5초마다)
            if frame_count % (fps * 5) == 0:
                elapsed = time.time() - start_time
                remaining = episode_time_s - elapsed
                print(f"   {frame_count} frames | {remaining:.1f}s remaining")

            # FPS 유지
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)

        # action 채우기: action[t] = observation.state[t+1]
        print(f"\n🔧 Processing actions (next-frame strategy)...")
        for i in range(len(episode_observations)):
            if i < len(episode_observations) - 1:
                # action[t] = observation.state[t+1]
                next_obs = episode_observations[i + 1]
            else:
                # 마지막 프레임: action = 현재 observation (변화 없음)
                next_obs = episode_observations[i]

            # episode_buffer의 action 업데이트
            self.dataset.episode_buffer["action"][i] = next_obs

        # 에피소드 저장
        print(f"💾 Saving episode ({frame_count} frames)...")
        self.dataset.save_episode()
        print("✅ Episode saved!")

        return frame_count

    def run(self):
        """메인 녹화 루프"""
        config = self.config

        # Task 설정 (config 파일에서)
        task_description = config['recording']['task_description']

        print(f"\n✅ Task: '{task_description}'")
        print(f"   All {config['recording']['num_episodes']} episodes will use this task.\n")

        try:
            # Quest 연결 및 동기화
            self.start_teleoperation()
            self.calibrate_quest()

            # 에피소드 녹화
            total_frames = 0
            for ep_idx in range(config['recording']['num_episodes']):
                frame_count = self.record_episode(ep_idx, task_description)
                total_frames += frame_count

                # 마지막 에피소드가 아니면 휴식
                if ep_idx < config['recording']['num_episodes'] - 1:
                    reset_time = config['recording']['reset_time_s']
                    print(f"\n⏸️  Rest time: {reset_time}s")
                    print("   (Reset environment for next episode)")
                    time.sleep(reset_time)

            # 완료
            print("\n" + "=" * 70)
            print("✅ Recording completed!")
            print("=" * 70)
            print(f"📊 Total episodes: {config['recording']['num_episodes']}")
            print(f"📊 Total frames: {total_frames}")
            print(f"💾 Saved to: {self.dataset_root}")
            print(f"📦 Repository: {self.repo_id}")
            print("\n💡 To upload to HuggingFace Hub:")
            print(f"   huggingface-cli upload {self.repo_id} {self.dataset_root} --repo-type=dataset")
            print("=" * 70)

        except KeyboardInterrupt:
            print("\n\n⚠️  Recording interrupted by user")
        except Exception as e:
            print(f"\n\n❌ Error: {e}")
            raise
        finally:
            # 정리
            print("\n🧹 Cleaning up...")
            self.teleop.running = False

            # 모든 카메라 해제 (동적)
            for cam_name, cap in self.cameras.items():
                cap.release()

            # OpenCV 윈도우 닫기 (display_data 사용 시)
            if config['dataset'].get('display_data', False):
                cv2.destroyAllWindows()

            print("✅ Done!")


def load_config(config_path):
    """YAML 설정 파일 로드"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="FR5-Quest LeRobot Dataset Recording")
    parser.add_argument(
        '--config',
        type=str,
        default='record_config.yaml',
        help='Path to config YAML file'
    )

    # CLI 오버라이드 옵션
    parser.add_argument('--dataset-name', type=str, help='Dataset name (overrides config)')
    parser.add_argument('--hf-username', type=str, help='HuggingFace username (overrides config)')
    parser.add_argument('--num-episodes', type=int, help='Number of episodes (overrides config)')
    parser.add_argument('--task', type=str, help='Task description (overrides config)')

    args = parser.parse_args()

    # 설정 로드
    config = load_config(args.config)

    # CLI 오버라이드 적용
    if args.dataset_name:
        config['dataset']['name'] = args.dataset_name
    if args.hf_username:
        config['dataset']['hf_username'] = args.hf_username
    if args.num_episodes:
        config['recording']['num_episodes'] = args.num_episodes
    if args.task:
        config['recording']['task_description'] = args.task

    # 녹화 시작
    recorder = FR5QuestRecorder(config)
    recorder.run()


if __name__ == "__main__":
    main()
