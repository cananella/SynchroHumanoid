#!/usr/bin/env python3
"""
LeRobot 데이터셋의 조인트 값 확인 스크립트

Usage:
    python check_dataset_joints.py /home/stream/vla/datasets/fr5_test_dataset
"""

import sys
import pandas as pd
import numpy as np
from pathlib import Path


def check_dataset_joints(dataset_path):
    """데이터셋의 조인트 값 분석"""
    dataset_path = Path(dataset_path)

    # 데이터 파일 찾기
    data_file = dataset_path / "data/chunk-000/file-000.parquet"

    if not data_file.exists():
        print(f"❌ Data file not found: {data_file}")
        return

    # 데이터 읽기
    df = pd.read_parquet(data_file)

    print("=" * 70)
    print(f"📊 Dataset: {dataset_path.name}")
    print("=" * 70)
    print(f"\nTotal frames: {len(df)}")
    print(f"Columns: {list(df.columns)}")

    # 첫 90 프레임
    print(f"\n{'='*70}")
    print("First 90 frames:")
    print("=" * 70)

    for i in range(min(90, len(df))):
        obs_state = df.iloc[i]['observation.state']
        action = df.iloc[i]['action']
        episode_idx = df.iloc[i]['episode_index']
        frame_idx = df.iloc[i]['frame_index']

        print(f"\nFrame {i} (Episode {episode_idx}, Frame {frame_idx}):")
        print(f"  observation.state: [{obs_state[0]:7.2f}, {obs_state[1]:7.2f}, {obs_state[2]:7.2f}, {obs_state[3]:7.2f}, {obs_state[4]:7.2f}, {obs_state[5]:7.2f}]")
        print(f"  action:            [{action[0]:7.2f}, {action[1]:7.2f}, {action[2]:7.2f}, {action[3]:7.2f}, {action[4]:7.2f}, {action[5]:7.2f}]")

    # 마지막 90 프레임
    print(f"\n{'='*70}")
    print("Last 90 frames:")
    print("=" * 70)

    for i in range(max(0, len(df)-90), len(df)):
        obs_state = df.iloc[i]['observation.state']
        action = df.iloc[i]['action']
        episode_idx = df.iloc[i]['episode_index']
        frame_idx = df.iloc[i]['frame_index']

        print(f"\nFrame {i} (Episode {episode_idx}, Frame {frame_idx}):")
        print(f"  observation.state: [{obs_state[0]:7.2f}, {obs_state[1]:7.2f}, {obs_state[2]:7.2f}, {obs_state[3]:7.2f}, {obs_state[4]:7.2f}, {obs_state[5]:7.2f}]")
        print(f"  action:            [{action[0]:7.2f}, {action[1]:7.2f}, {action[2]:7.2f}, {action[3]:7.2f}, {action[4]:7.2f}, {action[5]:7.2f}]")

    # 통계
    print(f"\n{'='*70}")
    print("Statistics:")
    print("=" * 70)

    obs_states = np.vstack(df['observation.state'].values)
    actions = np.vstack(df['action'].values)

    print(f"\nobservation.state shape: {obs_states.shape}")
    print(f"action shape: {actions.shape}")

    print(f"\nobservation.state range:")
    for j in range(6):
        print(f"  Joint {j}: [{obs_states[:, j].min():7.2f}, {obs_states[:, j].max():7.2f}]  (mean: {obs_states[:, j].mean():7.2f}, std: {obs_states[:, j].std():6.2f})")

    print(f"\naction range:")
    for j in range(6):
        print(f"  Joint {j}: [{actions[:, j].min():7.2f}, {actions[:, j].max():7.2f}]  (mean: {actions[:, j].mean():7.2f}, std: {actions[:, j].std():6.2f})")

    # observation.state와 action 비교
    print(f"\n{'='*70}")
    print("Comparing observation.state vs action:")
    print("=" * 70)
    diff = np.abs(obs_states - actions)
    print(f"Max difference: {diff.max():.6f}")
    print(f"Mean difference: {diff.mean():.6f}")
    print(f"Are they identical? {np.allclose(obs_states, actions)}")

    if not np.allclose(obs_states, actions):
        print(f"\n⚠️  observation.state and action are NOT identical!")
        print(f"Frames with differences > 0.01:")
        for i in range(len(df)):
            frame_diff = np.abs(obs_states[i] - actions[i])
            if frame_diff.max() > 0.01:
                print(f"  Frame {i}: max_diff = {frame_diff.max():.4f}")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python check_dataset_joints.py <dataset_path>")
        print("\nExample:")
        print("  python check_dataset_joints.py /home/stream/vla/datasets/fr5_test_dataset")
        sys.exit(1)

    dataset_path = sys.argv[1]
    check_dataset_joints(dataset_path)
