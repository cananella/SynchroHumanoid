#!/usr/bin/env python3
"""
데이터셋의 action이 다음 프레임 observation인지 확인

Usage:
    python analyze_action_type.py /path/to/dataset
"""

import sys
import pandas as pd
import numpy as np
from pathlib import Path


def analyze_action_type(dataset_path):
    """action이 next observation인지 분석"""
    dataset_path = Path(dataset_path)
    data_file = dataset_path / "data/chunk-000/file-000.parquet"

    if not data_file.exists():
        print(f"❌ Data file not found: {data_file}")
        return

    df = pd.read_parquet(data_file)

    print("=" * 70)
    print(f"🔍 Analyzing: {dataset_path.name}")
    print("=" * 70)
    print(f"Total frames: {len(df)}\n")

    # 연속된 프레임 분석
    print("Checking if action[i] == observation.state[i+1]:")
    print("=" * 70)

    match_count = 0
    total_checked = 0

    for i in range(min(20, len(df)-1)):
        obs_state_current = df.iloc[i]['observation.state']
        action_current = df.iloc[i]['action']
        obs_state_next = df.iloc[i+1]['observation.state']

        # Episode 경계 확인
        episode_current = df.iloc[i]['episode_index']
        episode_next = df.iloc[i+1]['episode_index']

        # action[i]와 observation.state[i+1] 비교
        diff = np.abs(action_current - obs_state_next)
        is_match = np.allclose(action_current, obs_state_next, atol=0.1)

        print(f"\nFrame {i} → {i+1} (Episode {episode_current} → {episode_next}):")
        print(f"  action[{i}]:            [{action_current[0]:7.2f}, {action_current[1]:7.2f}, {action_current[2]:7.2f}, {action_current[3]:7.2f}, {action_current[4]:7.2f}, {action_current[5]:7.2f}]")
        print(f"  observation.state[{i+1}]: [{obs_state_next[0]:7.2f}, {obs_state_next[1]:7.2f}, {obs_state_next[2]:7.2f}, {obs_state_next[3]:7.2f}, {obs_state_next[4]:7.2f}, {obs_state_next[5]:7.2f}]")
        print(f"  Diff:                    [{diff[0]:7.2f}, {diff[1]:7.2f}, {diff[2]:7.2f}, {diff[3]:7.2f}, {diff[4]:7.2f}, {diff[5]:7.2f}]")
        print(f"  Match? {is_match}")

        # Episode 경계가 아닌 경우만 카운트
        if episode_current == episode_next:
            if is_match:
                match_count += 1
            total_checked += 1

    print(f"\n{'='*70}")
    print(f"Summary (excluding episode boundaries): {match_count}/{total_checked} frames match")
    if total_checked > 0:
        print(f"Match rate: {match_count/total_checked*100:.1f}%")
        print("=" * 70)

        if match_count / total_checked > 0.9:
            print("\n✅ Conclusion: action[i] ≈ observation.state[i+1]")
            print("   → Uses NEXT frame's observation as action")
        else:
            print("\n❌ Conclusion: action is NOT simply next frame's observation")
            print("   → Likely using computed target from controller")
    else:
        print("Not enough data to determine")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python analyze_action_type.py <dataset_path>")
        print("\nExample:")
        print("  python analyze_action_type.py /home/stream/.cache/huggingface/lerobot/xhaka3456/so100_pick_red_1017")
        sys.exit(1)

    dataset_path = sys.argv[1]
    analyze_action_type(dataset_path)
