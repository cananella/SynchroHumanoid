import sys
from pathlib import Path


def setup_paths():
    current = Path(__file__).resolve()
    
    for parent in current.parents:
        if parent.name == 'SynchroHumanoid':
            vla_root = parent
            break
    else:
        raise RuntimeError("Could not find 'SynchroHumanoid' directory")
    
    print(f"VLA root directory: {vla_root}")

    fairino_path = str(vla_root / 'third_party/fairino/fairino_python_sdk/linux')
    lerobot_path = str(vla_root / 'third_party/lerobot/src')

    if fairino_path not in sys.path:
        sys.path.append(fairino_path)

    if lerobot_path not in sys.path:
        sys.path.insert(0, lerobot_path)
