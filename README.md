# SynchroHumanoid

SynchroHumanoid is a unified research stack for humanoid robots that synchronizes **vision**, **language**, and **action** across **simulation** and **real-world** environments with **digital twins** at its core.

This repository is intended as a modular playground for:

- Vision-Language Models (VLM)
- Vision-Language-Action (VLA) policies
- Sim-to-Real transfer for humanoid control
- High-fidelity humanoid digital twin pipelines

---

## Goals

- Build a **single coherent pipeline** from perception → reasoning → control for humanoids.
- Prototype and evaluate **VLM/VLA-based skills** (navigation, manipulation, interaction).
- Validate policies in **simulation first**, then deploy on real hardware with minimal gap.
- Maintain a **digital twin** for repeatable experiments and debugging.


---

## Getting Started

```bash
git clone --recurse-submodules https://github.com/cananella/SynchroHumanoid.git
cd SynchroHumanoid
# Setup instructions will be added here (env, deps, simulator, robot API, etc.)
uv sync
```

---

## Projects
### Common
* **Activate venv**
```bash
source activate
```
* **Activate open arm ros workspace**
```bash
source src/openarm_ros_ws/install/setup.bash
```

### [fr5 pybrain (fr5 vr controller)](projects/fr5_pytbrain/README_RECORD.md)
