# Hunter PyBullet RL

## 1. Overview

This project demonstrates a **full RL pipeline** for the **AgileX Hunter robot** running on **PyBullet**,
covering everything from simulation setup → sensor integration → RL training → CI/CD automation.

It is designed as a **series** for robotics & reinforcement learning practitioners who want to build
a reproducible workflow that connects RL (Python) ↔ Runtime logic (C++).
Can be found here: [Hunter RL on PyBullet – Full Series](#)

---

## 2. Roadmap – **Season 1: Hunter RL on PyBullet**

| **Stage** |  **Episode** | **Title** | **Goal** | **Output** |**Status** |**Tag**|
|------------|---|--------------|-----------|-----------|-------------|---|
| **Stage 1 – Simulation & Setup Base** | EP1 | Intro: Hunter Robot & PyBullet | Overview & motivation | Hunter run demo ||
|  | EP2 | Create URDF + Env | Create URDF robot & environment | URDF load ok ||
|  | EP3 | Build Simulation Base | Gravity, timestep, keyboard drive | Hunter moves ||
| | EP4 | Add Sensors (Lidar, IMU) | Realistic observation | Sensor data OK ||
|  | EP5 | Build Gym Env (`HunterEnv`) | reset/step/render loop | Env runs 1 episode ||
| **Stage 2 – RL Core Pipeline** |EP6 | Reward Design | Reward shaping & done logic | Reasonable reward ||
|  | EP7 | Train PPO Agent | RL training with SB3 | Model `.zip` ||
|  | EP8 | Visualization | TensorBoard & stats | Reward curves ||
|  | EP9 | Evaluation & Checkpoint | Load model, test agent | Hunter self-driving ||
| **Stage 3 – Integration & DevOps** |EP10 | RL + FSM Integration | Combine with Behavior FSM (C++) | Hybrid demo ||
|  | EP11 | Debug & Tuning | Analyze hyperparams, reward curve | Improved policy ||
|  | EP12 | Docker + CI/CD + Summary | Build, test, deploy (Python + C++) | Container + GitHub Actions ||


**Notes**
- Each EP = one branch
- Merge → main after full test

---
## 3. Project Structure

```bash
hunter_pybullet_rl/
├── README.md
└── src                             # Main source code
    ├── bringup
    ├── control_cpp
    ├── hunter_model                # URDF model package
    ├── interfaces
    └── pybullet_sim                # PyBullet simulation package
        ├── config                  # Configuration files
        ├── launch                  # Launch files
        │   └── sim.launch.py
        ├── package.xml             # ROS 2 package manifest
        ├── pybullet_sim            # Python package
        │   ├── __init__.py
        │   └── sim_node.py
        ├── resource                # Resource files
        │   └── pybullet_sim
        ├── setup.cfg               # Package configuration
        ├── setup.py                # Package setup
        └── urdf
            └── hunter_model.urdf

```

## 4. Quick Start

```bash
# 1. Clone repo
git clone https://github.com/phatcvo/hunter_pybullet_rl.git
cd hunter_pybullet_rl

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run simulation
cd ~/hunter_pybullet_rl
colcon build --symlink-install
source install/setup.bash
ros2 launch pybullet_sim sim.launch.py
