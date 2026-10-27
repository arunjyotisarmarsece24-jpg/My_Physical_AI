# Physical AI & Simulation Laboratory

[![MuJoCo](https://img.shields.io/badge/Physics%20Engine-MuJoCo%203.17-blue.svg)](https://mujoco.org)
[![Python](https://img.shields.io/badge/Python-3.14%2B-brightgreen.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-yellow.svg)](LICENSE)
[![Author](https://img.shields.io/badge/Author-Arunjyoti%20Sarma%20(IIT%20BHU)-orange.svg)](https://github.com/arunjyotisarmarsece24-jpg)

A rigorous, **100% free and open-source** codebase for learning, practicing, and mastering **Physical AI (Embodied Intelligence, Rigid Body Dynamics, Physics Simulation Design, and Robot Learning)**.

---

## 🖥️ Target Hardware Optimization & Profiling

This repository is optimized for high-performance physics execution on this machine:

- **CPU**: Intel® Core™ i7-14700T (20 Cores: 8 P-Cores, 12 E-Cores / 28 Threads)
- **RAM**: 16 GB DDR5
- **GPU**: Intel® UHD Graphics 770 (Integrated)
- **Physics Engine Strategy**: 
  - Standard robotic simulators (e.g., NVIDIA Isaac Sim) require high-end dedicated NVIDIA RTX GPUs. 
  - **MuJoCo (Google DeepMind)** is an optimized C/C++ multithreaded analytical physics engine. Leveraging the **28 logical cores** of the Intel i7-14700T allows running **hundreds of parallel vectorized environment rollouts simultaneously on the CPU** with zero licensing costs or paid hardware requirements.

---

## 📂 Repository Architecture

```text
My_Physical_AI/
├── DAILY_ROADMAP.md                          # 30-Day Physical AI Sprint Checklist
├── daily_practice/                           # Daily coding challenges & lab logs
│   ├── README.md
│   └── day_02_jacobian_ik/                   # Differential kinematics & DLS tracking
│       └── jacobian_inverse_kinematics.py
├── 01_classical_control_and_dynamics/
│   ├── 01_inverted_pendulum_pd_gravcomp.py   # Computed torque control (PD + gravity cancelation)
│   └── 02_double_pendulum_chaos.py           # Nonlinear coupled Lagrangian dynamics & chaotic motion
├── 02_robot_modeling_mjcf/
│   ├── models/
│   │   └── planar_2dof_arm.xml               # Custom MJCF 2-DoF robotic manipulator description
│   └── 01_load_and_inspect_model.py          # Forward kinematics, Jacobian J(q), Inertia M(q), and bias g(q)
├── 03_deep_reinforcement_learning/
│   └── README.md                             # Continuous-action PPO/SAC & Domain Randomization guides
├── 04_imitation_learning_and_vla/
│   └── README.md                             # Action Chunking (ACT), Diffusion Policy & OpenVLA guides
├── requirements.txt                          # Core dependencies
└── README.md
```

---

## 🚀 Quickstart & Execution

### 1. Environment Verification
The core physics dependencies are already verified on this system:
```bash
python -c "import mujoco; print('MuJoCo Version:', mujoco.__version__)"
```

### 2. Run Classical Control Benchmark
Run the torque-controlled inverted pendulum with gravity compensation:
```bash
# Interactive visualizer mode:
python 01_classical_control_and_dynamics/01_inverted_pendulum_pd_gravcomp.py

# Headless benchmark mode:
python 01_classical_control_and_dynamics/01_inverted_pendulum_pd_gravcomp.py --headless
```

### 3. Run Chaotic Nonlinear Dynamics
Simulate unactuated, coupled Lagrangian dynamics:
```bash
python 01_classical_control_and_dynamics/02_double_pendulum_chaos.py --headless
```

### 4. Extract Robot Kinematics & Analytical Matrices
Inspect the 2-DoF manipulator model to compute the End-Effector Cartesian position, translational Jacobian, mass matrix, and gravity bias:
```bash
python 02_robot_modeling_mjcf/01_load_and_inspect_model.py
```

---

## 📐 Mathematical Foundations

### Manipulator Dynamics Equation
$$\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}}$$

Where:
- $\mathbf{M}(\mathbf{q})$ is the symmetric positive-definite mass / inertia matrix.
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$ represents Coriolis and centripetal forces.
- $\mathbf{g}(\mathbf{q})$ is the gravitational bias vector.
- $\boldsymbol{\tau}$ is the generalized joint torque command.
- $\mathbf{J}(\mathbf{q})$ is the geometric manipulator Jacobian mapping joint velocities to Cartesian task space velocities: $\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}}$.

---

## 🎯 Syllabus & Roadmap

1. **Phase 1: Classical Dynamics & Control** (Inverted Pendulums, Computed Torque, Impedance Control).
2. **Phase 2: Simulation Engineering** (MJCF XML, URDF, contact friction cones, restitution).
3. **Phase 3: Deep Reinforcement Learning** (Continuous PPO, SAC, reward shaping, domain randomization).
4. **Phase 4: Modern Robot Learning** (ACT, Diffusion Policy with Hugging Face LeRobot, Sim-to-Real).
