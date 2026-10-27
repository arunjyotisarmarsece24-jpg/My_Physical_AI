# Module 03: Deep Reinforcement Learning for Physical AI

This module covers continuous-action physics control with deep reinforcement learning (PPO and SAC) using MuJoCo and Gymnasium.

---

## Core Focus Areas

1. **Continuous State & Action Spaces**:
   - Observations: Joint angles $q$, angular velocities $\dot{q}$, end-effector errors, IMU readings.
   - Actions: Motor torque $\tau$ or target joint angles for low-level PD controllers.

2. **Reward Function Engineering**:
   - Task completion reward (reaching target, walking velocity).
   - Regularization penalties:
     - Torque penalty: $-\lambda_1 \|\tau\|^2$ (prevents motor burning / chattering).
     - Action rate-of-change penalty: $-\lambda_2 \|\tau_t - \tau_{t-1}\|^2$ (prevents high-frequency jerk).
     - Joint limit violation penalties.

3. **Domain Randomization (The Sim-to-Real Bridge)**:
   - Randomizing link masses ($m \pm 20\%$).
   - Randomizing surface friction ($\mu \in [0.4, 1.2]$).
   - Simulating actuator latency ($10-30\text{ ms}$).

---

## Recommended Free Tools

- **Algorithm Implementation**: [CleanRL](https://github.com/vwxyzjn/cleanrl) (single-file PPO and SAC) or [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3).
- **Benchmark Environments**: `Gymnasium-Robotics` (MuJoCo Reacher, Fetch, Ant, Humanoid).
