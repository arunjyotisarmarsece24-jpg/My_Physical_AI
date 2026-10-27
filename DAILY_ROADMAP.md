# 30-Day Physical AI Sprint Roadmap

A structured, research-grade daily curriculum designed for the 28-thread CPU architecture. Every day has a tangible coding deliverable to commit to GitHub.

---

## 📅 Week 1: Classical Dynamics & Low-Level Control
- [x] **Day 01**: Direct Joint Torque & Gravity Compensation ($M, C, g$ breakdown on inverted pendulum).
- [ ] **Day 02**: Differential Kinematics & Jacobian Inverse Kinematics (DLS / Pseudoinverse tracking).
- [ ] **Day 03**: Cartesian Task-Space Impedance Control (End-effector springs & dampers).
- [ ] **Day 04**: Trajectory Generation (Minimum jerk trajectories & quintic polynomials).
- [ ] **Day 05**: State Estimation & Kalman Filter in Simulation (Noisy IMU + joint encoder fusion).
- [ ] **Day 06**: Actuator Saturation & Anti-Windup Dynamics (Realistic DC motor models).
- [ ] **Day 07**: Week 1 Consolidation Project: 2-DoF Cartesian Trajectory Drawing in MuJoCo.

---

## 📅 Week 2: Physics Engine Mechanics & Robot Description
- [ ] **Day 08**: MuJoCo MJCF Architecture Deep Dive (Geoms, sites, equality constraints, spatial tendons).
- [ ] **Day 09**: Contact Solvers & Friction Dynamics (Elliptic vs. pyramidal friction cones, `solref`, `solimp`).
- [ ] **Day 10**: Inertial Tensor Calculation from CAD & Parallel Axis Theorem verification.
- [ ] **Day 11**: URDF to MJCF Translation & Parsing Pipeline.
- [ ] **Day 12**: Designing a 3-DoF Quadruped Leg from scratch in XML.
- [ ] **Day 13**: Modeling Gripper Contact with deformable / compliant objects.
- [ ] **Day 14**: Week 2 Consolidation Project: Multi-link gripper picking up blocks with variable friction.

---

## 📅 Week 3: Deep Reinforcement Learning (Vectorized CPU Rollouts)
- [ ] **Day 15**: Gym Environment Wrapping for Custom MuJoCo Models (`gymnasium.Env` implementation).
- [ ] **Day 16**: Vectorized Parallel Environments (Running 16–24 environments across your 28 threads).
- [ ] **Day 17**: Policy Gradients & Continuous PPO from Scratch (Actor-Critic architecture).
- [ ] **Day 18**: Reward Function Engineering & Energy Regularization.
- [ ] **Day 19**: Domain Randomization (Randomizing link masses, motor damping, and surface friction).
- [ ] **Day 20**: Sensor Delay Injection & Perturbation Robustness Testing.
- [ ] **Day 21**: Week 3 Consolidation Project: Training an arm to reach dynamic moving targets in 3D.

---

## 📅 Week 4: Imitation Learning & Physical Foundation Models
- [ ] **Day 22**: Teleoperation & Trajectory Recording (Saving state-action demonstration datasets).
- [ ] **Day 23**: Behavioral Cloning (BC) Baseline & Why Compounding Error Causes Drift.
- [ ] **Day 24**: Action Chunking with Transformers (ACT) Architecture & Temporal Ensembling.
- [ ] **Day 25**: Diffusion Policy Foundations (Denoising trajectory distributions).
- [ ] **Day 26**: Hugging Face LeRobot Setup & Dataset Formatting.
- [ ] **Day 27**: Training a Diffusion Policy on Simulated Pick-and-Place.
- [ ] **Day 28**: Vision-Language-Action (VLA) Concepts: Prompting models for 6-DoF spatial actions.
- [ ] **Day 29**: Sim-to-Real Gap Analysis (SysID, latency mismatch, reality gap mitigations).
- [ ] **Day 30**: Final Capstone Project & Open-Source Release.
