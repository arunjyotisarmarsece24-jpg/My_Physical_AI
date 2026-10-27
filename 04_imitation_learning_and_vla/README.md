# Module 04: Imitation Learning & Vision-Language-Action (VLA)

This module focuses on sample-efficient manipulation learning via demonstration and physical foundation models.

---

## Core Focus Areas

1. **Behavioral Cloning vs Modern Imitation Learning**:
   - Why naive MLP Behavioral Cloning suffers from compounding drift errors.
   - **ACT (Action Chunking with Transformers)**: Predicting temporal action sequences chunked together to ensure smooth trajectories.
   - **Diffusion Policy**: Formulating robotic action generation as a conditional denoising diffusion process over trajectory space.

2. **Hugging Face LeRobot Integration**:
   - Completely free and open-source platform for PyTorch-based robotics.
   - Native simulated teleoperation environments (e.g. simulated ALOHA arm, SO-100).
   - Dataset format: Camera image streams + proprioception (joint angles) + commanded action sequences.

3. **Open-Source Robot Foundation Models (VLA)**:
   - **OpenVLA** (7B parameter open-source VLA model based on Llama-2 / Prismatic).
   - **Octo** (Generalist robot policy trained on Open X-Embodiment).
