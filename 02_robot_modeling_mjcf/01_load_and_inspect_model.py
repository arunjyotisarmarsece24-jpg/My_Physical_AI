"""
Phase 2: Robot Modeling & Dynamics Extraction
Script 01: Kinematics, Jacobians, and Mass Matrix from MJCF

Author: Arunjyoti Sarma (IIT BHU)
Physics Engine: MuJoCo (Google DeepMind)
"""

import os
import mujoco
import numpy as np

def inspect_robot():
    model_path = os.path.join(os.path.dirname(__file__), "models", "planar_2dof_arm.xml")
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found at: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print("=" * 65)
    print("Physical AI - Robot Kinematics & Dynamics Inspection")
    print(f"Robot Name: {model.names.decode().split(chr(0))[0]}")
    print(f"Generalized coordinates (nq): {model.nq} | Degrees of freedom (nv): {model.nv}")
    print(f"Actuators: {model.nu}")
    print("=" * 65)

    # Set joint configuration: shoulder = 45 deg (0.785 rad), elbow = -30 deg (-0.524 rad)
    data.qpos[0] = np.radians(45.0)
    data.qpos[1] = np.radians(-30.0)
    data.qvel[:] = 0.0

    # Compute Forward Kinematics
    mujoco.mj_forward(model, data)

    # 1. Forward Kinematics of End-Effector Site
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    ee_pos = data.site_xpos[site_id]
    print(f"\n[1] Forward Kinematics (End-Effector Cartesian Position):")
    print(f"    X: {ee_pos[0]:.4f} m | Y: {ee_pos[1]:.4f} m | Z: {ee_pos[2]:.4f} m")

    # 2. Translational & Rotational Jacobians
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    print(f"\n[2] Translational Jacobian Matrix J_pos (3x2):")
    print(np.array2string(jacp, precision=4, suppress_small=True))

    # 3. Mass / Inertia Matrix M(q)
    # In MuJoCo 3.x: mj_fullM(model, data, dst)
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, M)
    print(f"\n[3] Joint Space Mass / Inertia Matrix M(q) (2x2):")
    print(np.array2string(M, precision=4, suppress_small=True))

    # 4. Gravity & Coriolis Vector: bias force C(q, qdot) + g(q)
    print(f"\n[4] Gravity Bias Vector g(q) (Nm):")
    print(np.array2string(data.qfrc_bias, precision=4, suppress_small=True))
    print("\nRobot dynamics verification successful.")
    print("=" * 65)

if __name__ == "__main__":
    inspect_robot()
