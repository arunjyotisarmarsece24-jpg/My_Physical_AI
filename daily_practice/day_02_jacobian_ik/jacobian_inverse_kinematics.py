"""
Day 02: Differential Kinematics & Jacobian Inverse Kinematics (IK)
Method: Damped Least Squares (DLS / Levenberg-Marquardt style)

Author: Arunjyoti Sarma (IIT BHU)
Physics Engine: MuJoCo (Google DeepMind)

Math:
  End-Effector Error: e = x_target - x_current
  Jacobian: J = d(x_ee) / dq
  DLS Pseudoinverse: J_dls = J^T * (J * J^T + lambda^2 * I)^(-1)
  Joint Velocity Command: q_vel_cmd = alpha * J_dls * e
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import argparse

ROBOT_IK_XML = """
<mujoco model="planar_arm_ik">
  <compiler angle="radian" coordinate="local"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8"/>
  </visual>

  <default>
    <joint damping="0.5" armature="0.02"/>
  </default>

  <worldbody>
    <light diffuse=".8 .8 .8" pos="0 0 4" dir="0 0 -1"/>
    <geom type="plane" size="2 2 0.1" rgba="0.95 0.95 0.95 1"/>

    <!-- Target Indicator (Mocap body moved programmatically) -->
    <body name="target" mocap="true" pos="0.5 0 0.6">
      <geom type="sphere" size="0.03" rgba="1 0.1 0.1 0.7" contype="0" conaffinity="0"/>
    </body>

    <!-- 2-DoF Planar Robot Arm -->
    <body name="base" pos="0 0 0.4">
      <geom type="cylinder" size="0.08 0.2" rgba="0.25 0.25 0.25 1"/>

      <body name="link1" pos="0 0 0.2">
        <joint name="shoulder" type="hinge" axis="0 1 0" range="-2.8 2.8"/>
        <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.03" rgba="0.2 0.5 0.85 1" mass="1.5"/>

        <body name="link2" pos="0.4 0 0">
          <joint name="elbow" type="hinge" axis="0 1 0" range="-2.8 2.8"/>
          <geom type="capsule" fromto="0 0 0 0.35 0 0" size="0.025" rgba="0.85 0.3 0.2 1" mass="1.0"/>
          
          <!-- End-Effector Tracking Site -->
          <site name="ee_site" pos="0.35 0 0" size="0.025" rgba="0.1 0.9 0.2 1"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- High-gain PD Position Actuators for Resolved-Rate Kinematic Tracking -->
    <position name="shoulder_pos" joint="shoulder" kp="500" kv="50" ctrllimited="true" ctrlrange="-2.8 2.8"/>
    <position name="elbow_pos" joint="elbow" kp="500" kv="50" ctrllimited="true" ctrlrange="-2.8 2.8"/>
  </actuator>
</mujoco>
"""

def run_ik_tracking(headless: bool = False, duration: float = 12.0):
    model = mujoco.MjModel.from_xml_string(ROBOT_IK_XML)
    data = mujoco.MjData(model)

    # Initial joint positions (elbow bent naturally upward)
    data.qpos[0] = -0.5
    data.qpos[1] = 0.8
    mujoco.mj_forward(model, data)

    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    mocap_id = model.body_mocapid[target_body_id]

    # Target trajectory parameters (circle in X-Z plane around center)
    center_x, center_z = 0.40, 0.60
    radius = 0.08
    freq = 0.2  # 0.2 Hz (5 seconds per orbit)

    damping = 1e-4
    step_rate = 0.05  # Smooth integration step rate per physics tick

    q_des = np.copy(data.qpos)

    def compute_ik_step(sim_time: float):
        nonlocal q_des
        # 1. Update target position along circular path
        target_x = center_x + radius * np.cos(2 * np.pi * freq * sim_time)
        target_z = center_z + radius * np.sin(2 * np.pi * freq * sim_time)
        data.mocap_pos[mocap_id] = np.array([target_x, 0.0, target_z])

        # 2. Get current End-Effector position
        current_ee = data.site_xpos[ee_site_id]

        # 3. Position error e in 3D: [e_x, e_y, e_z]
        error = data.mocap_pos[mocap_id] - current_ee

        # 4. Compute Translational Jacobian J (3 x nv)
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, ee_site_id)

        # 5. Damped Least Squares Inverse: J_dls = J^T * (J * J^T + lambda^2 * I)^(-1)
        JJT = jacp @ jacp.T
        damped_inv = np.linalg.inv(JJT + (damping ** 2) * np.eye(3))
        J_dls = jacp.T @ damped_inv

        # 6. Integrated joint position update: Delta q = step_rate * J_dls * error
        delta_q = step_rate * (J_dls @ error)
        q_des += delta_q

        # Clamp within joint limits
        q_des = np.clip(q_des, -2.8, 2.8)
        data.ctrl[:] = q_des
        return np.linalg.norm(error)

    if headless:
        steps = int(duration / model.opt.timestep)
        for step in range(steps):
            err_norm = compute_ik_step(data.time)
            mujoco.mj_step(model, data)
            if step % 500 == 0:
                print(f"Time: {data.time:.3f}s | Tracking Error: {err_norm * 1000.0:.2f} mm | Joint Vels: {data.ctrl[:]}")
        print("\nHeadless IK tracking completed successfully.")
        return

    # Visualizer mode
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            start_wall = time.time()
            while viewer.is_running():
                step_start = time.time()
                compute_ik_step(data.time)
                mujoco.mj_step(model, data)
                viewer.sync()

                elapsed = time.time() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if (time.time() - start_wall) > duration:
                    print(f"\nCompleted {duration}s interactive IK tracking.")
                    break
    except Exception as e:
        print(f"Viewer launch issue ({e}). Running headless instead...")
        run_ik_tracking(headless=True, duration=duration)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Jacobian Inverse Kinematics Tracker")
    parser.add_argument("--headless", action="store_true", help="Run without UI window")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds")
    args = parser.parse_args()
    run_ik_tracking(headless=args.headless, duration=args.duration)
