"""
Phase 1: Classical Control & Rigid Body Dynamics
Script 02: Double Pendulum Chaos & Dynamic Coupling

Author: Arunjyoti Sarma (IIT BHU)
Physics Engine: MuJoCo (Google DeepMind)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import argparse

DOUBLE_PENDULUM_XML = """
<mujoco model="double_pendulum">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>

  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8"/>
  </visual>

  <worldbody>
    <light diffuse=".6 .6 .6" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1.5 1.5 0.1" rgba="0.95 0.95 0.95 1"/>

    <!-- Fixed base -->
    <body name="base" pos="0 0 1.2">
      <geom type="cylinder" size="0.05 0.04" rgba="0.2 0.2 0.2 1"/>

      <!-- Link 1 -->
      <body name="link1" pos="0 0 0">
        <joint name="joint1" type="hinge" axis="0 1 0" damping="0.01"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.02" rgba="0.2 0.4 0.9 1" mass="1.0"/>
        
        <!-- Link 2 attached to tip of Link 1 -->
        <body name="link2" pos="0 0 -0.4">
          <joint name="joint2" type="hinge" axis="0 1 0" damping="0.01"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.015" rgba="0.9 0.3 0.2 1" mass="0.8"/>
          <body name="tip" pos="0 0 -0.4">
            <geom type="sphere" size="0.035" rgba="0.95 0.7 0.1 1" mass="0.3"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="tau_j1" joint="joint1" gear="1" ctrllimited="true" ctrlrange="-30.0 30.0"/>
    <motor name="tau_j2" joint="joint2" gear="1" ctrllimited="true" ctrlrange="-20.0 20.0"/>
  </actuator>
</mujoco>
"""

def run_simulation(headless: bool = False, duration: float = 10.0):
    model = mujoco.MjModel.from_xml_string(DOUBLE_PENDULUM_XML)
    data = mujoco.MjData(model)

    # Release from horizontal position (theta1 = 90 deg, theta2 = 0 deg)
    mujoco.mj_resetData(model, data)
    data.qpos[0] = np.pi / 2.0
    data.qpos[1] = 0.0

    print("=" * 60)
    print("Physical AI - Double Pendulum Nonlinear Coupling")
    print(f"Degrees of Freedom: {model.nv}")
    print("Zero actuation applied (Free chaotic gravitational swinging)")
    print("=" * 60)

    if headless:
        steps = int(duration / model.opt.timestep)
        for step in range(steps):
            mujoco.mj_step(model, data)
            if step % 250 == 0:
                print(f"Time: {data.time:.3f}s | Joint 1: {np.degrees(data.qpos[0]):.1f}° | Joint 2: {np.degrees(data.qpos[1]):.1f}°")
        print("\nHeadless simulation completed successfully.")
        return

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            start_wall = time.time()
            while viewer.is_running():
                step_start = time.time()
                mujoco.mj_step(model, data)
                viewer.sync()

                elapsed = time.time() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if (time.time() - start_wall) > duration:
                    break
    except Exception as e:
        print(f"Viewer launch issue ({e}). Running in headless mode instead...")
        run_simulation(headless=True, duration=duration)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Double Pendulum Simulation")
    parser.add_argument("--headless", action="store_true", help="Run without UI window")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds")
    args = parser.parse_args()
    run_simulation(headless=args.headless, duration=args.duration)
