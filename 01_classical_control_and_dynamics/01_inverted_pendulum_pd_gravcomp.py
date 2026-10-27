"""
Phase 1: Classical Control & Rigid Body Dynamics
Script 01: Torque-Controlled Inverted Pendulum with PD + Gravity Compensation

Author: Arunjyoti Sarma (IIT BHU)
Physics Engine: MuJoCo (Google DeepMind)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import argparse

# XML definition of an Inverted Pendulum on a motorized revolute hinge
PENDULUM_XML = """
<mujoco model="inverted_pendulum">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>
  
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8" specular="0.1 0.1 0.1"/>
  </visual>

  <worldbody>
    <light diffuse=".6 .6 .6" pos="0 0 3" dir="0 0 -1"/>
    <!-- Floor plane -->
    <geom type="plane" size="1.5 1.5 0.1" rgba="0.9 0.9 0.9 1"/>
    
    <!-- Base mount -->
    <body name="base" pos="0 0 0.8">
      <geom type="cylinder" size="0.04 0.05" rgba="0.2 0.2 0.2 1"/>
      
      <!-- Inverted Pendulum Pole -->
      <body name="pole" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>
        <!-- Rod: length = 0.5m, radius = 0.015m -->
        <geom type="capsule" fromto="0 0 0 0 0 0.5" size="0.015" rgba="0.1 0.4 0.8 1" mass="0.5"/>
        <!-- Bob / Tip Mass -->
        <body name="bob" pos="0 0 0.5">
          <geom type="sphere" size="0.04" rgba="0.85 0.2 0.2 1" mass="0.5"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Raw direct torque motor applied to the hinge joint -->
    <motor name="torque_actuator" joint="hinge" gear="1" ctrllimited="true" ctrlrange="-15.0 15.0"/>
  </actuator>
</mujoco>
"""

def run_simulation(headless: bool = False, duration: float = 10.0):
    model = mujoco.MjModel.from_xml_string(PENDULUM_XML)
    data = mujoco.MjData(model)

    # Initial perturbation: displace pole by +0.35 radians (~20 degrees)
    mujoco.mj_resetData(model, data)
    data.qpos[0] = 0.35
    data.qvel[0] = 0.0

    # System parameters for analytical gravity compensation:
    # m_total = 0.5 (rod) + 0.5 (bob) = 1.0 kg
    # Center of mass distance ~ 0.375 m
    # g = 9.81 m/s^2
    m_rod = 0.5
    l_rod = 0.5
    m_bob = 0.5
    l_bob = 0.5
    g = 9.81
    r_com = (m_rod * (l_rod / 2.0) + m_bob * l_bob) / (m_rod + m_bob)
    m_total = m_rod + m_bob

    # PD Controller Gains
    Kp = 40.0   # Proportional gain (stiffness)
    Kd = 5.0    # Derivative gain (damping)

    print("=" * 60)
    print("Physical AI - Classical Control Benchmark")
    print(f"Total Mass: {m_total:.2f} kg | CoM length: {r_com:.3f} m")
    print(f"Controller: Computed Torque (PD + Gravity Compensation)")
    print(f"Target: theta = 0.0 rad (upright equilibrium)")
    print("=" * 60)

    if headless:
        steps = int(duration / model.opt.timestep)
        for step in range(steps):
            theta = data.qpos[0]
            theta_dot = data.qvel[0]

            # Gravity compensation torque: cancels out m * g * r_com * sin(theta)
            tau_grav = m_total * g * r_com * np.sin(theta)
            tau_pd = -Kp * theta - Kd * theta_dot
            tau_cmd = tau_pd + tau_grav

            data.ctrl[0] = tau_cmd
            mujoco.mj_step(model, data)

            if step % 250 == 0:
                print(f"Time: {data.time:.3f}s | Theta: {np.degrees(theta):.2f}° | Torque: {tau_cmd:.3f} Nm")
        print("\nHeadless simulation completed successfully.")
        return

    # Interactive visualizer mode
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            start_wall = time.time()
            while viewer.is_running():
                step_start = time.time()

                theta = data.qpos[0]
                theta_dot = data.qvel[0]

                # Full dynamics control equation: tau = tau_pd + tau_grav
                tau_grav = m_total * g * r_com * np.sin(theta)
                tau_pd = -Kp * theta - Kd * theta_dot
                data.ctrl[0] = tau_pd + tau_grav

                mujoco.mj_step(model, data)
                viewer.sync()

                # Maintain realistic real-time physics pacing
                elapsed = time.time() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if (time.time() - start_wall) > duration:
                    print(f"\nCompleted {duration}s interactive simulation.")
                    break
    except Exception as e:
        print(f"Viewer launch issue ({e}). Running in headless mode instead...")
        run_simulation(headless=True, duration=duration)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Torque-Controlled Inverted Pendulum")
    parser.add_argument("--headless", action="store_true", help="Run without UI window")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds")
    args = parser.parse_args()
    run_simulation(headless=args.headless, duration=args.duration)
