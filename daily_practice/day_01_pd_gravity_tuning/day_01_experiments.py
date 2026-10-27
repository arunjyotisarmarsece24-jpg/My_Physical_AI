"""
Day 01: Classical Dynamics, Energy Conservation & Phase Portrait Analysis
System: Inverted Pendulum on a Revolute Joint

Author: Arunjyoti Sarma (IIT BHU)
Physics Engine: MuJoCo (Google DeepMind)

Rigorous Experiments:
  1. Critical Stiffness Threshold (K_p vs. Gravity Torque m*g*l)
  2. Energy Drift Test (Symplectic check: E = T + V with zero damping)
  3. State-Space Phase Portrait Generation (theta vs. theta_dot)
"""

import os
import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import argparse

XML_EXPERIMENT = """
<mujoco model="inverted_pendulum_day1">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>
  <worldbody>
    <light diffuse=".6 .6 .6" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1.5 1.5 0.1" rgba="0.95 0.95 0.95 1"/>
    <body name="base" pos="0 0 0.8">
      <geom type="cylinder" size="0.04 0.05" rgba="0.2 0.2 0.2 1"/>
      <body name="pole" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="0.0"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.5" size="0.015" rgba="0.1 0.4 0.8 1" mass="0.5"/>
        <body name="bob" pos="0 0 0.5">
          <geom type="sphere" size="0.04" rgba="0.85 0.2 0.2 1" mass="0.5"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="torque_motor" joint="hinge" gear="1" ctrllimited="true" ctrlrange="-25.0 25.0"/>
  </actuator>
</mujoco>
"""

def experiment_energy_conservation(duration=5.0):
    """
    Test 1: Free undamped swinging. 
    Verifies if RK4 integration in MuJoCo conserves Hamiltonian Energy E = T + V.
    """
    print("\n[EXPERIMENT 1]: Hamiltonian Energy Conservation Test")
    model = mujoco.MjModel.from_xml_string(XML_EXPERIMENT)
    data = mujoco.MjData(model)

    # Initial perturbation: release from theta = 45 degrees
    data.qpos[0] = np.radians(45.0)
    data.qvel[0] = 0.0
    data.ctrl[0] = 0.0  # Zero motor torque

    steps = int(duration / model.opt.timestep)
    energies = []
    times = []

    m_total = 1.0       # 0.5 kg rod + 0.5 kg bob
    r_com = 0.375       # Center of mass distance
    g = 9.81
    # Rotational inertia about pivot I = I_rod + I_bob = (1/3)*m_rod*l^2 + m_bob*l^2
    I_pivot = (1.0/3.0) * 0.5 * (0.5**2) + 0.5 * (0.5**2)  # ~ 0.1667 kg*m^2

    for _ in range(steps):
        theta = data.qpos[0]
        theta_dot = data.qvel[0]

        # Potential energy (relative to pivot at z=0): V = m * g * r_com * cos(theta)
        V = m_total * g * r_com * np.cos(theta)
        # Kinetic energy: T = 0.5 * I * theta_dot^2
        T = 0.5 * I_pivot * (theta_dot ** 2)
        E_total = T + V

        energies.append(E_total)
        times.append(data.time)
        mujoco.mj_step(model, data)

    energy_drift = abs(energies[-1] - energies[0])
    print(f"  Initial Energy: {energies[0]:.4f} Joules")
    print(f"  Final Energy  : {energies[-1]:.4f} Joules")
    print(f"  Energy Drift  : {energy_drift:.6f} Joules (Numerical dissipation / drift)")
    return times, energies

def experiment_phase_portrait():
    """
    Test 2: Generate Phase Space Portraits (theta vs. theta_dot)
    Under 3 Controller Conditions:
      Case A: Pure PD with Kp < Critical (Failure / Collapse)
      Case B: High Kp without Gravity Compensation (Steady-State Droop)
      Case C: Computed Torque (PD + Exact Gravity Compensation)
    """
    print("\n[EXPERIMENT 2]: Phase Portrait Generation & Controller Comparison")
    model = mujoco.MjModel.from_xml_string(XML_EXPERIMENT)
    data = mujoco.MjData(model)

    m_total = 1.0
    r_com = 0.375
    g = 9.81
    tau_gravity_max = m_total * g * r_com  # ~ 3.67875 Nm (Critical stiffness Kp_crit)
    print(f"  Critical Restoring Stiffness Kp_crit = m*g*r_com = {tau_gravity_max:.3f} Nm/rad")

    initial_angles = [np.radians(15), np.radians(30), np.radians(45), np.radians(60)]
    
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    titles = [
        "Case A: Sub-Critical (Kp=2.0 < Kp_crit)\nInstability / Fall",
        "Case B: Linear PD (Kp=20.0, Kd=2.0)\nNo Gravity Compensation",
        "Case C: Computed Torque (PD + GravComp)\nGlobal Asymptotic Stability"
    ]

    for ax, case_id in zip(axes, [0, 1, 2]):
        for theta_0 in initial_angles:
            traj_theta = []
            traj_theta_dot = []

            mujoco.mj_resetData(model, data)
            data.qpos[0] = theta_0
            data.qvel[0] = 0.0

            steps = int(4.0 / model.opt.timestep)
            for _ in range(steps):
                theta = data.qpos[0]
                theta_dot = data.qvel[0]

                if case_id == 0:
                    # Sub-critical Kp
                    tau = -2.0 * theta - 0.5 * theta_dot
                elif case_id == 1:
                    # Stiff linear PD
                    tau = -25.0 * theta - 3.0 * theta_dot
                else:
                    # Computed Torque Control (PD + Gravity Cancellation)
                    tau_grav = m_total * g * r_com * np.sin(theta)
                    tau = -25.0 * theta - 3.0 * theta_dot + tau_grav

                data.ctrl[0] = tau
                mujoco.mj_step(model, data)

                traj_theta.append(np.degrees(theta))
                traj_theta_dot.append(np.degrees(theta_dot))

            ax.plot(traj_theta, traj_theta_dot, label=f"θ₀={np.degrees(theta_0):.0f}°")

        ax.axvline(0, color='black', linestyle='--', alpha=0.4)
        ax.axhline(0, color='black', linestyle='--', alpha=0.4)
        ax.set_title(titles[case_id], fontsize=11, fontweight='bold')
        ax.set_xlabel("Joint Angle θ (deg)")
        ax.set_ylabel("Angular Velocity θ̇ (deg/s)")
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.tight_layout()
    output_plot = os.path.join(os.path.dirname(__file__), "phase_portrait.png")
    plt.savefig(output_plot, dpi=200)
    print(f"  Phase portrait saved to: {output_plot}")

def run_interactive_tuning():
    """
    Interactive Real-time Visualizer for tuning Kp, Kd and experiencing disturbance response.
    """
    model = mujoco.MjModel.from_xml_string(XML_EXPERIMENT)
    data = mujoco.MjData(model)

    data.qpos[0] = np.radians(30.0)
    m_total = 1.0
    r_com = 0.375
    g = 9.81
    Kp = 35.0
    Kd = 4.0

    print("\n[INTERACTIVE BENCHMARK]")
    print("  Click & drag the pole with your right mouse button in MuJoCo to apply disturbance forces!")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            theta = data.qpos[0]
            theta_dot = data.qvel[0]

            # Computed torque law
            tau_grav = m_total * g * r_com * np.sin(theta)
            tau_cmd = -Kp * theta - Kd * theta_dot + tau_grav
            data.ctrl[0] = tau_cmd

            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Day 01 Physical AI Diagnostics")
    parser.add_argument("--interactive", action="store_true", help="Launch interactive 3D visualizer")
    args = parser.parse_args()

    # Always execute analytical experiments
    experiment_energy_conservation()
    experiment_phase_portrait()

    if args.interactive:
        run_interactive_tuning()
