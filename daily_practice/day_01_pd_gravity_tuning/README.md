# Day 01: Classical Dynamics, Stability & Phase Space Portraits

System: Inverted Pendulum on a Revolute Joint with Direct Torque Actuation  
Physics Engine: MuJoCo (Google DeepMind)

---

## 🔬 Mathematical Formulation

### 1. Nonlinear Equation of Motion
The dynamics of a single-link inverted pendulum actuated by joint torque $\tau$ are governed by:

$$I_{\text{pivot}} \ddot{\theta} + b \dot{\theta} - m g r_{\text{com}} \sin\theta = \tau$$

Where:
- $m = 1.0\text{ kg}$ (Total mass: rod + bob)
- $r_{\text{com}} = 0.375\text{ m}$ (Distance from pivot to center of mass)
- $I_{\text{pivot}} = \frac{1}{3} m_{\text{rod}} l^2 + m_{\text{bob}} l^2 \approx 0.1667\text{ kg}\cdot\text{m}^2$
- $\tau_{\text{grav}}(\theta) = m g r_{\text{com}} \sin\theta$ is the destabilizing gravitational torque.

---

## 💡 The "Bitter / Brutal" Physics Insights

### Insight 1: The Critical Stiffness Limit ($K_{p,\text{crit}}$)
For small perturbations around the upright equilibrium ($\theta \approx 0$), $\sin\theta \approx \theta$.  
Applying a standard linear PD controller ($\tau = -K_p \theta - K_d \dot{\theta}$), the closed-loop system is:

$$I_{\text{pivot}} \ddot{\theta} + (b + K_d)\dot{\theta} + (K_p - m g r_{\text{com}})\theta = 0$$

- **If $K_p < m g r_{\text{com}} = 3.679\text{ Nm/rad}$**: The effective restoring stiffness is **negative**. The upright position is an unstable saddle point, and the system collapses under gravity regardless of damping $K_d$.
- **If $K_p > 3.679\text{ Nm/rad}$**: The system is locally stable, but for large angles $\theta > 45^\circ$, the linear approximation fails and the system exhibits significant steady-state droop or limit cycles.

### Insight 2: Feedback Linearization (Computed Torque Control)
To achieve global asymptotic stability across all angles $\theta \in (-\pi, \pi)$, we feedforward the exact analytical gravity cancellation term:

$$\tau = -K_p \theta - K_d \dot{\theta} + \underbrace{m g r_{\text{com}} \sin\theta}_{\tau_{\text{gravity}}}$$

Substituting $\tau$ back into the equations of motion yields:

$$I_{\text{pivot}} \ddot{\theta} + (b + K_d)\dot{\theta} + K_p \theta = 0$$

The nonlinear gravity term is completely eliminated. The closed-loop system behaves as a pure linear, globally stable second-order harmonic oscillator!

---

## 📊 Phase Space Portraits

The generated phase portraits (`phase_portrait.png`) show the state trajectories in $(\theta, \dot{\theta})$ space across 3 scenarios:

1. **Case A (Sub-Critical $K_p = 2.0 < 3.679$)**: Trajectories diverge away from $(0, 0)$ and tumble towards the downward hanging equilibrium ($\theta = 180^\circ$).
2. **Case B (High Linear PD $K_p = 25.0$)**: Trajectories spiral inward, but struggle with large deflections due to uncompensated nonlinear gravity.
3. **Case C (Computed Torque Control)**: Smooth, textbook exponential spirals converging to $(0, 0)$ from any initial deflection angle.

---

## 🏃‍♂️ How to Run

```bash
# 1. Run mathematical verification & generate phase portrait plots:
python daily_practice/day_01_pd_gravity_tuning/day_01_experiments.py

# 2. Launch interactive 3D visualizer to physically disturb the pendulum:
python daily_practice/day_01_pd_gravity_tuning/day_01_experiments.py --interactive
```
