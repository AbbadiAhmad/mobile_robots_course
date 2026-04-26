# MOD-06: Mobile Robot Localization

## Student Script — Full Mathematical Foundations

**Course:** Mobile Robots  
**Instructor:** Dr. Ahmad Abbadi  
**Prerequisites:** MOD-05 (Probability, KF), MOD-04 (Sensors), MOD-03 (Kinematics)  
**Key References:**

- **[PR]** Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Ch. 7–8.
- **[AMR]** Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press. Ch. 5.6.
- **[KF]** Welch, G. & Bishop, G. (2006). *An Introduction to the Kalman Filter*. UNC TR 95-041.

---

## A. The Localization Problem

### A.1 Problem Statement

**Localization** is the problem of estimating the robot's pose given a known map, sensor readings, and controls [PR, §7.1]:

$$p(\mathbf{x}_t \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}, m)$$

Where **x**_t = (x, y, θ) is the robot pose, **z**_{1:t} are all sensor readings up to time t, **u**_{1:t} are all control inputs, and m is the known map.

### A.2 Map and Pose

A **map** m is a pre-built representation of the environment. It can be:
- An **occupancy grid**: 2D grid where each cell is free (0), occupied (1), or unknown (0.5)
- A **feature map**: set of landmarks with known positions {(l₁_x, l₁_y), (l₂_x, l₂_y), ...}

A **pose** x = (x, y, θ) describes the robot's position and heading in the map frame.

### A.3 Three Flavors of Localization

| Flavor | Initial Belief | Algorithm |
|--------|---------------|-----------|
| **Position tracking** | Narrow Gaussian (know roughly where we are) | EKF |
| **Global localization** | Uniform (could be anywhere) | Histogram filter, MCL |
| **Kidnapped robot** | Wrong Gaussian (was tracking, then displaced) | MCL with random injection |

---

## B. Extended Kalman Filter (EKF) Localization

### B.1 Why EKF Instead of KF?

The KF from MOD-05 requires linear models: x' = Ax + Bu + w, z = Hx + v. But the differential-drive motion model from MOD-03 is **nonlinear** [PR, §7.2]:

$$x' = x + \Delta s \cos(\theta + \Delta\theta/2)$$
$$y' = y + \Delta s \sin(\theta + \Delta\theta/2)$$
$$\theta' = \theta + \Delta\theta$$

The sin(θ) and cos(θ) terms make it impossible to write this as x' = Ax + Bu. The **Extended Kalman Filter** resolves this by **linearizing** around the current estimate using the Jacobian [PR, §3.3; AMR, §5.2.5].

### B.2 The EKF Algorithm

The EKF replaces the constant matrices A and H with state-dependent Jacobians F(x̂) and H(x̂) [PR, Algorithm 7.2]:

**Prediction:**

$$\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_k)$$

$$\mathbf{P}_k^- = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}$$

Where **F** is the Jacobian of f with respect to x, evaluated at x̂_{k-1}:

$$\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k-1}} = \begin{bmatrix} 1 & 0 & -\Delta s \sin(\hat{\theta} + \Delta\theta/2) \\ 0 & 1 & \Delta s \cos(\hat{\theta} + \Delta\theta/2) \\ 0 & 0 & 1 \end{bmatrix}$$

**Update (upon observing landmark j at position (l_x, l_y) on the map):**

The observation model gives range and bearing to the landmark:

$$h(\mathbf{x}) = \begin{bmatrix} r \\ \phi \end{bmatrix} = \begin{bmatrix} \sqrt{(l_x - x)^2 + (l_y - y)^2} \\ \text{atan2}(l_y - y,\, l_x - x) - \theta \end{bmatrix}$$

The observation Jacobian is:

$$\mathbf{H} = \frac{\partial h}{\partial \mathbf{x}} = \begin{bmatrix} -\Delta x / r & -\Delta y / r & 0 \\ \Delta y / r^2 & -\Delta x / r^2 & -1 \end{bmatrix}$$

Where Δx = l_x − x̂, Δy = l_y − ŷ, r = √(Δx² + Δy²).

Then the standard KF update applies:

$$\mathbf{K} = \mathbf{P}^- \mathbf{H}^T (\mathbf{H} \mathbf{P}^- \mathbf{H}^T + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - h(\hat{\mathbf{x}}^-))$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

### B.3 Worked Example

Robot at estimated pose x̂ = (3.0, 0.0, 0°), P = diag(0.09, 0.09, 0.01). Landmark at map position (6.0, 2.0). Sensor reads range = 3.5 m, bearing = 33°. Sensor noise R = diag(0.04, 0.01).

**Step 1 — Predicted observation:**

$$r = \sqrt{(6-3)^2 + (2-0)^2} = \sqrt{13} = 3.606 \text{ m}$$
$$\phi = \text{atan2}(2, 3) - 0 = 33.69°$$

**Step 2 — Innovation:** ν = (3.5 − 3.606, 33° − 33.69°) = (−0.106, −0.69°)

**Step 3 — Compute H, K, update.** The small innovation means the prediction was close; the correction will be small. After update, P shrinks — the landmark observation reduced uncertainty.

### B.4 EKF Limitations

- Maintains a **single Gaussian** → cannot handle multi-modal beliefs
- Linearization error → can diverge if nonlinearity is severe or initial estimate is far off
- Cannot recover from wrong initialization (no mechanism to detect failure)

---

## C. Markov Localization & Histogram Filter

### C.1 Concept

Discretize the configuration space into a grid of cells. Each cell stores the probability that the robot is in that cell [PR, §8.1]:

$$\text{bel}(x_i) = p(\mathbf{x}_t = x_i \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}, m), \quad i = 1, \ldots, N_{\text{cells}}$$

All cell probabilities must sum to 1: Σ bel(x_i) = 1.

### C.2 Algorithm

**Prediction** (motion update) [PR, §8.2]:

$$\text{bel}^-(x_i) = \sum_{j} p(x_i \mid u_t, x_j) \cdot \text{bel}(x_j)$$

For each cell x_i: sum over all cells x_j the probability of transitioning from x_j to x_i times the probability of having been at x_j. This is the discrete version of the convolution integral from MOD-05.

**Update** (sensor correction):

$$\text{bel}(x_i) = \eta \cdot p(z_t \mid x_i, m) \cdot \text{bel}^-(x_i)$$

Multiply each cell's probability by the likelihood of the sensor reading given that the robot is at that cell. Normalize so probabilities sum to 1.

### C.3 Advantages and Limitations

**Advantages:** Can represent multi-modal beliefs (multiple possible positions). Handles global localization. Exact Bayes filter (no linearization).

**Limitations:** Exponential in dimensions. For 3D pose (x, y, θ) with 10 cm resolution and 1° angular bins on a 50m × 30m floor: 500 × 300 × 360 = 54 million cells. Impractical for most real robots.

---

## D. Monte Carlo Localization (MCL)

### D.1 Particle Representation

Instead of a grid, represent the belief with M **particles** (weighted samples) [PR, §8.3]:

$$\text{bel}(\mathbf{x}_t) \approx \{(\mathbf{x}_t^{[m]}, w_t^{[m]})\}_{m=1}^{M}$$

Each particle x^[m] is a hypothesis for the robot's pose, with weight w^[m] indicating how plausible it is.

### D.2 The MCL Algorithm [PR, Algorithm 8.2]

```
Algorithm: MCL(X_{t-1}, u_t, z_t, m)
  X̄_t = ∅
  for m = 1 to M:
    1. SAMPLE:  x_t^[m] ~ p(x_t | u_t, x_{t-1}^[m])     // move particle
    2. WEIGHT:  w_t^[m] = p(z_t | x_t^[m], m)            // evaluate likelihood
    3. Add (x_t^[m], w_t^[m]) to X̄_t
  
  Normalize weights: w_t^[m] ← w_t^[m] / Σ_m w_t^[m]
  
  X_t = RESAMPLE(X̄_t)    // draw M particles with replacement, ∝ weights
  
  Return X_t
```

**Step 1 — Sample (Predict):** For each particle, sample a new pose from the motion model p(x_t | u_t, x_{t-1}). In practice: apply the motion model f(x, u) plus random noise sampled from Q. Each particle gets a different noise realization, so they diverge.

**Step 2 — Weight (Evaluate):** For each particle, compute how well the sensor reading z_t matches what would be expected at that particle's position. Using the sensor model from MOD-04:

$$w^{[m]} = p(\mathbf{z}_t \mid \mathbf{x}_t^{[m]}, m)$$

For LiDAR, this uses the beam model or likelihood field model from MOD-04. Particles at positions that match the sensor reading well get high weights.

**Step 3 — Resample:** Draw M new particles from the current set with probability proportional to their weights. Particles with high weights get copied (possibly multiple times); particles with low weights are eliminated. This is **importance resampling** [PR, §4.3].

### D.3 Convergence

Initially (global localization): particles are scattered uniformly across the map. After several predict-weight-resample cycles, particles cluster around positions that consistently explain the sensor readings. Eventually, all particles converge near the true position.

### D.4 Kidnapped Robot Recovery

If all particles are in the wrong location, all weights become very small. Standard MCL cannot recover because resampling just copies the same bad particles. Solution: **random particle injection** — after each resample, replace a small fraction (1–5%) of particles with randomly sampled poses across the map [PR, §8.3.3]. If the robot was moved to a new location, the random particles there will get high weights and multiply.

### D.5 Properties

**Computational cost:** O(M × K) per step, where M = number of particles (typically 500–5000), K = number of sensor beams used for weighting.

**Completeness:** Probabilistically complete — given enough particles and time, MCL will find the correct pose.

**Non-parametric:** Can represent any distribution shape — multi-modal, skewed, or irregular. Unlike EKF (one Gaussian) or histogram (fixed grid).

---

## E. Algorithm Comparison

| Property | EKF | Histogram | MCL |
|----------|-----|-----------|-----|
| Belief type | Single Gaussian | Discrete grid | Particle set |
| Multi-modal | No | Yes | Yes |
| Global localization | No | Yes | Yes |
| Kidnapped recovery | No | Yes (slow) | Yes (fast, with random injection) |
| Computational cost | O(n²) per landmark | O(cells) — exponential in dim | O(M × K) — tunable |
| Smoothness | Very smooth | Quantized | Noisy (sampling) |
| ROS 2 package | robot_localization | — | AMCL (Nav2) |

**Best practice:** Use MCL (AMCL) for initial global localization, then EKF for smooth position tracking once converged.

---

## F. AMCL in ROS 2

**Adaptive MCL (AMCL)** extends MCL with [PR, §8.3.5]:
- **KLD-sampling:** automatically adjusts particle count — more when uncertain, fewer when confident
- **Random particle injection:** recovers from kidnapping by monitoring average weight
- **Likelihood field model:** efficient scan-to-map matching without ray-casting

Key ROS 2 topics: /scan (input LiDAR), /map (input map), /tf map→odom (output pose correction), /amcl_pose (output estimated pose).

TF tree: **map → odom → base_link**. AMCL publishes map→odom. robot_localization (or odometry) publishes odom→base_link.

---

## References

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Ch. 7–8.
2. Dellaert, F., Fox, D., Burgard, W., & Thrun, S. (1999). Monte Carlo Localization for Mobile Robots. *ICRA*.
3. Fox, D. (2003). Adapting the Sample Size in Particle Filters Through KLD-Sampling. *IJRR*, 22(12).
4. Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press. Ch. 5.6.
5. Macenski, S. et al. (2023). The Marathon 2: A Navigation System. *IROS*.
