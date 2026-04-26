# MOD-05: Probabilistic Robotics & State Estimation

## Student Script — Full Mathematical Foundations

**Course:** Mobile Robots  
**Instructor:** Dr. Ahmad Abbadi  
**Prerequisites:** MOD-03 (Kinematics), MOD-04 (Sensor Modeling), basic probability, matrix algebra  
**Key References:**

- **[PR]** Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapters 2–3, 7.
- **[AMR]** Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press. Chapter 5.
- **[KF]** Welch, G. & Bishop, G. (2006). *An Introduction to the Kalman Filter*. UNC Technical Report TR 95-041.
- **[OSE]** Simon, D. (2006). *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*. Wiley.

---

## A. Why Probabilistic Robotics?

### A.1 The Fundamental Problem

In MOD-04, we established that every sensor measurement contains noise. The consequence is profound: **the robot never knows its true state with certainty**. Consider the difference:

- **Deterministic approach:** "The robot is at position (3.2, 5.1)."
- **Probabilistic approach:** "The robot is *probably* at (3.2, 5.1) with standard deviation σ = 0.3 m."

The deterministic approach pretends we know exactly — but we don't. One bad sensor reading, and the estimate is wrong with no way to know how wrong. The probabilistic approach is honest: it represents what we actually know, including our uncertainty [PR, §2.1].

### A.2 The State Estimation Pipeline

The pipeline we build in this module:

```
Motion (encoders, IMU)  →  PREDICT  →  bel⁻(x)  →  grows uncertainty
                                ↓
Sensor (LiDAR, camera)  →  UPDATE   →  bel(x)   →  shrinks uncertainty
```

- **bel⁻(x)** = predicted belief (after motion, before sensor correction)
- **bel(x)** = updated belief (after sensor correction)

This predict-update cycle is the **heart of all state estimation in robotics** [PR, §2.4]. Everything in MOD-05 builds toward implementing this cycle mathematically.

---

## B. Probability Foundations

### B.1 Random Variables and Probability Density

A **random variable** X is a quantity whose value is uncertain. In robotics, nearly everything is a random variable: robot position, sensor readings, landmark locations [PR, §2.2].

**Discrete random variable:** Takes on countable values. Characterized by the probability mass function (PMF):

$$P(X = x_i) \geq 0, \quad \sum_i P(X = x_i) = 1$$

**Continuous random variable:** Takes on any value in an interval. Characterized by the probability density function (PDF) p(x):

$$p(x) \geq 0, \quad \int_{-\infty}^{\infty} p(x)\, dx = 1$$

The probability that X falls in interval [a, b] is:

$$P(a \leq X \leq b) = \int_a^b p(x)\, dx$$

> **Important:** p(x) is a *density*, not a probability. It can exceed 1 at a point (e.g., a very narrow, tall Gaussian). Only the *integral* (area) gives probability.

**Robot example:** The LiDAR range measurement z is a continuous random variable. Its PDF is approximately Gaussian centered on the true range r_true, with standard deviation σ_r ≈ 0.02 m.

### B.2 Mean and Variance

For a continuous random variable X with PDF p(x):

**Mean (expected value):**

$$\mu = E[X] = \int_{-\infty}^{\infty} x \cdot p(x)\, dx$$

The mean is our *best guess* of the true value — the "center of mass" of the distribution.

**Variance:**

$$\sigma^2 = \text{Var}(X) = E[(X - \mu)^2] = \int_{-\infty}^{\infty} (x - \mu)^2 \cdot p(x)\, dx$$

The variance quantifies *spread*. A small σ² means readings cluster near the mean (reliable sensor); a large σ² means readings are spread out (unreliable sensor).

**Standard deviation:** σ = √(σ²), which has the same units as X.

**Robot example:** If we take 100 LiDAR readings of a wall at 2.0 m, we might get:

- μ = 2.003 m (mean — close to truth)
- σ = 0.018 m (std dev — tight spread)
- 68% of readings fall in [1.985, 2.021] = μ ± 1σ
- 95% of readings fall in [1.967, 2.039] = μ ± 2σ

### B.3 Joint Probability

When we have two random variables X and Y (e.g., robot's x-position and y-position), their **joint probability** describes the probability of both taking specific values simultaneously [PR, §2.2]:

$$p(x, y) \geq 0, \quad \int \int p(x, y)\, dx\, dy = 1$$

**Independence:** X and Y are independent if and only if:

$$p(x, y) = p(x) \cdot p(y)$$

Knowing X tells us nothing about Y, and vice versa.

**Dependence:** If X and Y are not independent (which is the common case in robotics), then:

$$p(x, y) \neq p(x) \cdot p(y)$$

**Robot example:** The x-error and y-error of odometry are typically *dependent* because they both depend on the heading error θ. A heading error to the right causes positive x-error AND negative y-error simultaneously. This dependence is captured by the **covariance** (Section C.2).

### B.4 Conditional Probability

The **conditional probability** of A given B is [PR, §2.2]:

$$P(A | B) = \frac{P(A, B)}{P(B)}, \quad P(B) > 0$$

Read as: "the probability of A, given that we know B is true."

**Robot example:** A robot is either near a door (D) or in a hallway (H). Its light sensor reads "dark."

- P(dark | D) = 0.7 — doors are often in dim areas
- P(dark | H) = 0.2 — hallways are usually bright

The sensor reading "dark" makes "at door" *more likely*. Conditional probability captures exactly this: how evidence changes our beliefs.

### B.5 Bayes' Theorem

**Bayes' theorem** is the single most important equation in probabilistic robotics [PR, §2.3]:

$$\boxed{P(x | z) = \frac{P(z | x) \cdot P(x)}{P(z)}}$$

Each term has a specific role:

| Term | Name | Robot Meaning |
|------|------|---------------|
| P(x \| z) | **Posterior** | What we want: where is the robot, given the sensor data? |
| P(z \| x) | **Likelihood** | From MOD-04: if the robot is at x, how likely is reading z? |
| P(x) | **Prior** | What we believed *before* the measurement |
| P(z) | **Evidence** | Normalizing constant (ensures probabilities sum to 1) |

The evidence P(z) can be computed as:

$$P(z) = \int P(z | x) \cdot P(x)\, dx = \sum_x P(z | x) \cdot P(x) \quad \text{(discrete)}$$

Or we simply write P(z) = η⁻¹ (normalization constant) and normalize at the end.

#### Worked Example: Door vs. Hallway

Robot is equally likely to be near a door (D) or hallway (H). Sensor reads "dark."

**Step 1 — Prior:** P(D) = 0.5, P(H) = 0.5

**Step 2 — Likelihood:** P(dark | D) = 0.7, P(dark | H) = 0.2

**Step 3 — Unnormalized posterior:**

$$P(D | \text{dark}) \propto P(\text{dark} | D) \cdot P(D) = 0.7 \times 0.5 = 0.35$$
$$P(H | \text{dark}) \propto P(\text{dark} | H) \cdot P(H) = 0.2 \times 0.5 = 0.10$$

**Step 4 — Normalize:** Sum = 0.35 + 0.10 = 0.45

$$P(D | \text{dark}) = \frac{0.35}{0.45} = 0.778$$
$$P(H | \text{dark}) = \frac{0.10}{0.45} = 0.222$$

**Result:** After seeing "dark," belief shifted from 50%/50% → 78%/22%. The sensor didn't *prove* we're at the door — it made it *more likely*.

### B.6 Marginalization

To find the probability of one variable alone, we **integrate out** (marginalize) the other [PR, §2.2]:

$$p(x) = \int p(x, y)\, dy$$

**Total probability theorem** (using conditional probability):

$$p(x) = \int p(x | y) \cdot p(y)\, dy$$

This is needed for the **prediction step** of the Bayes filter:

$$\text{bel}^-(x') = \int p(x' | x, u) \cdot \text{bel}(x)\, dx$$

We integrate over all possible previous positions x to get the predicted new position x'. This "smears" the current belief through the motion model.

### B.7 The Gaussian Distribution

The **Gaussian (normal) distribution** is the most important distribution in robotics [PR, §2.2; AMR, §5.2].

**Univariate Gaussian:**

$$p(x) = \mathcal{N}(x; \mu, \sigma^2) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\left(-\frac{(x - \mu)^2}{2\sigma^2}\right)$$

- μ = mean (peak location)
- σ = standard deviation (bell width)
- 68% of values within μ ± 1σ
- 95% within μ ± 2σ
- 99.7% within μ ± 3σ

**Multivariate Gaussian (d-dimensional):**

$$p(\mathbf{x}) = \mathcal{N}(\mathbf{x}; \boldsymbol{\mu}, \boldsymbol{\Sigma}) = \frac{1}{(2\pi)^{d/2} |\boldsymbol{\Sigma}|^{1/2}} \exp\left(-\frac{1}{2}(\mathbf{x} - \boldsymbol{\mu})^T \boldsymbol{\Sigma}^{-1} (\mathbf{x} - \boldsymbol{\mu})\right)$$

Where:
- **μ** ∈ ℝᵈ = mean vector (peak of the distribution)
- **Σ** ∈ ℝᵈˣᵈ = covariance matrix (shape of the ellipse)
- |**Σ**| = determinant of Σ

The contours of constant probability form **ellipses** in 2D (ellipsoids in higher dimensions). These are the "uncertainty ellipses" from MOD-04.

**Why Gaussian is special for robotics [PR, §2.4]:**

1. **Closed under linear transformation:** If x ~ N(μ, Σ), then y = Ax + b ~ N(Aμ + b, AΣAᵀ).
2. **Closed under multiplication:** Product of two Gaussians is Gaussian (enables the update step).
3. **Closed under marginalization:** Marginalizing a subset of variables gives a Gaussian.
4. **Maximum entropy:** For a given mean and covariance, the Gaussian has maximum entropy — it makes the fewest assumptions about the distribution shape.
5. **Central Limit Theorem:** Sum of many independent random variables converges to Gaussian, regardless of their individual distributions.

---

## C. Linear Algebra for State Estimation

### C.1 Matrices in Robotics

State estimation involves several key matrices [AMR, §5.2; KF]:

| Matrix | Size | Role | Example |
|--------|------|------|---------|
| **A** (or **F**) | n × n | State transition: how state evolves | Position from velocity |
| **B** | n × m | Control input: how commands affect state | Steering → position |
| **H** | k × n | Observation: what sensor measures | LiDAR range from position |
| **P** (or **Σ**) | n × n | Covariance: uncertainty shape | Position uncertainty |
| **Q** | n × n | Process noise: motion uncertainty | Wheel slip noise |
| **R** | k × k | Measurement noise: sensor uncertainty | LiDAR range noise |
| **K** | n × k | Kalman gain: trust weighting | Balance prediction vs. sensor |

### C.2 The Covariance Matrix

For a 2D state x = [x, y]ᵀ, the covariance matrix is [PR, §2.2]:

$$\boldsymbol{\Sigma} = \begin{bmatrix} \sigma_x^2 & \sigma_{xy} \\ \sigma_{xy} & \sigma_y^2 \end{bmatrix}$$

- **σ²ₓ** = variance in x (how spread out in x direction)
- **σ²_y** = variance in y (how spread out in y direction)
- **σ_xy** = covariance between x and y (how they are linked)

**Interpretation of σ_xy:**
- σ_xy = 0: x and y are uncorrelated → circular uncertainty ellipse
- σ_xy > 0: when x is above average, y tends to be above average too → tilted ellipse (positive slope)
- σ_xy < 0: when x is above average, y tends to be below average → tilted ellipse (negative slope)

### C.3 Eigenvalues and Eigenvectors of Σ

The covariance matrix Σ can be decomposed as [OSE, §1.3]:

$$\boldsymbol{\Sigma} = \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T$$

Where:
- **V** = matrix of eigenvectors (columns) — the **directions** of the ellipse axes
- **Λ** = diagonal matrix of eigenvalues λ₁, λ₂ — the **lengths² ** of the ellipse axes

The uncertainty ellipse is characterized by:
- **Eigenvector v₁** → direction of maximum uncertainty (major axis)
- **Eigenvector v₂** → direction of minimum uncertainty (minor axis)
- **√λ₁** → semi-axis length in direction v₁
- **√λ₂** → semi-axis length in direction v₂

**Positive-definiteness:** A covariance matrix must have ALL eigenvalues λᵢ > 0. This means:
- The ellipse is a real shape (not collapsed to a line or point)
- The uncertainty is well-defined in all directions
- If any λᵢ ≤ 0, the filter has failed (numerical instability)

### C.4 Quadratic Forms and Mahalanobis Distance

The exponent of the multivariate Gaussian contains the **quadratic form** [PR, §2.2]:

$$d_M^2 = (\mathbf{x} - \boldsymbol{\mu})^T \boldsymbol{\Sigma}^{-1} (\mathbf{x} - \boldsymbol{\mu})$$

This is the **squared Mahalanobis distance** — the same concept from MOD-04 used for sensor health monitoring. It measures "how many standard deviations away" a point is from the center of the distribution, accounting for the shape and orientation of the uncertainty.

- d_M < 1: point is within the 1σ ellipse (likely)
- d_M ≈ 2: point is at the 2σ boundary (plausible)
- d_M > 3: point is outside the 3σ ellipse (unlikely — possible outlier)

---

## D. Probabilistic Motion Model

### D.1 From Kinematics to Probability

In MOD-03, we derived the differential-drive motion model:

$$x' = x + \Delta s \cos(\theta + \Delta\theta/2)$$
$$y' = y + \Delta s \sin(\theta + \Delta\theta/2)$$
$$\theta' = \theta + \Delta\theta$$

In MOD-04, we added Gaussian noise:

$$x' = f(x, u) + w, \quad w \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$$

In MOD-05, we rewrite this as a **probability distribution** [PR, §5.3]:

$$\boxed{p(\mathbf{x}' | \mathbf{x}, \mathbf{u}) = \mathcal{N}(\mathbf{x}'; f(\mathbf{x}, \mathbf{u}), \mathbf{Q})}$$

This reads: "The probability of ending up at state x', given that we were at state x and applied control u, is a Gaussian centered on f(x, u) with covariance Q."

The three formulations contain identical information — the probabilistic form is what Bayes' theorem can work with.

### D.2 Prediction as Belief Propagation

The prediction step of the Bayes filter uses marginalization [PR, §2.4.3]:

$$\text{bel}^-(\mathbf{x}') = \int p(\mathbf{x}' | \mathbf{x}, \mathbf{u}) \cdot \text{bel}(\mathbf{x})\, d\mathbf{x}$$

**Intuition:** For each possible current position x (weighted by how likely that position is), compute where the robot would end up after applying control u. Sum over all possibilities.

**Key property:** The predicted belief bel⁻(x') is **always wider** (more uncertain) than the previous belief bel(x). Motion adds uncertainty Q — the ellipse only grows during prediction.

For the linear-Gaussian case (Kalman Filter), this integral has a closed-form solution:

If bel(x) = N(μ, P) and x' = Ax + Bu + w with w ~ N(0, Q), then:

$$\text{bel}^-(\mathbf{x}') = \mathcal{N}(\mathbf{A}\boldsymbol{\mu} + \mathbf{B}\mathbf{u},\; \mathbf{A}\mathbf{P}\mathbf{A}^T + \mathbf{Q})$$

This gives us KF Equations 1 and 2 (Section G).

---

## E. Probabilistic Observation Model

### E.1 From Sensor Equation to Likelihood

In MOD-04, the universal sensor equation was:

$$z = h(x) + v, \quad v \sim \mathcal{N}(\mathbf{0}, \mathbf{R})$$

In probabilistic form, this becomes the **likelihood function** [PR, §6.1]:

$$\boxed{p(\mathbf{z} | \mathbf{x}) = \mathcal{N}(\mathbf{z}; h(\mathbf{x}), \mathbf{R})}$$

This reads: "The probability of observing z, given the robot is at state x, is a Gaussian centered on h(x) with covariance R."

**As a function of x (not z):** For a fixed observation z, p(z | x) evaluated at different x values tells us how well each possible position explains the observation. The peak occurs where h(x) = z — the position that would produce exactly the observed reading.

### E.2 The Update Step

Bayes' theorem gives us the update [PR, §2.4.3]:

$$\text{bel}(\mathbf{x}) = \eta \cdot p(\mathbf{z} | \mathbf{x}) \cdot \text{bel}^-(\mathbf{x})$$

**Posterior = Likelihood × Prior** (up to normalization)

**Geometric interpretation:** Multiplying two Gaussians produces a narrower Gaussian:

- The prior bel⁻(x) is wide (uncertain prediction)
- The likelihood p(z | x) is narrow (precise sensor reading)
- The posterior bel(x) sits **between** the two, closer to whichever is more certain
- The posterior is **always narrower** than the prior — every measurement reduces uncertainty

For two 1D Gaussians with means μ₁, μ₂ and variances σ₁², σ₂²:

$$\mu_{\text{fused}} = \frac{\sigma_2^2 \mu_1 + \sigma_1^2 \mu_2}{\sigma_1^2 + \sigma_2^2}$$

$$\sigma_{\text{fused}}^2 = \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2}$$

Note: σ²_fused < min(σ₁², σ₂²) — the fused result is **better than either input alone**. This is the mathematical basis for sensor fusion from MOD-04.

---

## F. The Bayes Filter

### F.1 The General Algorithm

The **recursive Bayes filter** is the general framework for all state estimation [PR, Algorithm 2.1]:

```
Algorithm: BAYES_FILTER(bel(xₜ₋₁), uₜ, zₜ)

  For all xₜ:
    1. PREDICT:  bel⁻(xₜ) = ∫ p(xₜ | uₜ, xₜ₋₁) · bel(xₜ₋₁) dxₜ₋₁
    2. UPDATE:   bel(xₜ) = η · p(zₜ | xₜ) · bel⁻(xₜ)
  
  Return bel(xₜ)
```

This algorithm repeats at every time step:
1. **Predict:** propagate belief through the motion model → uncertainty grows
2. **Update:** incorporate the sensor measurement → uncertainty shrinks

### F.2 1D Worked Example

**Setup:** Robot on a 10 m line. One landmark at position 7 m. Sensor measures distance to landmark with σ = 0.5 m. Motion noise σ_motion = 0.3 m per step.

**Initial belief:** bel(x₀) = N(μ = 5.0, σ² = 4.0) — very uncertain.

**Step 1 — Predict (move right 0.5 m):**

$$\mu^- = 5.0 + 0.5 = 5.5$$
$$(\sigma^-)^2 = 4.0 + 0.09 = 4.09 \quad (\sigma^- = 2.02)$$

Uncertainty grew slightly: σ = 2.0 → 2.02 m.

**Step 2 — Update (sensor reads d = 1.8 m to landmark at 7 m):**

The sensor says robot is at 7.0 - 1.8 = 5.2 m, with σ_sensor = 0.5 m.

Using the Gaussian fusion formulas:

$$K = \frac{(\sigma^-)^2}{(\sigma^-)^2 + \sigma_{\text{sensor}}^2} = \frac{4.09}{4.09 + 0.25} = 0.942$$

$$\mu = \mu^- + K \cdot (z_{\text{implied}} - \mu^-) = 5.5 + 0.942 \cdot (5.2 - 5.5) = 5.217$$

$$\sigma^2 = (1 - K) \cdot (\sigma^-)^2 = (1 - 0.942) \cdot 4.09 = 0.237 \quad (\sigma = 0.487)$$

**Result:** Uncertainty collapsed from σ = 2.02 → 0.49 m! One sensor reading cut uncertainty by 4×.

The factor K = 0.942 means: trust the sensor 94%, trust the prediction 6%. This makes sense because the prediction was very uncertain (σ = 2.02) while the sensor is relatively precise (σ = 0.5).

### F.3 Why the General Bayes Filter Is Intractable

The integral in the prediction step has **no closed-form solution** for arbitrary distributions [PR, §2.4.4]. Three families of approximations exist:

| Approximation | Representation | Algorithm | Module |
|---------------|---------------|-----------|--------|
| **Gaussian** | Mean + covariance | Kalman Filter (KF), EKF | MOD-05 (KF), MOD-06 (EKF) |
| **Histogram** | Grid of probabilities | Histogram filter | MOD-06 |
| **Samples** | Set of particles | Particle filter / MCL | MOD-06 |

The Gaussian assumption gives the Bayes filter a **closed-form solution**: the Kalman Filter.

---

## G. The Kalman Filter

### G.1 The Linear-Gaussian Assumption

The Kalman Filter assumes [PR, §3.2; AMR, §5.2.4]:

**State transition model (linear):**

$$\mathbf{x}_k = \mathbf{A} \mathbf{x}_{k-1} + \mathbf{B} \mathbf{u}_k + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$$

**Observation model (linear):**

$$\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R})$$

| Symbol | Dimension | Meaning |
|--------|-----------|---------|
| **x** | n × 1 | State vector (what we estimate) |
| **u** | m × 1 | Control input (what we command) |
| **z** | k × 1 | Measurement vector (what sensor reports) |
| **A** | n × n | State transition matrix |
| **B** | n × m | Control input matrix |
| **H** | k × n | Observation matrix |
| **Q** | n × n | Process noise covariance |
| **R** | k × k | Measurement noise covariance |
| **P** | n × n | State covariance (uncertainty) |

Under these assumptions, if the prior belief is Gaussian, the posterior belief is **also Gaussian** [PR, Theorem 3.1]. This is the key insight that makes the KF tractable.

### G.2 The Five KF Equations

The complete Kalman Filter algorithm [PR, Algorithm 3.1; AMR, Table 5.2]:

---

**PREDICTION STEP** (Motion update — uncertainty grows):

**Equation 1 — State Prediction:**

$$\boxed{\hat{\mathbf{x}}_k^- = \mathbf{A} \hat{\mathbf{x}}_{k-1} + \mathbf{B} \mathbf{u}_k}$$

*"Where do I think I am, based on my motion model?"*

- Propagate the previous estimate through the state transition model
- A rotates/stretches the state, B applies the control
- No noise term here — we predict the **expected** state

**Equation 2 — Covariance Prediction:**

$$\boxed{\mathbf{P}_k^- = \mathbf{A} \mathbf{P}_{k-1} \mathbf{A}^T + \mathbf{Q}}$$

*"How uncertain am I after moving?"*

- A·P·Aᵀ = stretch the old uncertainty through the motion model (same as MOD-04's F·P·Fᵀ for the linear case)
- +Q = add new uncertainty from process noise
- P⁻ is ALWAYS larger than P_{k-1} — prediction always increases uncertainty

---

**UPDATE STEP** (Measurement correction — uncertainty shrinks):

**Equation 3 — Kalman Gain:**

$$\boxed{\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}^T (\mathbf{H} \mathbf{P}_k^- \mathbf{H}^T + \mathbf{R})^{-1}}$$

*"How much should I trust the measurement vs. my prediction?"*

This is the most important equation. The term S = H·P⁻·Hᵀ + R is the **innovation covariance** — the combined uncertainty of prediction (mapped to measurement space) and sensor noise.

**Interpretation of K:**
- If R is small (sensor precise) → K is large → trust sensor, correct a lot
- If R is large (sensor noisy) → K is small → trust prediction, correct little
- If P⁻ is small (prediction confident) → K is small → trust prediction
- If P⁻ is large (prediction uncertain) → K is large → trust sensor
- K acts as a **slider** between full trust in prediction (K = 0) and full trust in sensor (K = I)

**Equation 4 — State Update:**

$$\boxed{\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_k^-)}$$

*"Correct my position based on the surprise."*

- (z_k − H·x̂⁻_k) = **innovation** (or residual) = actual measurement − predicted measurement
- K × innovation = weighted correction
- The corrected estimate is a blend of prediction and measurement

**Equation 5 — Covariance Update:**

$$\boxed{\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_k^-}$$

*"How uncertain am I after the correction?"*

- (I − KH) shrinks the predicted covariance
- P_k is ALWAYS smaller than or equal to P⁻_k — every measurement reduces uncertainty
- If K is large → P shrinks a lot (sensor provided good information)
- If K is small → P barely changes (sensor didn't help much)

> **Note:** The numerically more stable **Joseph form** is: P = (I − KH) P⁻ (I − KH)ᵀ + K R Kᵀ. This avoids potential loss of positive-definiteness due to floating-point errors [OSE, §5.3].

### G.3 Complete Algorithm

```
Algorithm: KALMAN_FILTER(x̂ₖ₋₁, Pₖ₋₁, uₖ, zₖ)

  // PREDICT
  x̂⁻ₖ = A · x̂ₖ₋₁ + B · uₖ                    // Eq. 1: state prediction
  P⁻ₖ  = A · Pₖ₋₁ · Aᵀ + Q                     // Eq. 2: covariance prediction

  // UPDATE
  Kₖ   = P⁻ₖ · Hᵀ · (H · P⁻ₖ · Hᵀ + R)⁻¹     // Eq. 3: Kalman gain
  x̂ₖ   = x̂⁻ₖ + Kₖ · (zₖ − H · x̂⁻ₖ)            // Eq. 4: state update
  Pₖ   = (I − Kₖ · H) · P⁻ₖ                     // Eq. 5: covariance update

  Return (x̂ₖ, Pₖ)
```

### G.4 Worked Example: 1D Robot with Position Sensor

**Setup:**

- State: x = [position] (1D)
- Motion: x_k = x_{k-1} + u_k + w_k, so A = 1, B = 1
- Sensor: z_k = x_k + v_k, so H = 1
- Q = 0.1 (motion noise variance)
- R = 0.04 (sensor noise variance, σ_sensor = 0.2 m)
- Initial: x̂₀ = 1.0, P₀ = 0.25

**Cycle 1 — Move 2m, see landmark at 3.0m (reading: z = 3.15):**

```
PREDICT:
  x̂⁻ = 1·1.0 + 1·2.0 = 3.0
  P⁻ = 1·0.25·1 + 0.1 = 0.35

UPDATE:
  K = 0.35·1·(1·0.35·1 + 0.04)⁻¹ = 0.35/0.39 = 0.897
  x̂ = 3.0 + 0.897·(3.15 − 1·3.0) = 3.0 + 0.897·0.15 = 3.135
  P = (1 − 0.897·1)·0.35 = 0.103·0.35 = 0.036

Result: σ dropped from √0.35 = 0.59 → √0.036 = 0.19 m
```

**Cycle 2 — Move 2m, reading: z = 5.08:**

```
PREDICT:
  x̂⁻ = 3.135 + 2.0 = 5.135
  P⁻ = 0.036 + 0.1 = 0.136

UPDATE:
  K = 0.136/(0.136 + 0.04) = 0.773
  x̂ = 5.135 + 0.773·(5.08 − 5.135) = 5.135 − 0.042 = 5.093
  P = (1 − 0.773)·0.136 = 0.031

Result: σ = √0.031 = 0.176 m — even better!
```

**Pattern observed:**
- K decreases over time (0.897 → 0.773) because P is getting smaller → prediction is becoming more trusted
- σ after update gets smaller each cycle: 0.19 → 0.176 → converging
- The system reaches a **steady-state** where the predict increase (Q) and update decrease balance out

### G.5 Tuning Q and R

The performance of the KF depends critically on how well Q and R match the actual noise [KF; OSE, §5.6]:

| Scenario | Q too large | Q just right | R too large | R too small |
|----------|-------------|-------------|-------------|-------------|
| Filter behavior | Trusts sensors too much | Balanced | Trusts prediction too much | Trusts sensors too much |
| Estimate | Jerky, jumps with each reading | Smooth, accurate | Drifts (follows odometry) | Noisy (follows raw sensor) |
| P steady-state | Large (never confident) | Moderate | Small (overconfident) | Small (overconfident) |
| Practical risk | Noise amplification | — | Misses real changes | Follows sensor outliers |

**How to get Q and R in practice:**
- **R:** From sensor datasheets and MOD-04 characterization (LiDAR σ_r, IMU Allan Variance, GPS accuracy)
- **Q:** From motion experiments — drive the robot straight, compare odometry to ground truth, compute variance of the error

### G.6 Assumptions and Limitations

The KF is **optimal** (minimum mean-squared error) under three conditions [PR, §3.2]:

1. **Linearity:** State transition and observation models are linear (x' = Ax + Bu + w, z = Hx + v)
2. **Gaussian noise:** w and v are Gaussian distributed
3. **Known models:** A, B, H, Q, R are known and correct

When these assumptions are violated:

| Violation | Consequence | Solution |
|-----------|-------------|----------|
| Nonlinear model (sin/cos) | Linearization error, can diverge | **EKF** (MOD-06): linearize with Jacobians |
| Non-Gaussian noise (outliers) | Outliers corrupt estimate | Robust KF, reject outliers |
| Multi-modal belief ("room A or B?") | Single Gaussian can't represent | **Particle filter / MCL** (MOD-06) |
| Unknown models | Filter is miscalibrated | Adaptive KF, system identification |

---

## H. The Localization Problem

### H.1 What Is Localization?

**Localization:** Estimate the robot's pose x = (x, y, θ) given a map m, sensor readings z₁:ₜ, and controls u₁:ₜ [PR, §7.1; AMR, §5.6]:

$$p(\mathbf{x}_t | \mathbf{z}_{1:t}, \mathbf{u}_{1:t}, m)$$

This is the specific application of the Bayes filter to the problem of "where am I?"

### H.2 Three Flavors

| Flavor | Description | Initial belief | KF Handles? |
|--------|-------------|---------------|-------------|
| **Position tracking** | Know approximate start, track over time | Narrow Gaussian | ✅ Yes |
| **Global localization** | No idea where robot is | Uniform / multi-modal | ❌ No |
| **Kidnapped robot** | Was tracking, then moved unexpectedly | Wrong Gaussian | ❌ No |

**Position tracking** is the simplest: initial uncertainty is small, the KF (or EKF) maintains a single Gaussian belief that evolves over time. This is what `robot_localization` in ROS 2 does.

**Global localization** requires representing "I might be anywhere" — a uniform distribution over the entire map. A single Gaussian cannot represent this. Solutions: histogram filter (discrete grid of probabilities) or MCL (cloud of particles).

**Kidnapped robot** is the hardest: the filter was confident in a WRONG position. It must detect the failure and re-localize. MCL with random particle injection handles this.

### H.3 Bridge to MOD-06

MOD-05 provides the **mathematical tools**:
- Probability theory (Bayes, conditional, marginalization)
- Linear algebra (covariance, eigenvalues)
- Motion model as p(x'|x,u) and sensor model as p(z|x)
- Bayes filter (general predict-update algorithm)
- Kalman Filter (exact solution for linear-Gaussian)
- Localization problem definition

MOD-06 provides the **specific algorithms** that solve localization:
- **EKF:** Extends KF to nonlinear models via Jacobian linearization (handles position tracking)
- **Histogram Filter:** Discretizes space into grid, exact Bayes filter on grid (handles global localization)
- **MCL (Particle Filter):** Represents belief with particles, weighted sampling (handles all three flavors)
- **AMCL:** Adaptive MCL, the standard localization in ROS 2 Nav2

---

## Summary of Key Results

| Concept | Key Equation | Reference |
|---------|-------------|-----------|
| Bayes' theorem | P(x\|z) = P(z\|x)·P(x)/P(z) | [PR] §2.3 |
| Gaussian PDF | N(x; μ, σ²) = (2πσ²)^(-1/2) exp(-(x-μ)²/(2σ²)) | [PR] §2.2 |
| Bayes filter predict | bel⁻(x') = ∫ p(x'\|x,u) · bel(x) dx | [PR] §2.4 |
| Bayes filter update | bel(x) = η · p(z\|x) · bel⁻(x) | [PR] §2.4 |
| KF Eq. 1 — State predict | x̂⁻ = A·x̂ + B·u | [PR] §3.2 |
| KF Eq. 2 — Cov predict | P⁻ = A·P·Aᵀ + Q | [PR] §3.2 |
| KF Eq. 3 — Kalman gain | K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹ | [PR] §3.2 |
| KF Eq. 4 — State update | x̂ = x̂⁻ + K·(z − H·x̂⁻) | [PR] §3.2 |
| KF Eq. 5 — Cov update | P = (I − K·H)·P⁻ | [PR] §3.2 |
| Gaussian fusion | σ²_fused = σ₁²·σ₂²/(σ₁²+σ₂²) | [AMR] §5.2 |
| Mahalanobis distance | d²_M = (x−μ)ᵀ Σ⁻¹ (x−μ) | [PR] §2.2 |
| Motion model | p(x'\|x,u) = N(x'; f(x,u), Q) | [PR] §5.3 |
| Observation model | p(z\|x) = N(z; h(x), R) | [PR] §6.1 |

---

## References

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
2. Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press.
3. Simon, D. (2006). *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*. Wiley.
4. Welch, G. & Bishop, G. (2006). *An Introduction to the Kalman Filter*. UNC Technical Report TR 95-041.
5. Faragher, R. (2012). *Understanding the Basis of the Kalman Filter Via a Simple and Intuitive Derivation*. IEEE Signal Processing Magazine, 29(5), 128–132.
6. Bar-Shalom, Y., Li, X.R., & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*. Wiley.
7. Kalman, R.E. (1960). *A New Approach to Linear Filtering and Prediction Problems*. Journal of Basic Engineering, 82(1), 35–45.

---

*Next module: MOD-06 — Mobile Robot Localization (EKF, Particle Filter, MCL, AMCL in ROS 2)*
