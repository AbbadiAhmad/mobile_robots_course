# MOD-004: Sensor Modeling, Measurement & Fusion
## Student Teaching Script — Full Equation Deck

**Design Spine Robot:** MediBot — Hospital Delivery Robot  
**Final Integration Challenge:** Last-Mile Sidewalk Delivery Robot  
**Teaching Method:** Problem-Based Learning (PBL), Failure-Driven Narrative  

---

# DESIGN SPINE — The MediBot Challenge

## The Mission

MediBot is an autonomous mobile robot deployed at **City General Hospital**. Its daily mission:

- **Pick up** medications from the Pharmacy (Ground Floor)
- **Navigate** hallways, pass through doors, ride the elevator
- **Deliver** to Ward B — Cardiology (Floor 3)
- **Return** with blood samples from the Lab
- **Safety constraint:** ZERO collisions — patients, nurses, wheelchairs, IV stands

## The Hospital Environment

The hospital is structured but full of challenges:

- **Polished floors** — smooth, sometimes wet → wheel slip
- **Glass doors and walls** — lobby entrance, ICU partitions
- **Dynamic obstacles** — people walk unpredictably, carts appear suddenly
- **Lighting variation** — bright lobby, dim corridors, flickering fluorescent lights
- **Elevators** — vibration, tilt, temporary loss of external references
- **Narrow doorways** — tight clearance, must be precise

## The Core Question

> MediBot carries sensors to "see" and "feel" the world. But every sensor **lies** in a different way. How do we **model** what sensors report, **quantify** their uncertainty, and **combine** them to get a reliable estimate?

We will equip MediBot with **6 sensors**, one at a time. Each time, we discover what the new sensor can do — and where it **fails**. Each failure motivates the next sensor.

---
---

# BLOCK 1: Wheel Encoders & Odometry

---

## 1.1 — Scenario: MediBot Leaves the Pharmacy

MediBot begins its first delivery. It rolls out of the pharmacy toward the elevator lobby — a straight 80-meter corridor.

**The only sensor available:** two wheel encoders (one per wheel), counting rotation ticks.

**Question for students:**  
> *If we know how many times each wheel has turned, can we figure out where MediBot is?*

---

## 1.2 — What is a Wheel Encoder?

- A **rotary encoder** is a device attached to each wheel's axle
- It produces a fixed number of **ticks** (pulses) per revolution
- By counting ticks over a time interval, we know how much each wheel has rotated

**Key parameters:**

| Parameter | Symbol | Typical Value |
|-----------|--------|---------------|
| Ticks per revolution | N | 500–4096 ticks/rev |
| Wheel radius | R | 0.05–0.15 m |
| Wheelbase (distance between wheels) | L | 0.3–0.6 m |
| Update rate | — | 1–10 kHz |

---

## 1.3 — Sensor Model: From Ticks to Motion

**Step 1: Ticks → distance traveled by each wheel**

For the left and right wheels:

$$\Delta s_L = \frac{2\pi R}{N} \cdot \Delta n_L \qquad \Delta s_R = \frac{2\pi R}{N} \cdot \Delta n_R$$

Where:
- $\Delta n_L, \Delta n_R$ = number of ticks counted in one time step
- $R$ = wheel radius
- $N$ = ticks per revolution

**Step 2: Wheel distances → robot motion (differential drive)**

$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2} \qquad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{L}$$

Where:
- $\Delta s$ = distance traveled by the robot center
- $\Delta \theta$ = change in heading (yaw angle)
- $L$ = wheelbase

**Step 3: Update the robot's pose $(x, y, \theta)$**

$$x_{k+1} = x_k + \Delta s \cdot \cos\!\left(\theta_k + \frac{\Delta\theta}{2}\right)$$

$$y_{k+1} = y_k + \Delta s \cdot \sin\!\left(\theta_k + \frac{\Delta\theta}{2}\right)$$

$$\theta_{k+1} = \theta_k + \Delta\theta$$

> **Note:** We use $\theta_k + \Delta\theta/2$ (mid-point angle) for better accuracy than using $\theta_k$ alone. This is the **mid-point Euler integration** method.

---

## 1.4 — Measurement Theory: Gaussian Noise

### Why noise?

In a perfect world, encoder ticks are exact. In reality:
- Wheel radius $R$ is not perfectly known (manufacturing tolerance)
- Ticks can be miscounted (electrical noise, vibration)
- Wheel contact is imperfect (micro-slip on every step)

### Modeling noise as Gaussian

We model each tick measurement as:

$$\Delta n = \Delta n_{true} + \epsilon, \qquad \epsilon \sim \mathcal{N}(0, \sigma^2_{tick})$$

**Why Gaussian?**
- **Central Limit Theorem:** many small, independent error sources add up → the total error is approximately Gaussian
- A Gaussian is fully described by two numbers: **mean** $\mu$ and **variance** $\sigma^2$
- Notation: $x \sim \mathcal{N}(\mu, \sigma^2)$ means "$x$ is a random variable with mean $\mu$ and variance $\sigma^2$"

### The Gaussian probability density function (PDF):

$$p(x) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\!\left(-\frac{(x - \mu)^2}{2\sigma^2}\right)$$

- **68% of values** fall within $\mu \pm \sigma$
- **95% of values** fall within $\mu \pm 2\sigma$
- **99.7% of values** fall within $\mu \pm 3\sigma$

---

## 1.5 — Error Propagation via Jacobians

### The problem

We know the noise on the ticks ($\sigma^2_{tick}$). But we care about the noise on the **pose** $(x, y, \theta)$. How does tick noise propagate through the nonlinear odometry equations?

### General rule (first-order approximation)

If we have a nonlinear function $\mathbf{y} = f(\mathbf{x})$ and the input has uncertainty:

$$\mathbf{x} \sim \mathcal{N}(\boldsymbol{\mu}_x, \mathbf{P}_x)$$

Then the output uncertainty is approximately:

$$\mathbf{P}_y \approx \mathbf{J} \, \mathbf{P}_x \, \mathbf{J}^\top$$

Where $\mathbf{J}$ is the **Jacobian matrix** of $f$:

$$J_{ij} = \frac{\partial f_i}{\partial x_j}$$

### Applied to odometry

**State vector:** $\mathbf{x}_k = [x_k, \, y_k, \, \theta_k]^\top$

**Motion model:** $\mathbf{x}_{k+1} = f(\mathbf{x}_k, \, \Delta s, \, \Delta\theta)$

**Jacobian with respect to the state** (how current pose uncertainty propagates):

$$\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}_k} = \begin{bmatrix} 1 & 0 & -\Delta s \cdot \sin(\theta_k + \Delta\theta/2) \\ 0 & 1 & \Delta s \cdot \cos(\theta_k + \Delta\theta/2) \\ 0 & 0 & 1 \end{bmatrix}$$

**Jacobian with respect to the control inputs** $(\Delta s, \Delta\theta)$:

$$\mathbf{G} = \frac{\partial f}{\partial (\Delta s, \Delta\theta)} = \begin{bmatrix} \cos(\theta_k + \Delta\theta/2) & -\frac{\Delta s}{2}\sin(\theta_k + \Delta\theta/2) \\ \sin(\theta_k + \Delta\theta/2) & \frac{\Delta s}{2}\cos(\theta_k + \Delta\theta/2) \\ 0 & 1 \end{bmatrix}$$

### Covariance propagation

The pose covariance matrix updates as:

$$\mathbf{P}_{k+1} = \mathbf{F} \, \mathbf{P}_k \, \mathbf{F}^\top + \mathbf{G} \, \mathbf{Q} \, \mathbf{G}^\top$$

Where:
- $\mathbf{P}_k$ = current pose covariance (3×3 matrix)
- $\mathbf{Q}$ = process noise covariance on $(\Delta s, \Delta\theta)$
- $\mathbf{F} \, \mathbf{P}_k \, \mathbf{F}^\top$ = propagation of previous uncertainty
- $\mathbf{G} \, \mathbf{Q} \, \mathbf{G}^\top$ = new uncertainty added by this motion step

### What is the covariance matrix?

$\mathbf{P}$ is a **3×3 symmetric positive-definite matrix**:

$$\mathbf{P} = \begin{bmatrix} \sigma^2_x & \sigma_{xy} & \sigma_{x\theta} \\ \sigma_{xy} & \sigma^2_y & \sigma_{y\theta} \\ \sigma_{x\theta} & \sigma_{y\theta} & \sigma^2_\theta \end{bmatrix}$$

- **Diagonal entries** ($\sigma^2_x, \sigma^2_y, \sigma^2_\theta$): individual variances
- **Off-diagonal entries** ($\sigma_{xy}$, etc.): correlations between variables
- **Geometrically:** the 2D position part forms an **uncertainty ellipse** — the region where MediBot "probably" is

---

## 1.6 — Worked Example: MediBot in the Corridor

**Setup:**
- MediBot starts at $(0, 0, 0°)$ — pharmacy door, facing down the corridor
- Wheel radius $R = 0.075$ m, wheelbase $L = 0.4$ m
- Encoder resolution: $N = 1000$ ticks/rev
- Motion noise: $\sigma_{\Delta s} = 0.01$ m per meter traveled, $\sigma_{\Delta\theta} = 0.5°$ per radian turned

**After 10 straight-line steps of 1 meter each:**

Each step adds noise:
- $\mathbf{Q}_{step} = \text{diag}(\sigma^2_{\Delta s}, \sigma^2_{\Delta\theta}) = \text{diag}(0.01^2, 0.0087^2)$

Since the robot drives straight ($\Delta\theta \approx 0$):

$$\mathbf{F}_{straight} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad \mathbf{G}_{straight} = \begin{bmatrix} 1 & 0 \\ 0 & 0 \\ 0 & 1 \end{bmatrix}$$

After $n$ steps of straight driving:
- $\sigma^2_x$ grows **linearly** with $n$: $\sigma^2_x(n) = n \cdot \sigma^2_{\Delta s}$
- $\sigma^2_\theta$ grows **linearly** with $n$: $\sigma^2_\theta(n) = n \cdot \sigma^2_{\Delta\theta}$

But after turns, **heading error** $\sigma_\theta$ causes the $x$ and $y$ errors to **explode** because future straight-line driving projects error sideways:

$$\sigma_y(n) \propto \sigma_\theta \cdot \sum_{k=1}^{n} \Delta s_k$$

> **Key insight:** Odometry covariance **grows without bound**. There is no mechanism to SHRINK it — because encoders only measure relative motion, never absolute position.

After 80m of travel with several turns, typical odometry error: **2–5 meters**.

---

## 1.7 — Operate Perspective: When to Use / Not Use

| Aspect | Assessment |
|--------|-----------|
| **Use for** | Short-term dead reckoning, velocity estimation, motion model in Kalman filter prediction step |
| **Do NOT use for** | Sole localization over distances > 10m, any safety-critical positioning |
| **Works well when** | Floor is dry and textured (carpet, rubber), robot moves slowly, short distances |
| **Fails when** | Smooth/wet floors (slip), bumps (wheel bounces), tight turns (differential errors amplified), long distances |
| **Hospital-specific** | Polished hospital floors cause significant wheel slip; waxed surfaces reduce traction |

---

## 1.8 — 💥 FAILURE → Transition to Block 2

> MediBot travels 80 meters from the pharmacy toward the elevator lobby. According to its odometry, it has arrived at the elevator — but it is actually **3 meters past the elevator**, facing the wrong wall. The polished hospital floor caused micro-slip on every turn, and heading error accumulated into a large lateral offset.
>
> **Odometry alone cannot fix itself.** The covariance only grows — it never shrinks. We need a sensor that measures something EXTERNAL to the robot — something in the environment that gives MediBot a reference point.

**Question for students:**  
> *What is the simplest, cheapest sensor we could add to detect nearby walls and obstacles?*

---
---

# BLOCK 2: Ultrasonic Sensors (Time-of-Flight)

---

## 2.1 — Scenario: MediBot Needs Wall References

MediBot is lost in the corridor. We add **ultrasonic sensors** — small, cheap transducers mounted around the robot body — to detect nearby walls, doors, and obstacles.

As MediBot approaches the elevator, the ultrasonics ping: *"wall at 0.6 m left, wall at 0.8 m right."* Now it can sense the corridor structure and correct its path.

**Question for students:**  
> *How does an ultrasonic sensor measure distance? What could go wrong?*

---

## 2.2 — Sensor Model: Time-of-Flight Principle

The ultrasonic transducer emits a short **sound pulse** and waits for the **echo** to return.

$$d = \frac{c \cdot t_{round-trip}}{2}$$

Where:
- $d$ = distance to the reflecting surface
- $c$ = speed of sound in air
- $t_{round-trip}$ = time from emission to echo reception
- Division by 2 because sound travels **to** the object and **back**

### Speed of sound depends on temperature

$$c(T) = 331.3 + 0.606 \cdot T \quad \text{(m/s)}$$

Where $T$ is air temperature in °C.

| Temperature | Speed of Sound |
|-------------|---------------|
| 15°C (cold storage) | 340.4 m/s |
| 22°C (hallway) | 344.6 m/s |
| 37°C (near sterilizer) | 353.7 m/s |

> **Implication:** If MediBot uses $c = 343$ m/s everywhere, but enters a cold storage room at 15°C, range measurements will have a **systematic bias** of about **0.9%** — that's 9 mm per meter.

---

## 2.3 — Beam Cone Model

Unlike a laser (narrow beam), an ultrasonic transducer emits a **cone of sound** with a half-angle of typically **15°–30°**.

**Consequences:**
- The sensor returns the range to the **nearest object anywhere in the cone**
- You do NOT know exactly **where** in the cone the reflection came from
- **Angular resolution is poor** — two objects close together cannot be distinguished

### Mathematical model:

The measurement returns a single range $z$ that corresponds to:

$$z = \min_{(\alpha, \beta) \in \text{cone}} \, d(\alpha, \beta)$$

Where $\alpha, \beta$ are angles within the cone. This is NOT a point measurement — it's the minimum range over a wide angular region.

---

## 2.4 — Measurement Theory: Systematic vs. Random Errors

### Formal measurement model

Any sensor measurement can be decomposed as:

$$z = h(\mathbf{x}) + b + v$$

Where:
- $z$ = the measured value (what the sensor reports)
- $h(\mathbf{x})$ = the **observation function** — what the sensor SHOULD measure given the true state $\mathbf{x}$
- $b$ = **systematic error (bias)** — repeatable, predictable offset
- $v$ = **random noise** — stochastic, unpredictable, $v \sim \mathcal{N}(0, R)$

### Systematic errors (bias) in ultrasonic sensors

| Source | Effect | Mitigation |
|--------|--------|-----------|
| Temperature variation | Speed of sound changes → range bias | Temperature compensation (measure T, correct c) |
| Transducer aging | Sensitivity drift over time | Periodic recalibration |
| Mounting offset | Sensor not at assumed position on robot | Measure and include extrinsic offset |

### Random errors in ultrasonic sensors

| Source | Effect | Typical magnitude |
|--------|--------|------------------|
| Timing jitter | Uncertainty in round-trip time | σ ≈ 1–5 mm |
| Surface roughness | Multiple micro-reflections | σ ≈ 5–20 mm |
| Air turbulence | Sound path variation | Variable, worse near HVAC vents |

### Sensor noise covariance $R$

For a single ultrasonic range measurement:

$$R = \sigma^2_z$$

This is often **specified in the datasheet** (e.g., ±2 mm at 1m range), or **measured experimentally** by taking many readings at a known distance and computing the sample variance:

$$\hat{\sigma}^2_z = \frac{1}{N-1}\sum_{i=1}^{N}(z_i - \bar{z})^2$$

### Resolution vs. Accuracy

These are **different concepts**:

- **Resolution:** the smallest change in distance the sensor can detect (determined by timing resolution and frequency)
- **Accuracy:** how close the average measurement is to the true value (affected by bias + noise)

> A sensor can have fine resolution (detect 1 mm changes) but poor accuracy (average reading is 5 mm off from truth).

---

## 2.5 — Calibration: Estimating and Removing Bias

**Calibration** = systematic process to estimate bias $b$ and correct for it.

**Procedure:**
1. Place a flat target at several **known distances** $d_1, d_2, \ldots, d_M$
2. Take multiple readings at each distance
3. Compute average reading $\bar{z}_i$ at each known distance $d_i$
4. Fit a correction model:

$$d_{corrected} = a \cdot z_{raw} + b$$

Where $a$ (scale factor) and $b$ (offset) are estimated by **least-squares regression**:

$$\min_{a, b} \sum_{i=1}^{M} (d_i - a\bar{z}_i - b)^2$$

The solution:

$$a = \frac{\sum(d_i - \bar{d})(\bar{z}_i - \bar{\bar{z}})}{\sum(\bar{z}_i - \bar{\bar{z}})^2}, \qquad b = \bar{d} - a\bar{\bar{z}}$$

---

## 2.6 — Worked Example: MediBot Corridor Wall Detection

**Setup:**
- MediBot is in a corridor, width = 2.0 m
- Left ultrasonic sensor mounted at center of robot, pointing left
- True distance to left wall: 0.8 m
- Temperature: 22°C → $c = 344.6$ m/s
- MediBot firmware assumes: $c = 343.0$ m/s (not temperature-compensated)

**Step 1: True round-trip time**

$$t = \frac{2d}{c_{true}} = \frac{2 \times 0.8}{344.6} = 4.643 \text{ ms}$$

**Step 2: MediBot computes distance using wrong $c$**

$$z = \frac{c_{assumed} \cdot t}{2} = \frac{343.0 \times 4.643 \times 10^{-3}}{2} = 0.7963 \text{ m}$$

**Bias:** $b = z - d_{true} = 0.7963 - 0.800 = -3.7$ mm

- This is a **systematic underestimate** of 3.7 mm at 0.8 m range
- At 4 m range, the bias would be ~18.5 mm — significant for navigation!
- **Fix:** add a temperature sensor and correct $c$ in real-time

---

## 2.7 — Operate Perspective: When to Use / Not Use

| Aspect | Assessment |
|--------|-----------|
| **Use for** | Close-range obstacle detection (< 4m), wall following, elevator/door proximity, **detecting glass doors and walls** |
| **Do NOT use for** | Precision localization, large open spaces (lobby), detecting soft/sound-absorbing objects (cotton, foam) |
| **Works well when** | Target is perpendicular to sensor (±3°), solid flat surface, short range |
| **Fails when** | Target at oblique angle (> 5°) → specular reflection deflects echo away; very thin objects; sound-absorbing materials (curtains, foam padding); cross-talk between multiple sensors firing simultaneously |
| **Key advantage over LiDAR** | Ultrasonic **detects glass reliably** — sound reflects off glass due to large acoustic impedance mismatch between air and glass. LiDAR laser passes through glass. |
| **Hospital-specific** | Excellent for detecting glass ICU partitions that LiDAR misses; but affected by HVAC air currents and temperature variation between rooms |

---

## 2.8 — 💥 FAILURE → Transition to Block 3

> MediBot enters the **main lobby** — a large, open space (15m × 20m). The ultrasonic sensors max out at 4 meters and return "no reading" for distant walls. Even nearby surfaces are problematic: the polished marble columns reflect sound at oblique angles, causing **specular reflection** — the echo bounces away instead of returning to the sensor.
>
> Worse, multiple ultrasonic sensors firing simultaneously create **cross-talk** — sensor A picks up the echo from sensor B's pulse, generating **phantom obstacles** that don't exist.
>
> MediBot needs a sensor with **much longer range**, **much finer angular resolution**, and **hundreds of measurement points per scan** instead of just one range per sensor.

**Question for students:**  
> *What sensor gives us hundreds of precise range measurements at once, across a full 360° sweep?*

---
---

# BLOCK 3: LiDAR (2D and 3D)

---

## 3.1 — Scenario: MediBot in the Lobby

MediBot is stuck in the open lobby. Ultrasonics can't cover the space. We mount a **2D LiDAR** on top — a spinning laser scanner that sweeps 360° and returns **hundreds of range-bearing points per revolution**.

Instantly, MediBot gets a rich picture: walls, pillars, reception desk, people walking — all visible as a crisp **point cloud**.

**Question for students:**  
> *A LiDAR beam is just a single laser pulse — what makes it so much better than ultrasonic at mapping the environment?*

---

## 3.2 — Sensor Model: Range-Bearing Measurement

Each LiDAR beam returns a **range-bearing pair** $(r_i, \phi_i)$:

- $r_i$ = distance to the reflecting surface along beam $i$
- $\phi_i$ = angle of beam $i$ relative to the sensor's forward axis

**Conversion to Cartesian coordinates (sensor frame):**

$$x_i^{sensor} = r_i \cos(\phi_i), \qquad y_i^{sensor} = r_i \sin(\phi_i)$$

**Conversion to world frame** (given robot pose $\mathbf{x} = [x_r, y_r, \theta_r]$):

$$\begin{bmatrix} x_i^{world} \\ y_i^{world} \end{bmatrix} = \begin{bmatrix} x_r \\ y_r \end{bmatrix} + \mathbf{R}(\theta_r) \begin{bmatrix} x_i^{sensor} \\ y_i^{sensor} \end{bmatrix}$$

Where the rotation matrix is:

$$\mathbf{R}(\theta_r) = \begin{bmatrix} \cos\theta_r & -\sin\theta_r \\ \sin\theta_r & \cos\theta_r \end{bmatrix}$$

---

## 3.3 — Beam Model: Why LiDAR Readings Are Not Always Clean

A perfect LiDAR would always return the exact range to the nearest surface. In reality, LiDAR readings follow a **mixture model** with 4 components:

$$p(z_i \mid x, m) = \alpha_{hit}\, p_{hit} + \alpha_{short}\, p_{short} + \alpha_{max}\, p_{max} + \alpha_{rand}\, p_{rand}$$

Where $\alpha_{hit} + \alpha_{short} + \alpha_{max} + \alpha_{rand} = 1$.

### Component 1: $p_{hit}$ — Normal measurement

The beam hits the expected surface and returns a noisy range:

$$p_{hit}(z_i) = \frac{1}{\sqrt{2\pi\sigma_r^2}} \exp\!\left(-\frac{(z_i - r_{true})^2}{2\sigma_r^2}\right) \quad \text{for } 0 \leq z_i \leq z_{max}$$

This is a Gaussian centered on the true range, with standard deviation $\sigma_r$ (typically 1–3 cm for indoor LiDAR).

### Component 2: $p_{short}$ — Unexpected close object

Something between the sensor and the expected surface (a person's leg, a cart wheel):

$$p_{short}(z_i) = \lambda \, e^{-\lambda z_i} \quad \text{for } 0 \leq z_i \leq r_{true}$$

This is an **exponential distribution** — biased toward short readings.

### Component 3: $p_{max}$ — Max range return

The beam missed everything (glass door, black surface that absorbs light, beam aimed at open doorway):

$$p_{max}(z_i) = \begin{cases} 1 & \text{if } z_i = z_{max} \\ 0 & \text{otherwise} \end{cases}$$

This is a **point mass** at the maximum range.

### Component 4: $p_{rand}$ — Random noise

Electronic noise, stray reflections from dust particles:

$$p_{rand}(z_i) = \frac{1}{z_{max}} \quad \text{for } 0 \leq z_i \leq z_{max}$$

This is a **uniform distribution** over the entire range.

### Typical mixing weights (indoor environment):

$$\alpha_{hit} = 0.85, \quad \alpha_{short} = 0.05, \quad \alpha_{max} = 0.05, \quad \alpha_{rand} = 0.05$$

---

## 3.4 — Likelihood Field Model (Practical Alternative)

The beam model above is physically accurate but **computationally expensive** (requires ray-casting through the map). In practice, many systems use the **likelihood field model**:

**Idea:** Precompute a "distance map" — at every point in the map, store the distance to the nearest obstacle. Then, for a given measurement endpoint $(x_i^{world}, y_i^{world})$:

$$p(z_i \mid \mathbf{x}, m) = \alpha_{hit}\, \frac{1}{\sqrt{2\pi\sigma^2}} \exp\!\left(-\frac{d_{nearest}^2}{2\sigma^2}\right) + \alpha_{rand}\, \frac{1}{z_{max}}$$

Where $d_{nearest}$ = distance from the measurement endpoint to the closest obstacle in the map.

**Advantages:**
- No ray-casting needed → much faster
- Smooth gradient → works well with optimization-based methods

**Disadvantages:**
- Cannot model $p_{short}$ (unexpected obstacles)
- Cannot model $p_{max}$ (max-range returns)

---

## 3.5 — Uncertainty Propagation: Range-Bearing to Cartesian

Given noisy measurements $(r, \phi)$ with:
- Range noise: $\sigma_r$ (e.g., 0.02 m)
- Angular noise: $\sigma_\phi$ (e.g., 0.1° ≈ 0.00175 rad)

The Cartesian coordinates $(x^s, y^s)$ have uncertainty:

$$\mathbf{P}_{xy} = \mathbf{J}_{polar} \begin{bmatrix} \sigma_r^2 & 0 \\ 0 & \sigma_\phi^2 \end{bmatrix} \mathbf{J}_{polar}^\top$$

Where the Jacobian is:

$$\mathbf{J}_{polar} = \frac{\partial(x^s, y^s)}{\partial(r, \phi)} = \begin{bmatrix} \cos\phi & -r\sin\phi \\ \sin\phi & r\cos\phi \end{bmatrix}$$

**Result:**

$$\mathbf{P}_{xy} = \begin{bmatrix} \sigma_r^2\cos^2\phi + r^2\sigma_\phi^2\sin^2\phi & (\sigma_r^2 - r^2\sigma_\phi^2)\sin\phi\cos\phi \\ (\sigma_r^2 - r^2\sigma_\phi^2)\sin\phi\cos\phi & \sigma_r^2\sin^2\phi + r^2\sigma_\phi^2\cos^2\phi \end{bmatrix}$$

> **Key insight:** The angular component $r^2\sigma_\phi^2$ grows with $r^2$. At long range, angular noise dominates. The uncertainty ellipse elongates **tangentially** (perpendicular to the beam) at far distances.

---

## 3.6 — Measurement Theory: Mahalanobis Distance

When we get a LiDAR reading, we want to ask: **"Is this reading consistent with what we expected?"**

The **Mahalanobis distance** answers this:

$$d_M = \sqrt{(\mathbf{z} - \hat{\mathbf{z}})^\top \mathbf{S}^{-1} (\mathbf{z} - \hat{\mathbf{z}})}$$

Where:
- $\mathbf{z}$ = actual measurement
- $\hat{\mathbf{z}}$ = predicted measurement
- $\mathbf{S}$ = innovation covariance (combined measurement + prediction uncertainty)

**Interpretation:**
- $d_M < 2$: measurement is consistent — probably correct
- $d_M > 3$: measurement is suspicious — possible outlier or sensor failure
- $d_M > 5$: almost certainly wrong — reject this measurement

> **This is the foundation of "gating" in sensor fusion** — we will use Mahalanobis distance later to decide whether to accept or reject sensor readings.

---

## 3.7 — Measurement Theory: Log-Likelihood

When computing the probability of an entire LiDAR scan (hundreds of beams), we multiply the individual beam probabilities:

$$p(\mathbf{z} \mid \mathbf{x}, m) = \prod_{i=1}^{K} p(z_i \mid \mathbf{x}, m)$$

**Problem:** multiplying hundreds of small probabilities → numerical underflow (number becomes too small for the computer).

**Solution:** use the **log-likelihood** instead:

$$\ell(\mathbf{z} \mid \mathbf{x}, m) = \sum_{i=1}^{K} \log p(z_i \mid \mathbf{x}, m)$$

- Sums are numerically stable
- Maximizing log-likelihood = maximizing likelihood (log is monotonic)

---

## 3.8 — Worked Example: MediBot Scan in the Lobby

**Setup:**
- MediBot is at pose $(5.0, 3.0, 0°)$ in the lobby
- LiDAR: 360 beams, 1° resolution, $z_{max} = 30$ m, $\sigma_r = 0.02$ m
- Beam at $\phi = 45°$ returns $r = 4.2$ m (hitting a pillar)

**Cartesian endpoint in sensor frame:**

$$x^s = 4.2 \cos(45°) = 2.970 \text{ m}, \quad y^s = 4.2 \sin(45°) = 2.970 \text{ m}$$

**Uncertainty at this range (with $\sigma_\phi = 0.00175$ rad):**

- Range contribution: $\sigma_r^2 = 0.0004 \text{ m}^2$
- Angular contribution: $r^2\sigma_\phi^2 = 4.2^2 \times 0.00175^2 = 0.000054 \text{ m}^2$
- Angular contribution is smaller here, but at $r = 20$ m: $r^2\sigma_\phi^2 = 0.00123 \text{ m}^2$ — **3× larger than range noise**

**Beam at $\phi = 90°$ hits a glass door → returns $z_{max} = 30$ m:**

This is a $p_{max}$ event — the laser passed through the glass. The beam model assigns high probability to this event via the $\alpha_{max}$ component. This is a **known failure mode** of LiDAR.

> **Note:** Ultrasonic sensors would DETECT this glass door because sound reflects off glass (acoustic impedance mismatch). This is an example of **complementary fusion** — using ultrasonic to fill LiDAR's glass blind spot.

---

## 3.9 — Sensor Characteristics: 2D vs. 3D

| Parameter | 2D LiDAR | 3D LiDAR |
|-----------|----------|----------|
| Beams per scan | 360–1800 | 16,000–300,000+ |
| Vertical FoV | Single plane (0°) | 30°–90° vertical |
| Range | 10–30 m (indoor) | 30–200 m (outdoor) |
| Update rate | 10–40 Hz | 10–20 Hz |
| Cost (2024–2025) | $200–$2,000 | $1,000–$10,000+ |
| Data output | 2D point cloud (x, y) | 3D point cloud (x, y, z) |

**2D LiDAR limitation in hospital:** Only scans ONE horizontal plane. It will MISS:
- Table edges above the scan plane
- Low obstacles below the scan plane (fallen object, wheelchair footrest)
- Hanging signs, IV poles above scan height

---

## 3.10 — Operate Perspective: When to Use / Not Use

| Aspect | Assessment |
|--------|-----------|
| **Use for** | Hallway mapping, localization (scan matching / AMCL), obstacle avoidance, SLAM |
| **Do NOT use for** | Detecting glass walls/doors (laser passes through), identifying WHAT an object is (no color/texture), low-budget projects (3D) |
| **Works well when** | Environment has solid, diffuse surfaces; structured geometry (walls, pillars); stable lighting irrelevant |
| **Fails when** | Glass/transparent surfaces, highly reflective mirrors (beam bounces to wrong place), black surfaces (absorb laser), rain/fog/heavy dust (outdoor), moving between vertical levels (2D) |
| **Hospital-specific** | Excellent in hallways and rooms; fails at glass ICU partitions, glass entrance doors; 2D misses IV poles above scan plane |

---

## 3.11 — 💥 FAILURE → Transition to Block 4

> MediBot maps the lobby beautifully with LiDAR and navigates to the elevator. It enters and the doors close. As the elevator rides to Floor 3, it **vibrates and tilts slightly**. The LiDAR scan shifts — but the LiDAR only measures the geometry AROUND MediBot, not MediBot's own MOTION.
>
> When the elevator doors open on Floor 3, MediBot's heading estimate is **rotated by 3°** (because it didn't detect the subtle elevator rotation). This 3° error means that after 20 meters of corridor travel, MediBot is offset by over **1 meter** laterally ($20 \times \sin 3° \approx 1.05$ m) — enough to clip the doorframe of Ward B.
>
> LiDAR measures the external world perfectly — but has **no knowledge of MediBot's own acceleration, rotation, or tilt.** We need a sensor that directly measures the robot's **inertial motion**.

**Question for students:**  
> *What sensor can feel acceleration, rotation, and tilt — without needing any external reference?*

---
---

# BLOCK 4: IMU (Accelerometer + Gyroscope)

---

## 4.1 — Scenario: The Elevator Problem

MediBot is in the elevator. For ~15 seconds, it has no LiDAR reference (enclosed metal box, doors closed). The elevator accelerates upward, vibrates, and rotates slightly.

We add a **6-axis IMU** — a small chip containing a 3-axis accelerometer and a 3-axis gyroscope. It outputs data at **200 Hz** (every 5 ms). Now MediBot can FEEL every motion.

**Question for students:**  
> *An accelerometer measures acceleration, a gyroscope measures angular velocity. Why can't we just integrate these to get position and orientation perfectly?*

---

## 4.2 — Gyroscope Model

The gyroscope measures angular velocity in the **body frame**:

$$\boldsymbol{\omega}_{meas} = \boldsymbol{\omega}_{true} + \mathbf{b}_g + \mathbf{n}_g$$

Where:
- $\boldsymbol{\omega}_{meas}$ = measured angular velocity (3×1 vector: roll rate, pitch rate, yaw rate)
- $\boldsymbol{\omega}_{true}$ = true angular velocity
- $\mathbf{b}_g$ = **gyroscope bias** — a slowly drifting offset
- $\mathbf{n}_g$ = **white noise** — $\mathbf{n}_g \sim \mathcal{N}(\mathbf{0}, \sigma^2_g \mathbf{I})$

### Bias drift model

The bias is NOT constant — it wanders slowly over time:

$$\mathbf{b}_g(k+1) = \mathbf{b}_g(k) + \mathbf{w}_b, \qquad \mathbf{w}_b \sim \mathcal{N}(\mathbf{0}, \sigma^2_{bg}\mathbf{I})$$

This is a **random walk**: each step the bias moves a tiny random amount. Over minutes, the accumulated drift becomes significant.

### Integration: angular velocity → orientation

To get orientation (angle), we integrate:

$$\theta(t) = \theta(0) + \int_0^t \omega_{meas}(\tau) \, d\tau = \theta(0) + \int_0^t [\omega_{true}(\tau) + b_g(\tau) + n_g(\tau)] \, d\tau$$

**Discrete form (for one axis):**

$$\theta_{k+1} = \theta_k + \omega_{meas,k} \cdot \Delta t$$

### Error analysis — what happens to the heading error?

The heading error after time $T$ has two components:

**From white noise** (angular random walk):

$$\sigma_\theta^{noise}(T) = \sigma_g \sqrt{\Delta t \cdot T} = \text{ARW} \cdot \sqrt{T}$$

Where ARW = Angular Random Walk (in °/√hr or rad/√s), a datasheet parameter.

**From bias drift:**

$$\sigma_\theta^{bias}(T) \approx b_g \cdot T$$

The bias contribution grows **linearly** with time — this dominates over long periods.

---

## 4.3 — Accelerometer Model

The accelerometer measures **specific force** (acceleration minus gravity) in the body frame:

$$\mathbf{f}_{meas} = \mathbf{R}_{body}^{world \top} (\mathbf{a}_{world} - \mathbf{g}) + \mathbf{b}_a + \mathbf{n}_a$$

Where:
- $\mathbf{f}_{meas}$ = measured specific force (3×1 vector)
- $\mathbf{R}_{body}^{world}$ = rotation matrix from world to body frame
- $\mathbf{a}_{world}$ = true acceleration in world frame
- $\mathbf{g} = [0, 0, -9.81]^\top$ = gravity vector in world frame
- $\mathbf{b}_a$ = accelerometer bias (drifts like gyroscope bias)
- $\mathbf{n}_a$ = white noise, $\mathbf{n}_a \sim \mathcal{N}(\mathbf{0}, \sigma^2_a \mathbf{I})$

### The gravity problem

To extract the true acceleration $\mathbf{a}_{world}$, we must:

1. **Know the orientation** $\mathbf{R}$ to transform from body to world frame
2. **Subtract gravity** $\mathbf{g}$

$$\mathbf{a}_{world} = \mathbf{R}_{body}^{world} (\mathbf{f}_{meas} - \mathbf{b}_a - \mathbf{n}_a) + \mathbf{g}$$

> **Critical coupling:** A small error in orientation → incorrect gravity subtraction → large acceleration error. For example, a 1° tilt error means $9.81 \times \sin(1°) \approx 0.17$ m/s² of false acceleration.

### Double integration: acceleration → velocity → position

$$v(t) = v(0) + \int_0^t a(\tau) \, d\tau$$
$$p(t) = p(0) + \int_0^t v(\tau) \, d\tau$$

### Error growth analysis

**From accelerometer white noise** $\sigma_a$:

| Quantity | Error growth |
|----------|-------------|
| Acceleration error | $\sigma_a$ (constant — just the noise) |
| Velocity error | $\sigma_v(T) = \sigma_a \cdot T$ (linear) |
| Position error | $\sigma_p(T) = \frac{1}{2}\sigma_a \cdot T^2$ (quadratic) |

**From accelerometer bias** $b_a$:

| Quantity | Error growth |
|----------|-------------|
| Acceleration error | $b_a$ (constant) |
| Velocity error | $b_a \cdot T$ (linear) |
| Position error | $\frac{1}{2}b_a \cdot T^2$ (quadratic) |

> **Key insight:** Position error from accelerometer integration grows as $T^2$. After just 10 seconds with a typical MEMS accelerometer ($b_a \approx 0.01$ m/s²), position error reaches $\frac{1}{2}(0.01)(10^2) = 0.5$ m. After 60 seconds: **18 meters of error!**

---

## 4.4 — Allan Variance: Characterizing IMU Noise

The **Allan Variance** is a standard tool for characterizing IMU noise from the datasheet or from experimental data.

### What it is

1. Record IMU output for a long time while the IMU is **stationary**
2. Divide the data into clusters of length $\tau$ (the averaging time)
3. Compute the variance of cluster-to-cluster differences

$$\sigma^2_{Allan}(\tau) = \frac{1}{2} \langle (\bar{\omega}_{k+1} - \bar{\omega}_k)^2 \rangle$$

Where $\bar{\omega}_k$ is the average angular velocity in cluster $k$.

### Reading the Allan Variance plot

Plot $\sigma_{Allan}(\tau)$ vs. $\tau$ on a **log-log** scale. The slope reveals the noise type:

| Slope | Noise type | Physical meaning |
|-------|-----------|-----------------|
| $-1/2$ | White noise / Angle Random Walk | Electronic noise, quantization |
| $0$ | Bias instability | Minimum of the curve — the best the gyro can do |
| $+1/2$ | Rate random walk / Bias drift | Slow environmental changes, aging |

**Reading the key parameters:**
- **ARW** (Angular Random Walk) = value of $\sigma_{Allan}$ at $\tau = 1$ s, read from the $-1/2$ slope line
- **Bias instability** = the minimum value of $\sigma_{Allan}$ on the plot

### Typical MEMS IMU values (consumer-grade, e.g., BMI088)

| Parameter | Gyroscope | Accelerometer |
|-----------|-----------|---------------|
| White noise | ARW ≈ 0.2 °/√hr | VRW ≈ 0.06 m/s/√hr |
| Bias instability | ≈ 3–10 °/hr | ≈ 0.03–0.1 mg |
| Bias drift | Random walk | Random walk |

---

## 4.5 — Measurement Theory: Bias as a Hidden State

The gyroscope bias $\mathbf{b}_g$ cannot be measured directly — we can only ESTIMATE it.

**The idea:** Include the bias in the **state vector** that we estimate:

$$\mathbf{x}_{augmented} = \begin{bmatrix} x \\ y \\ \theta \\ v \\ b_g \\ b_a \end{bmatrix}$$

Now the state has 6+ dimensions instead of 3. The bias states have their own dynamics:

$$b_g(k+1) = b_g(k) + w_b$$

and are "observed" indirectly — when an external sensor (like LiDAR) corrects the position, the difference between prediction and measurement reveals information about the bias.

> **This is a preview of sensor fusion:** we will formally estimate bias as part of the Kalman filter state in the fusion module. For now, the key insight is: **bias is not a fixed number — it is a hidden variable that must be continuously estimated.**

### Observable vs. Unobservable states

**Question:** Can we estimate the gyroscope bias using ONLY the gyroscope?  
**Answer:** NO. The gyroscope cannot distinguish "true rotation" from "bias." We need an EXTERNAL reference (LiDAR, magnetometer, etc.) to make the bias **observable**.

This is the concept of **observability**: a state is observable if the available measurements contain enough information to estimate it. Bias is only observable when we have a complementary sensor that provides an absolute reference.

---

## 4.6 — Worked Example: MediBot in the Elevator

**Setup:**
- Elevator ride duration: $T = 15$ seconds
- Gyroscope bias: $b_g = 0.05$ °/s (typical for MEMS)
- Gyroscope white noise: ARW = 0.2 °/√hr = 0.0033 °/√s
- Accelerometer bias: $b_a = 0.5$ mg = 0.0049 m/s²

**Heading error after 15s:**

$$\epsilon_\theta^{bias} = b_g \times T = 0.05 \times 15 = 0.75°$$
$$\epsilon_\theta^{noise} = \text{ARW} \times \sqrt{T} = 0.0033 \times \sqrt{15} = 0.013°$$

**Total heading error:** ≈ 0.76° (dominated by bias)

**Position error from accelerometer (vertical axis) after 15s:**

$$\epsilon_p = \frac{1}{2} b_a \cdot T^2 = \frac{1}{2}(0.0049)(15^2) = 0.55 \text{ m}$$

This means MediBot cannot tell precisely which floor it's on using the accelerometer alone — the 0.55 m error is comparable to a floor-to-floor height uncertainty!

**After exiting the elevator and traveling 20m down the corridor:**

The 0.76° heading error causes a lateral offset:

$$\epsilon_y = 20 \times \sin(0.76°) = 0.27 \text{ m}$$

This is manageable — but if the elevator ride were 60 seconds (stuck between floors), heading error would be 3° and lateral offset would be over 1 meter.

---

## 4.7 — Operate Perspective: When to Use / Not Use

| Aspect | Assessment |
|--------|-----------|
| **Use for** | Short-term orientation tracking (< 30s without correction), vibration/bump detection, elevator/ramp detection, fast motion estimation between LiDAR scans |
| **Do NOT use for** | Standalone position estimation beyond ~10 seconds, absolute heading without external reference |
| **Works well when** | Used with frequent external corrections (LiDAR at 10Hz), short dead-reckoning intervals, bias estimated in real-time |
| **Fails when** | Long periods without external correction, strong vibration (saturates accelerometer), extreme temperature changes (bias shifts), pure accelerometer integration for position |
| **Hospital-specific** | Essential for elevator rides, ramp transitions, bump detection (doorway thresholds); but bias drift means it cannot be sole navigation during any period longer than ~15 seconds |

---

## 4.8 — 💥 FAILURE → Transition to Block 5

> MediBot exits the elevator on Floor 3 and heads toward Ward B. The LiDAR is momentarily blocked — a crowd of medical staff exiting the elevator surrounds MediBot. For 45 seconds, MediBot relies on IMU + odometry alone.
>
> **Result:** After 45 seconds, the IMU's gyroscope bias has accumulated a heading error of $0.05 \times 45 = 2.25°$. Combined with odometry drift, MediBot is 1.5 meters off course.
>
> But there's a bigger problem: MediBot can see (via LiDAR) that it's in a corridor — but it doesn't know WHICH corridor. All corridors on Floor 3 look geometrically identical. MediBot needs to READ the "Ward B — Cardiology" sign on the wall. It needs to RECOGNIZE a wheelchair blocking the path vs. a cart it can navigate around.
>
> **LiDAR gives geometry. The IMU gives motion. But neither gives MEANING.** We need a sensor that sees color, texture, and text — a sensor that understands WHAT things are, not just WHERE they are.

**Question for students:**  
> *What sensor can read text, recognize objects, and provide dense 3D information about the nearby environment?*

---
---

# BLOCK 5: Camera (RGB & Depth)

---

## 5.1 — Scenario: MediBot Needs Semantic Understanding

MediBot is on Floor 3 but doesn't know which ward it's near. We mount an **RGB-D camera** (e.g., Intel RealSense D435) — combining a color camera with a depth sensor. Now MediBot can read "Ward B — Cardiology" on the door sign and build a dense 3D model of the corridor.

**Question for students:**  
> *A camera gives us a 2D image. How do we relate pixels to 3D positions in the world?*

---

## 5.2 — Pinhole Camera Model

The camera projects 3D world points into 2D pixel coordinates:

### Projection equation

$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z_c} \mathbf{K} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}$$

Where:
- $(u, v)$ = pixel coordinates in the image
- $(X_c, Y_c, Z_c)$ = 3D point in the **camera frame**
- $Z_c$ = depth (distance along the camera's optical axis)

### Intrinsic matrix $\mathbf{K}$

$$\mathbf{K} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

Where:
- $f_x, f_y$ = focal lengths in pixels (may differ slightly due to non-square pixels)
- $(c_x, c_y)$ = principal point — where the optical axis intersects the image plane (ideally image center)

### Full projection (world → pixel)

To go from **world frame** to **pixel coordinates**, we also need the **extrinsic parameters** $[\mathbf{R}_{cw} | \mathbf{t}_{cw}]$ (camera pose relative to world):

$$\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = \mathbf{R}_{cw} \begin{bmatrix} X_w \\ Y_w \\ Z_w \end{bmatrix} + \mathbf{t}_{cw}$$

Then project:

$$u = f_x \frac{X_c}{Z_c} + c_x, \qquad v = f_y \frac{Y_c}{Z_c} + c_y$$

> **Key insight:** Projection is a **lossy** transformation — depth $Z_c$ is lost. A pixel $(u, v)$ could correspond to any point along a **ray** from the camera center. This is why we need depth information separately.

---

## 5.3 — Lens Distortion

Real lenses are not perfect pinholes. They introduce **distortion**:

### Radial distortion (barrel / pincushion)

$$x_{distorted} = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$$
$$y_{distorted} = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$$

Where $r^2 = x^2 + y^2$ and $(x, y)$ are normalized image coordinates.

### Tangential distortion

$$x_{distorted} = x + 2p_1 xy + p_2(r^2 + 2x^2)$$
$$y_{distorted} = y + p_1(r^2 + 2y^2) + 2p_2 xy$$

**Distortion coefficients:** $(k_1, k_2, p_1, p_2, k_3)$ — estimated during calibration.

In practice, we **undistort** the image before using it — applying the inverse transformation to straighten the image.

---

## 5.4 — Depth Sensing Models

### Stereo depth (two cameras, known baseline)

Two cameras separated by baseline $B$:

$$Z = \frac{f \cdot B}{d}$$

Where:
- $Z$ = depth to the point
- $f$ = focal length (in pixels)
- $B$ = baseline (physical distance between the two cameras)
- $d$ = disparity (difference in pixel position between left and right image)

### Depth uncertainty from stereo

Differentiating $Z = fB/d$ with respect to disparity:

$$\frac{dZ}{dd} = -\frac{fB}{d^2} = -\frac{Z^2}{fB}$$

Therefore:

$$\sigma_Z = \left|\frac{Z^2}{fB}\right| \sigma_d$$

Where $\sigma_d$ is the disparity estimation noise (typically 0.5–2 pixels).

> **Key insight:** Depth uncertainty grows as $Z^2$ — **quadratically** with distance! At 1m depth, uncertainty might be 1 cm. At 5m depth, it's **25× worse** = 25 cm. This is **heteroscedastic noise** — the measurement uncertainty depends on the measurement itself.

### Structured light depth (e.g., Intel RealSense)

- Projects an IR pattern onto the scene
- Computes depth by matching the deformed pattern
- Mathematically similar to stereo — depth uncertainty also grows with $Z^2$
- **Fails on:** glass (IR passes through), surfaces in direct sunlight (IR saturated), textureless white walls

### Time-of-Flight depth camera

- Emits modulated IR light, measures phase shift
- Depth uncertainty is approximately constant with range (unlike stereo)
- **Fails on:** reflective surfaces (multipath), black surfaces (absorption), interference from other ToF cameras

---

## 5.5 — Measurement Theory: Heteroscedastic Noise

Traditional models assume noise $\sigma$ is constant. Cameras violate this:

### Constant noise (homoscedastic):
$$z = h(\mathbf{x}) + v, \qquad v \sim \mathcal{N}(0, R) \quad \text{(R is constant)}$$

### Depth-dependent noise (heteroscedastic):
$$z_{depth} = Z_{true} + v(Z), \qquad v(Z) \sim \mathcal{N}\!\left(0, \left(\frac{Z^2}{fB}\right)^2 \sigma_d^2\right)$$

The noise covariance $R$ is now a **function of the measurement**:

$$R(Z) = \left(\frac{Z^2}{fB}\right)^2 \sigma_d^2$$

> **Implication for fusion:** When we fuse camera depth with LiDAR range, we must use the **appropriate $R$ for each measurement**. A camera depth at 1m should be trusted more than one at 5m.

---

## 5.6 — Calibration

### Intrinsic calibration

**Goal:** Estimate $f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3$

**Method (Zhang's method):**
1. Print a **checkerboard pattern** with known square size
2. Take 15–30 images of the checkerboard from different angles and distances
3. Detect corner points automatically
4. Solve a nonlinear optimization to find the parameters that minimize **reprojection error**:

$$E = \sum_{i=1}^{N}\sum_{j=1}^{M} \left\| \mathbf{p}_{ij}^{observed} - \pi(\mathbf{K}, \mathbf{d}, \mathbf{R}_i, \mathbf{t}_i, \mathbf{P}_j) \right\|^2$$

Where $\pi()$ is the full projection function with distortion.

### Extrinsic calibration (camera → robot body)

**Goal:** Find the transformation $[\mathbf{R}, \mathbf{t}]$ from camera frame to robot body frame

**Method:** Solve the "hand-eye calibration" problem — measure the same points from both the camera and a known reference (e.g., LiDAR) and find the rigid transformation that aligns them.

---

## 5.7 — Worked Example: MediBot Reading a Ward Sign

**Setup:**
- Camera: $f_x = f_y = 600$ pixels, image size 640×480
- Ward sign "B" is at world position $(3.0, 0.5, 1.5)$ m
- MediBot camera pose: looking straight at the sign, camera at $(0, 0, 1.2)$ m
- Stereo baseline: $B = 0.05$ m

**Step 1: Sign distance**

$Z = 3.0$ m (depth along optical axis)

**Step 2: Pixel coordinates of sign center**

$$u = 600 \times \frac{0.5}{3.0} + 320 = 420 \text{ px}, \quad v = 600 \times \frac{1.5 - 1.2}{3.0} + 240 = 300 \text{ px}$$

**Step 3: Depth uncertainty at 3m**

$$\sigma_Z = \frac{Z^2}{fB}\sigma_d = \frac{3.0^2}{600 \times 0.05}(1.0) = \frac{9}{30} = 0.30 \text{ m}$$

At 3 meters, the depth uncertainty is **30 cm** — fine for sign detection but too imprecise for obstacle avoidance!

Compare: at 1m → $\sigma_Z = 0.033$ m (3.3 cm) — **9× better**.

---

## 5.8 — Operate Perspective: When to Use / Not Use

| Aspect | Assessment |
|--------|-----------|
| **Use for** | Sign/label reading (OCR), person detection, close-range dense 3D (< 4m), visual odometry, object recognition, semantic understanding |
| **Do NOT use for** | Long-range depth (> 8m stereo), dark environments, sole navigation sensor, environments with strong IR interference |
| **Works well when** | Good lighting, textured surfaces, range < 4m (for depth), rich visual features |
| **Fails when** | Direct sunlight saturates IR (depth camera dies), textureless white walls (no stereo match), dark corridors (RGB useless), motion blur at high speed, reflective floors (IR multipath) |
| **Hospital-specific** | Reads ward signs (semantic localization), detects people/wheelchairs; but fluorescent lights flicker at camera frame rate, and IR depth fails near windows with sunlight. Depth camera CANNOT see through glass (IR passes through glass just like LiDAR light). |

---

## 5.9 — 💥 FAILURE → Transition to Block 6

> MediBot reaches the Ward B corridor. A supply cart is approaching from **behind** at 2 m/s. The camera faces forward — it has a limited field of view (~70°) and **cannot see behind**. Even the forward-facing depth camera struggles: the corridor has a large window at the end, and afternoon sunlight floods in, **saturating the IR depth sensor**. The depth image turns to noise.
>
> MediBot also cannot tell HOW FAST the supply cart is moving — the camera only sees position at each frame; computing velocity requires differentiating noisy position estimates, which amplifies noise.
>
> We need a sensor that:
> - Works in **any lighting condition**
> - Can cover a **wide area including behind the robot**
> - Directly measures **velocity** (not just position)
> - Sees through glass, dust, and changing light

**Question for students:**  
> *What sensor uses radio waves — which are unaffected by light, glass, dust, and fog — and can directly measure velocity via the Doppler effect?*

---
---

# BLOCK 6: Radar (mmWave)

---

## 6.1 — Scenario: MediBot Needs All-Weather, All-Condition Sensing

MediBot's camera and LiDAR both struggle with the sunlit window and glass partitions. We add a **mmWave radar module** (e.g., Texas Instruments IWR1443/IWR6843) — a small, affordable chip that transmits and receives millimeter-wave radio signals.

Radar operates at 60–80 GHz — radio waves that:
- Pass through glass, plastic, dust, fog, smoke
- Work in complete darkness and blinding sunlight
- **Directly measure velocity** of moving objects via the Doppler effect

**Question for students:**  
> *LiDAR measures (range, angle). What extra dimension does radar add — and why is that so valuable?*

---

## 6.2 — Sensor Model: Range-Azimuth-Doppler

Each radar detection returns a **3-tuple** $(r, \theta, v_r)$:

- $r$ = range to the target
- $\theta$ = azimuth angle (horizontal direction)
- $v_r$ = **radial velocity** (velocity component along the line from radar to target)

> This is fundamentally different from other sensors — radar gives velocity **for free**, without differentiating position. No integration drift, no noise amplification.

---

## 6.3 — FMCW Radar Principle

Modern automotive/robotics radars use **Frequency-Modulated Continuous Wave (FMCW)**:

### How it works

1. **Transmit** a signal whose frequency increases linearly over time (**chirp**):
   $$f_{TX}(t) = f_0 + \frac{BW}{T_c} \cdot t$$
   Where $f_0$ = start frequency, $BW$ = bandwidth, $T_c$ = chirp duration

2. **Receive** the echo after delay $\tau = 2r/c$

3. **Mix** the received signal with the transmitted signal → **beat frequency**:
   $$f_{beat} = \frac{BW}{T_c} \cdot \tau = \frac{2 \cdot BW \cdot r}{c \cdot T_c}$$

4. **Compute range** from the beat frequency:
   $$r = \frac{f_{beat} \cdot c \cdot T_c}{2 \cdot BW}$$

### Range resolution

$$\Delta r = \frac{c}{2 \cdot BW}$$

| Bandwidth | Range Resolution |
|-----------|-----------------|
| 1 GHz | 0.15 m |
| 4 GHz | 0.0375 m |

> **Key point:** Range resolution depends ONLY on bandwidth, not on carrier frequency.

### Velocity from Doppler effect

By transmitting multiple chirps and measuring the phase change between them:

$$v_r = \frac{\lambda \cdot f_{Doppler}}{2}$$

Where $\lambda = c / f_0$ is the wavelength.

**Maximum unambiguous velocity:**

$$v_{max} = \frac{\lambda}{4 \cdot T_c}$$

**Velocity resolution:**

$$\Delta v = \frac{\lambda}{2 \cdot N_{chirps} \cdot T_c}$$

---

## 6.4 — Resolution vs. Accuracy

This is a critical distinction that radar makes very clear:

| Concept | Definition | Radar example |
|---------|-----------|---------------|
| **Resolution** | Minimum separation between two distinguishable targets | $\Delta r = c/(2 \cdot BW) = 0.15$ m (with 1 GHz BW) |
| **Accuracy** | How close a single measurement is to the true value | Can be sub-centimeter with sufficient SNR |

> A radar with 15 cm range resolution can still measure a SINGLE target's range with 1 cm accuracy — but it CANNOT tell apart two targets that are 10 cm away from each other.

### Angular resolution

$$\Delta\theta \approx \frac{\lambda}{D \cdot \cos\theta}$$

Where $D$ = antenna aperture size.

For a typical mmWave module at 77 GHz ($\lambda = 3.9$ mm) with $D = 5$ cm:

$$\Delta\theta \approx \frac{0.0039}{0.05} = 0.078 \text{ rad} \approx 4.5°$$

Compare to LiDAR angular resolution: ~0.1–0.5°. Radar's angular resolution is **10–50× worse** than LiDAR.

---

## 6.5 — Measurement Theory: Doppler as Direct Velocity

### Why Doppler velocity is special

With encoders or IMU, we get velocity by **integration or differentiation**:
- Encoder → position changes → differentiate → velocity (amplifies noise)
- IMU accelerometer → acceleration → integrate → velocity (accumulates bias)

Radar gives velocity DIRECTLY:
- No integration → no drift
- No differentiation → no noise amplification
- Each measurement is **independent** — no temporal correlation

### Velocity measurement model

$$v_{r,meas} = v_{r,true} + n_v, \qquad n_v \sim \mathcal{N}(0, \sigma^2_v)$$

With $\sigma_v$ typically 0.05–0.1 m/s — very precise.

---

## 6.6 — Failure Modes in Hospital

| Failure mode | Cause | Effect |
|-------------|-------|--------|
| **Multipath** | Radar bounces off metal walls, IV poles, equipment before reaching target | Ghost targets at wrong positions |
| **Clutter** | Strong static reflections from large metal objects (elevator doors, bed frames) | Masks weaker moving targets |
| **Low angular resolution** | Small antenna aperture | Two people side-by-side appear as one target |
| **Radar cross-section (RCS) variation** | Metal cart (large RCS) vs. person in cotton scrubs (small RCS) | Person may be harder to detect than cart |

### Radar cross-section

The RCS determines how "visible" a target is:

$$\text{SNR} \propto \frac{\sigma_{RCS}}{r^4}$$

Where $\sigma_{RCS}$ is the radar cross-section. Notice the **$r^4$ dependency** — doubling range reduces SNR by 16×!

| Target | Typical RCS |
|--------|------------|
| Metal cart | 10–30 m² |
| Person (standing) | 0.5–2 m² |
| Wheelchair | 2–5 m² |
| Glass door (with metal frame) | 1–5 m² |

---

## 6.7 — Worked Example: Detecting the Supply Cart

**Setup:**
- Supply cart approaching from behind at 2 m/s
- Radar mounted on MediBot's rear, 77 GHz, BW = 4 GHz
- Cart is 8 m away, metal body, RCS ≈ 15 m²

**Range measurement:**

$$r_{meas} = 8.0 + n_r, \quad n_r \sim \mathcal{N}(0, (0.02)^2) \text{ m}$$

**Velocity measurement:**

$$v_{r,meas} = -2.0 + n_v, \quad n_v \sim \mathcal{N}(0, (0.05)^2) \text{ m/s}$$

(Negative because cart is approaching — closing range)

**Range resolution with 4 GHz BW:**

$$\Delta r = \frac{3 \times 10^8}{2 \times 4 \times 10^9} = 0.0375 \text{ m}$$

**Can the radar see through the glass partition?**

YES — 77 GHz radar can detect objects through standard glass with some attenuation (~3–5 dB loss). The cart's strong RCS means it remains detectable.

**Time to collision:**

$$t_{collision} = \frac{r}{|v_r|} = \frac{8.0}{2.0} = 4.0 \text{ s}$$

MediBot has 4 seconds to react — plenty of time to stop or yield if it processes the radar data promptly.

---

## 6.8 — Operate Perspective: When to Use / Not Use

| Aspect | Assessment |
|--------|-----------|
| **Use for** | Moving object detection + velocity, all-weather sensing, seeing through glass/dust/smoke, rear collision avoidance, object tracking |
| **Do NOT use for** | Fine-grained shape recognition (angular resolution too poor), distinguishing two people standing next to each other, sole navigation sensor indoors |
| **Works well when** | Detecting moving targets (Doppler separates from clutter), long-range detection, environments with glass/dust/variable lighting |
| **Fails when** | Dense metallic environments (multipath/clutter overwhelming), targets have very small RCS, need high spatial detail |
| **Hospital-specific** | Detects approaching carts/people through glass partitions and around corners (radio reflects off walls); but metal-heavy hospital equipment creates multipath clutter that requires careful processing |

---

## 6.9 — 💥 CRITICAL REALIZATION → Transition to Fusion

> MediBot now has **6 sensors**, each running simultaneously:
>
> | Sensor | Gives | Fails at |
> |--------|-------|---------|
> | Wheel encoders | Motion (Δs, Δθ) | Drift, slip |
> | Ultrasonic | Close-range obstacles, **glass detection** | Short range, wide beam, specular reflection at angles |
> | LiDAR | Rich geometry, long range | Glass, mirrors, no semantics |
> | IMU | Fast motion, orientation | Drift in seconds |
> | Camera | Semantics, dense depth | Lighting, sunlight, limited FoV, depth noise at range |
> | Radar | Velocity, all-weather, through glass | Low angular resolution, multipath, clutter |
>
> **No single sensor is sufficient for safe hospital navigation.**
>
> Each sensor excels where others fail. The question is no longer "which sensor?" but:
> **"How do we OPTIMALLY COMBINE them?"**

---
---

# BLOCK 7: Why Sensor Fusion?

---

## 7.1 — The Core Idea

**Sensor fusion** = combining measurements from multiple sensors to produce an estimate that is **better than any single sensor alone**.

"Better" means:
- **More accurate** (lower error)
- **More robust** (survives individual sensor failures)
- **More complete** (covers more situations)

---

## 7.2 — Three Types of Fusion

### Type 1: Complementary Fusion

Sensors provide **different types of information** that fill each other's gaps.

**MediBot example:**
- IMU provides orientation at **200 Hz** — fast, but drifts
- LiDAR provides absolute position at **10 Hz** — slow, but no drift

**Fusion strategy:** Use the IMU to **predict** MediBot's pose between LiDAR scans, then **correct** when LiDAR data arrives.

$$\text{IMU fills the gaps between LiDAR updates}$$
$$\text{LiDAR corrects the IMU drift periodically}$$

### Type 2: Redundant Fusion

Sensors measure the **same thing** → increased reliability.

**MediBot example:**
- LiDAR measures range to a wall: 2.50 m
- Ultrasonic measures range to the same wall: 2.48 m

**Fusion strategy:** Combine both estimates. If they agree → more confident. If one fails → the other keeps working.

### Type 3: Competitive Fusion

Sensors compete — use the **best one for the current condition**.

**MediBot example:**
- In a bright corridor with good features: use camera depth
- Near a glass partition: switch to ultrasonic (camera IR and LiDAR fail on glass)
- Detecting a moving person in variable light: use radar (gives velocity directly)

**Fusion strategy:** Monitor sensor health, weight each by its reliability in the current context.

---

## 7.3 — The Bayesian Insight: Weight by Uncertainty

The central principle of optimal fusion:

> **Trust each sensor inversely proportional to its uncertainty.**

### Simple example: Fusing two range measurements

Suppose two sensors measure the same distance:
- Sensor A: $z_A = 2.50$ m with $\sigma_A = 0.05$ m
- Sensor B: $z_B = 2.42$ m with $\sigma_B = 0.10$ m

The **optimal fused estimate** (minimum variance) is:

$$\hat{z} = \frac{\sigma_B^2 \cdot z_A + \sigma_A^2 \cdot z_B}{\sigma_A^2 + \sigma_B^2}$$

$$= \frac{0.01 \times 2.50 + 0.0025 \times 2.42}{0.0025 + 0.01} = \frac{0.025 + 0.00605}{0.0125} = 2.484 \text{ m}$$

The fused uncertainty:

$$\sigma_{fused}^2 = \frac{\sigma_A^2 \cdot \sigma_B^2}{\sigma_A^2 + \sigma_B^2} = \frac{0.0025 \times 0.01}{0.0125} = 0.002 \text{ m}^2$$

$$\sigma_{fused} = 0.045 \text{ m}$$

**Result:**
- $\sigma_{fused} = 0.045$ m is **smaller than EITHER individual sensor** (0.05 m and 0.10 m)
- The fused estimate is closer to sensor A because sensor A has lower uncertainty → higher weight

> **This is the miracle of sensor fusion:** the fused estimate is ALWAYS at least as good as the best individual sensor, and usually better.

---

## 7.4 — The Predict-Update Cycle (Conceptual Preview)

The formal mathematical framework is covered in a later module (Kalman Filter). Here we introduce the **concept**:

### Step 1: PREDICT
Use the motion model (encoders + IMU) to **predict** where MediBot should be:

$$\hat{\mathbf{x}}_{k+1}^{predict} = f(\hat{\mathbf{x}}_k, \mathbf{u}_k) \qquad \text{(where } \mathbf{u}_k \text{ = control input)}$$

The prediction uncertainty **grows** (because motion adds noise).

### Step 2: UPDATE (CORRECT)
When an exteroceptive sensor (LiDAR, camera, radar, ultrasonic) provides a measurement:

$$\hat{\mathbf{x}}_{k+1}^{corrected} = \hat{\mathbf{x}}_{k+1}^{predict} + \mathbf{K} (\mathbf{z} - h(\hat{\mathbf{x}}_{k+1}^{predict}))$$

Where:
- $\mathbf{z}$ = the actual measurement
- $h(\hat{\mathbf{x}})$ = what we EXPECTED to measure
- $\mathbf{z} - h(\hat{\mathbf{x}})$ = the **innovation** (surprise)
- $\mathbf{K}$ = **gain** — how much to trust the measurement vs. the prediction

The correction **shrinks** the uncertainty.

### The cycle

$$\text{PREDICT (grow uncertainty)} \rightarrow \text{UPDATE (shrink uncertainty)} \rightarrow \text{PREDICT} \rightarrow \text{UPDATE} \rightarrow \ldots$$

> **Key insight:** This cycle is WHY sensor fusion works. Proprioceptive sensors (encoders, IMU) provide fast predictions that drift. Exteroceptive sensors (LiDAR, camera, radar, ultrasonic) provide periodic corrections that bound the drift. Together, they produce continuous, bounded uncertainty.

---

## 7.5 — MediBot's Sensor Fusion Timeline

Here is what happens during MediBot's mission, moment by moment:

| Time | Event | Prediction source | Correction source |
|------|-------|------------------|------------------|
| 0s | Leave pharmacy | Encoders | LiDAR (hallway walls) |
| 0–30s | Travel corridor | Encoders + IMU (200 Hz) | LiDAR (10 Hz), ultrasonic (near walls) |
| 30s | Approach glass door | Encoders + IMU | **Ultrasonic** (detects glass), camera (reads sign) |
| 35s | Enter elevator | Encoders + IMU | Nothing — enclosed box |
| 35–50s | Ride elevator | **IMU only** (feels motion) | Nothing — drift accumulates |
| 50s | Doors open Floor 3 | Encoders + IMU | **LiDAR** corrects (hallway geometry) — uncertainty collapses |
| 50–90s | Navigate to Ward B | Encoders + IMU | LiDAR + camera (reads "Ward B") |
| 85s | Cart approaching from behind | — | **Radar** (detects, measures velocity) |
| 90s | Arrive at Ward B | Encoders | LiDAR + camera (confirm position at door) |

> **Notice:** Different sensors take the lead at different moments. The fusion system seamlessly hands off between them based on what's available and reliable.

---
---

# BLOCK 8: Fusion Architecture & Robustness

---

## 8.1 — How to Wire 6 Sensors Together

### Option A: Centralized Fusion

All sensors feed into **one filter** that estimates the full state:

$$\mathbf{x} = [x, y, \theta, v, b_g, b_a, \ldots]$$

**Pros:** Theoretically optimal — all information is combined at once  
**Cons:** Complex, single point of failure, hard to debug

### Option B: Decentralized (Federated) Fusion

Each sensor has its **own local filter** → local estimates are **merged**:

- Filter 1: Encoders + IMU → dead reckoning estimate
- Filter 2: LiDAR → scan matching estimate
- Filter 3: Camera → visual odometry estimate
- **Master filter:** combines the three estimates

**Pros:** Robust — one filter can fail without crashing everything, easier to test each subsystem  
**Cons:** Suboptimal (correlations between filters are approximated), more computational overhead

> **MediBot recommendation:** Federated architecture — because in a safety-critical hospital environment, we need **graceful degradation** if a sensor fails.

---

## 8.2 — Multi-Rate & Asynchronous Data

MediBot's sensors do NOT all update at the same rate:

| Sensor | Update rate | Latency |
|--------|-----------|---------|
| Encoders | 1000 Hz | < 1 ms |
| IMU | 200 Hz | < 1 ms |
| LiDAR | 10 Hz | 20–50 ms |
| Camera | 30 Hz | 30–100 ms |
| Radar | 15 Hz | 10–30 ms |
| Ultrasonic | 20 Hz | 2–30 ms |

### The challenge

At time $t$, a LiDAR scan arrives — but it was captured 30 ms ago. The IMU has moved the robot since then. How do we fuse this "stale" data correctly?

### The solution (concept)

**Timestamp every measurement** and fuse it at the correct point in the timeline:
1. When stale data arrives, "rewind" the state estimate to the measurement time
2. Apply the correction
3. "Fast-forward" back to the current time using IMU predictions

> **Practical note:** This is called **out-of-order measurement handling** and is a standard technique in navigation systems.

---

## 8.3 — Sensor Health Monitoring

### Innovation check (Mahalanobis gating)

Before accepting a sensor measurement, check if it's consistent with the prediction:

$$d_M = \sqrt{(\mathbf{z} - \hat{\mathbf{z}})^\top \mathbf{S}^{-1} (\mathbf{z} - \hat{\mathbf{z}})}$$

**Decision rule:**
- $d_M < \gamma$ (e.g., $\gamma = 3$): **Accept** the measurement
- $d_M \geq \gamma$: **Reject** — this measurement is an outlier or the sensor has failed

### Chi-squared consistency test

Monitor the innovation sequence over time. If the average Mahalanobis distance exceeds the expected value (from a $\chi^2$ distribution), the filter's uncertainty model is **inconsistent** — either the sensor has degraded or the motion model is wrong.

### Graceful degradation

When a sensor fails (health check triggers):
1. **Reduce its weight** (increase its noise covariance $R$ → the fusion trusts it less)
2. If persistent failure: **disable it entirely** (set weight to zero)
3. **Alert the operator** (log the failure, flag the sensor)
4. Continue operating with remaining sensors

**MediBot example:** Camera depth fails when passing a sunlit window.  
→ Innovation check detects depth readings are wildly inconsistent  
→ Camera depth weight set to zero  
→ MediBot continues with LiDAR + ultrasonic + radar for obstacle avoidance  
→ When MediBot enters the shaded corridor, camera depth recovers, health check passes, weight restored

---

## 8.4 — Common-Cause Failures

Some failures affect **multiple sensors simultaneously**:

| Common cause | Sensors affected | Mitigation |
|-------------|-----------------|-----------|
| Power supply failure | All electronic sensors | Redundant power rails, battery backup |
| Heavy vibration (elevator, construction) | IMU (saturation), camera (motion blur), LiDAR (scan distortion) | Vibration isolation mounting |
| Complete darkness | Camera (RGB useless) | LiDAR, radar, ultrasonic unaffected |
| Glass environment | LiDAR (passes through), camera depth (IR passes through) | Ultrasonic + radar unaffected by glass |
| EMI/RF interference | Radar, electronics | Shielding, frequency hopping |

> **Design principle:** Choose sensors with **uncorrelated failure modes**. If all sensors fail for the same reason, fusion cannot help.

---

## 8.5 — The Complete MediBot Sensor Architecture

**Summary table:**

| Sensor | Primary role | Complements | Backup for |
|--------|-------------|------------|-----------|
| Wheel encoders | Motion prediction (high rate) | IMU | — |
| IMU | Orientation, short-term dead reckoning | Encoders | LiDAR (between scans), camera |
| LiDAR | Localization, mapping, obstacle avoidance | Camera (semantics) | Ultrasonic (range) |
| Ultrasonic | Close-range detection, **glass detection** | LiDAR | Camera depth (at short range) |
| Camera (RGB-D) | Semantic understanding, sign reading, dense depth | LiDAR (geometry) | Visual odometry if LiDAR fails |
| Radar | Moving object detection, velocity, all-weather | Camera | LiDAR (in glass/dust), IMU (velocity) |

---
---

# FINAL INTEGRATION CHALLENGE: Last-Mile Delivery Robot

---

## Challenge Description

You are the engineering team designing the sensor suite for **"StreetBot"** — a last-mile delivery robot that navigates city sidewalks, crosses streets, enters building lobbies, rides elevators, and delivers packages to apartment doors.

### Environment challenges (harder than hospital):

| Challenge | Why it's harder than hospital |
|-----------|------------------------------|
| Rain, snow, fog | Camera, LiDAR degraded; radar essential |
| Direct sunlight | Camera depth saturated; LiDAR unaffected |
| Uneven terrain (curbs, cobblestones) | Wheel slip worse, IMU vibration |
| Moving traffic (cars, cyclists, pedestrians) | Need velocity estimation at long range |
| Glass storefronts | Same as hospital glass, but everywhere |
| Indoor → outdoor transitions | Lighting changes, GPS availability changes |
| No controlled environment | Temperature swings affect ultrasonic, IMU drift |

### Your deliverables

**1. Sensor selection & placement**
- Which sensors? How many of each? Where mounted?
- Justify each choice based on what you learned in Blocks 1–8

**2. Sensor models & noise parameters**
- For each sensor, write the mathematical sensor model
- Specify expected noise parameters (from datasheets or estimation)
- Identify heteroscedastic noise sources

**3. Fusion architecture**
- Centralized or decentralized? Justify your choice
- Draw the data flow diagram: which sensors feed which filters
- How do you handle multi-rate data?

**4. Failure analysis**
- Identify the top 5 failure scenarios (e.g., "heavy rain + glass storefront")
- For each: which sensors fail? Which survive? Can StreetBot still navigate safely?
- Design the graceful degradation strategy

**5. Comparison with MediBot**
- What transfers directly from the hospital robot design?
- What must be changed? Why?
- What NEW challenges does the outdoor environment introduce?

---

## Grading Criteria

| Criterion | Weight | What we're looking for |
|-----------|--------|----------------------|
| Sensor selection justification | 20% | Every sensor has a clear reason based on the environment analysis |
| Mathematical models | 25% | Correct sensor models with appropriate noise characterization |
| Fusion architecture | 25% | Logical data flow, handles multi-rate data, appropriate architecture choice |
| Failure analysis | 20% | Realistic failure scenarios, correct identification of affected sensors, viable mitigation |
| Transfer analysis | 10% | Clear articulation of what changes and why |

---

# References

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. — Chapters 5–7 (sensor models, beam model, likelihood field, odometry model)
2. Siciliano, B. & Khatib, O. (2016). *Springer Handbook of Robotics*, 2nd ed. — Chapter 4 (sensing and estimation)
3. Siegwart, R., Nourbakhsh, I.R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press. — Chapters 4–5 (perception, feature extraction)
4. Hartley, R. & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision*, 2nd ed. Cambridge University Press. — Camera model, calibration (Chapters 6–7)
5. Titterton, D.H. & Weston, J.L. (2004). *Strapdown Inertial Navigation Technology*, 2nd ed. IET. — IMU models, error analysis, Allan Variance
6. Skog, I. & Händel, P. (2011). "In-car positioning and navigation technologies—A survey." *IEEE Trans. ITS*, 10(1), 4–21. — Sensor fusion architectures
7. IEEE Std 952-1997. *IEEE Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros* — Allan Variance methodology
8. Zhang, Z. (2000). "A flexible new technique for camera calibration." *IEEE TPAMI*, 22(11), 1330–1334. — Camera calibration (Zhang's method)
9. Texas Instruments (2023). *mmWave Sensor Fundamentals*. Application Note SWRA553. — FMCW radar principles
10. Pepperl+Fuchs (2022). "AMR Glass Detection with Ultrasonic Sensors." Technical Blog. — Ultrasonic glass detection in AMR applications

---

*MOD-004 Student Script v1.0 — Course Architect Engine*  
*Mobile Robots Course — Sensor Modeling, Measurement & Fusion*
