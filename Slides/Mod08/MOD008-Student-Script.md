# MOD-09: Mapping & SLAM

## Student Script — Full Mathematical Foundations

**Course:** Mobile Robots  
**Instructor:** Dr. Ahmad Abbadi  
**Prerequisites:** MOD-05 (KF, Bayes), MOD-06 (EKF, MCL), MOD-07 (Features)  
**Key References:**

- **[PR]** Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Ch. 9–13.
- **[AMR]** Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press. Ch. 5.8.
- **[GS]** Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010). A Tutorial on Graph-Based SLAM. *IEEE Intelligent Transportation Systems*, 2(4).

---

## A. The SLAM Problem

### A.1 Simultaneous Localization and Mapping

SLAM estimates both the robot trajectory and the map simultaneously [PR, §10.1]:

$$p(\mathbf{x}_{1:t}, m \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})$$

Where x_{1:t} = sequence of robot poses, m = map, z_{1:t} = sensor readings, u_{1:t} = control inputs.

**The chicken-and-egg problem:** Localization (MOD-06) requires a map. Mapping requires knowing the pose. SLAM solves both jointly.

### A.2 Online SLAM vs. Full SLAM

**Online SLAM** — estimate only the current pose + map (discard old poses):

$$p(\mathbf{x}_t, m \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}) = \int \cdots \int p(\mathbf{x}_{1:t}, m \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})\, d\mathbf{x}_1 \cdots d\mathbf{x}_{t-1}$$

Used by: EKF-SLAM, FastSLAM (incremental processing).

**Full SLAM** — estimate the entire trajectory + map:

$$p(\mathbf{x}_{1:t}, m \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})$$

Used by: Graph-based SLAM (batch optimization).

---

## B. Occupancy Grid Mapping

### B.1 Log-Odds Update

Given known poses, the occupancy probability of each cell c is updated using log-odds [PR, §9.2]:

$$l_t(c) = l_{t-1}(c) + \log\frac{p(c \mid z_t, x_t)}{1 - p(c \mid z_t, x_t)} - l_0$$

Where l(c) = log(p(c)/(1−p(c))) is the log-odds representation and l₀ = log(p₀/(1−p₀)) is the prior (typically l₀ = 0 for p₀ = 0.5).

**Advantages of log-odds:** Simple addition instead of multiplication (no underflow). Easy to clamp: l ∈ [−L_max, +L_max] prevents overconfidence.

**Convert back:** p(c) = 1/(1 + exp(−l(c))).

### B.2 Inverse Sensor Model

For a LiDAR beam at angle φ with measured range r [PR, §9.2]:
- Cells along the beam from sensor to range r: mark **free** (log-odds decrease)
- Cell at range r: mark **occupied** (log-odds increase)
- Cells beyond r: no update (unknown)

---

## C. Map Representations

| Type | Dimensions | Best For | Memory | SLAM Algorithm |
|------|-----------|----------|--------|----------------|
| **Occupancy Grid** | 2D | Indoor navigation, planning | O(area/resolution²) | GMapping, slam_toolbox |
| **Feature Map** | 2D/3D | Structured environments | O(N landmarks) | EKF-SLAM |
| **Topological** | Abstract | Large-scale planning | O(N nodes + edges) | Place recognition |
| **OctoMap** | 3D (octree) | Aerial robots, manipulation | O(occupied volume) | 3D LiDAR SLAM |
| **Point Cloud** | 3D | Dense reconstruction | O(N points) — large | LiDAR SLAM |
| **Semantic** | 2D/3D + labels | Task planning, HRI | O(geometry + labels) | Semantic SLAM |

---

## D. Data Association

### D.1 The Problem

When the robot observes a landmark, it must determine: **is this a previously seen landmark (update) or a new landmark (add)?** [PR, §10.2]

### D.2 Nearest Neighbor with Mahalanobis Gating

For each observation z, compute the **Mahalanobis distance** to each existing landmark j:

$$d_M^{(j)} = \sqrt{(\mathbf{z} - \hat{\mathbf{z}}_j)^T \mathbf{S}_j^{-1} (\mathbf{z} - \hat{\mathbf{z}}_j)}$$

Where ẑ_j = predicted observation if robot sees landmark j, and S_j = H_j P⁻ H_jᵀ + R is the innovation covariance.

**Decision rule:**
- If min_j d_M^{(j)} < threshold (e.g., 3.0): associate with that landmark (update)
- If all d_M^{(j)} > threshold: it's a **new landmark** (add to map)

### D.3 Maximum Likelihood Data Association

Compute the likelihood of each assignment [PR, §10.2.2]:

$$j^* = \arg\max_j\; p(\mathbf{z} \mid \text{landmark } j, \hat{\mathbf{x}}_t)$$

For Gaussian observations: maximizing likelihood is equivalent to minimizing Mahalanobis distance. For multiple simultaneous observations, the joint likelihood across all assignments should be maximized — this is combinatorial in the worst case.

---

## E. EKF-SLAM

### E.1 Augmented State Vector

The state vector includes both the robot pose and all landmark positions [PR, §10.3]:

$$\mathbf{x} = \begin{bmatrix} x_r \\ y_r \\ \theta_r \\ l_{1,x} \\ l_{1,y} \\ l_{2,x} \\ l_{2,y} \\ \vdots \\ l_{N,x} \\ l_{N,y} \end{bmatrix} \in \mathbb{R}^{3+2N}$$

The covariance matrix P is (3+2N) × (3+2N) with block structure:

$$\mathbf{P} = \begin{bmatrix} \mathbf{P}_{rr} & \mathbf{P}_{rL_1} & \mathbf{P}_{rL_2} & \cdots \\ \mathbf{P}_{L_1 r} & \mathbf{P}_{L_1 L_1} & \mathbf{P}_{L_1 L_2} & \cdots \\ \mathbf{P}_{L_2 r} & \mathbf{P}_{L_2 L_1} & \mathbf{P}_{L_2 L_2} & \cdots \\ \vdots & \vdots & \vdots & \ddots \end{bmatrix}$$

**Critical property:** The off-diagonal blocks P_{L_i L_j} represent **cross-correlations** between landmarks through the robot's uncertain path. These are what make SLAM fundamentally different from independent landmark tracking.

### E.2 Prediction

Only the robot pose changes during motion (landmarks are static) [PR, §10.3.1]:

$$\hat{\mathbf{x}}^- = \begin{bmatrix} f(\hat{\mathbf{x}}_r, \mathbf{u}) \\ \hat{\mathbf{l}}_1 \\ \vdots \\ \hat{\mathbf{l}}_N \end{bmatrix}$$

$$\mathbf{P}^- = \mathbf{F}_x \mathbf{P} \mathbf{F}_x^T + \mathbf{F}_u \mathbf{Q} \mathbf{F}_u^T$$

Where F_x is (3+2N) × (3+2N) — identity everywhere except the top-left 3×3 robot block which contains the motion model Jacobian.

**Cost:** O(N) — only the robot rows/columns of P change.

### E.3 Update (Observing Landmark j)

Observation model: z = h(x_r, l_j) + v, where h computes range and bearing from robot to landmark j.

The Jacobian H is (2) × (3+2N) but **sparse** — only the robot columns (3) and landmark j columns (2) are nonzero:

$$\mathbf{H} = \begin{bmatrix} \frac{\partial h}{\partial \mathbf{x}_r} & \mathbf{0} & \cdots & \frac{\partial h}{\partial \mathbf{l}_j} & \cdots & \mathbf{0} \end{bmatrix}$$

Kalman gain, state update, and covariance update follow the standard EKF equations from MOD-06.

**Cost:** O(N²) per update — the Kalman gain K is (3+2N) × 2 and P is (3+2N) × (3+2N). This O(N²) scaling is the fundamental limitation of EKF-SLAM.

### E.4 Adding a New Landmark

When data association determines that observation z corresponds to a new landmark, the state vector grows:

$$\hat{\mathbf{l}}_{\text{new}} = g(\hat{\mathbf{x}}_r, \mathbf{z}) = \begin{bmatrix} \hat{x}_r + r\cos(\hat{\theta}_r + \phi) \\ \hat{y}_r + r\sin(\hat{\theta}_r + \phi) \end{bmatrix}$$

Where (r, φ) is the observed range and bearing. The covariance matrix P grows by 2 rows and 2 columns, initialized using the Jacobian of g and the current robot uncertainty.

### E.5 The Convergence Property

A remarkable property of EKF-SLAM [PR, §10.3.3]: as the robot re-observes landmarks, the **determinant of P monotonically decreases**. Every observation adds information. Re-observing landmark L₁ not only improves L₁ but also improves **all other landmarks** through their cross-correlations. This is why SLAM converges.

---

## F. FastSLAM

### F.1 Rao-Blackwellized Factorization

The key insight of FastSLAM [Montemerlo et al., 2002; PR, §13.2]:

$$p(\mathbf{x}_{1:t}, m \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}) = p(\mathbf{x}_{1:t} \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}) \prod_{i=1}^{N} p(\mathbf{l}_i \mid \mathbf{x}_{1:t}, \mathbf{z}_{1:t})$$

**If the trajectory is known**, landmarks are conditionally independent! No cross-correlations needed.

**FastSLAM exploits this:** Use particles for the trajectory (handles nonlinearity, multi-modality) and independent EKFs for each landmark conditioned on each particle's trajectory.

### F.2 Algorithm [PR, Algorithm 13.1]

Each particle m maintains:
- A pose hypothesis x_t^[m]
- N independent landmark EKFs: {μ_i^[m], Σ_i^[m]} for i = 1...N, each 2×2

```
FASTSLAM(X_{t-1}, u_t, z_t):
  for each particle m:
    1. SAMPLE: x_t^[m] ~ p(x_t | u_t, x_{t-1}^[m])
    2. For each observed landmark j:
       a. If known: UPDATE EKF(μ_j^[m], Σ_j^[m]) using (z_t, x_t^[m])
       b. If new: INITIALIZE μ_j^[m] = g(x_t^[m], z_t), Σ_j^[m] = large
    3. WEIGHT: w^[m] = p(z_t | x_t^[m], μ_j^[m], Σ_j^[m])
  
  RESAMPLE particles proportional to weights
  Return X_t
```

**Cost per step:** O(M × N) where M = particles, N = landmarks observed. Compare to EKF-SLAM's O(N²).

### F.3 Key Properties

- **Handles multi-modal trajectory hypotheses** (via particles)
- **Data association can differ per particle** — each particle independently decides which landmark it's seeing. If one particle makes the wrong association, it gets a low weight and dies during resampling.
- **Grid-based variant (GMapping):** Each particle carries an occupancy grid instead of landmarks. The standard 2D SLAM in early ROS.

---

## G. Graph-Based SLAM

### G.1 Pose Graph Formulation

Model SLAM as a graph optimization problem [GS]:

- **Nodes:** Robot poses x₁, x₂, ..., xₜ (and optionally landmark positions)
- **Edges:** Constraints between poses:
  - **Odometry edge** (xᵢ, x_{i+1}): motion constraint from odometry
  - **Observation edge** (xᵢ, lⱼ): landmark observation from pose i
  - **Loop closure edge** (xᵢ, xⱼ): recognition that poses i and j observed the same place

### G.2 Optimization

Find poses that minimize the total weighted error across all edges [GS]:

$$\mathbf{x}^* = \arg\min_{\mathbf{x}} \sum_{(i,j) \in \text{edges}} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)^T \boldsymbol{\Omega}_{ij}\; \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)$$

Where:
- **e_{ij}**(x_i, x_j) = error function: how much the constraint disagrees with current pose estimates
- **Ω_{ij}** = information matrix (inverse covariance) of the constraint

**For odometry edge:** e_{ij} = (x_j − f(x_i, u_{ij})) — how much the actual pose difference deviates from the odometry prediction.

**For loop closure edge:** e_{ij} = (x_j − x_i − z_{ij}) — how much the relative pose differs from the loop closure observation.

### G.3 Gauss-Newton Solution

Linearize the error functions and solve iteratively [GS]:

$$\mathbf{H} \Delta\mathbf{x} = -\mathbf{b}$$

Where:

$$\mathbf{H} = \sum_{(i,j)} \mathbf{J}_{ij}^T \boldsymbol{\Omega}_{ij} \mathbf{J}_{ij}, \qquad \mathbf{b} = \sum_{(i,j)} \mathbf{J}_{ij}^T \boldsymbol{\Omega}_{ij} \mathbf{e}_{ij}$$

J_{ij} = Jacobian of e_{ij} with respect to x.

**Key property:** H is **sparse** because each edge connects only 2 nodes → most entries are zero. Sparse Cholesky factorization solves the system in O(N^{1.5}) for planar graphs.

**Iterate:** x ← x + Δx until convergence (typically 3–5 iterations).

### G.4 Loop Closure: The Key Ingredient

Without loop closure, Graph-SLAM reduces to simple odometry chaining (drift accumulates). **Loop closure** adds a constraint between distant poses → the optimizer **distributes the accumulated error** across the entire trajectory, producing a globally consistent map.

**Detection methods:**
- **Visual:** Bag-of-words (DBoW2) — describe each image as a set of visual words, compare to database [Gálvez-López & Tardós, 2012]
- **LiDAR:** Scan matching (ICP) — align current scan to previous scans, accept if alignment error is low
- **Verification:** Always verify geometrically (RANSAC) — false loop closures are catastrophic

---

## H. Visual SLAM (Concept)

### H.1 Architecture (ORB-SLAM3)

Three parallel threads [Campos et al., 2021]:

1. **Tracking (30 Hz):** Extract ORB features → match to local map → estimate camera pose
2. **Local mapping (background):** Process keyframes → triangulate new 3D points → local bundle adjustment
3. **Loop closing (background):** DBoW2 place recognition → compute similarity transform → pose graph optimization

### H.2 Feature-Based vs. Direct

**Feature-based** (ORB-SLAM, VINS): Extract sparse features, minimize reprojection error. Robust to lighting changes, efficient.

**Direct** (LSD-SLAM, DSO): Use raw pixel intensities, minimize photometric error. Dense/semi-dense maps, works on textureless surfaces, but sensitive to lighting.

**VIO (Visual-Inertial Odometry):** Tightly couple camera + IMU. IMU provides motion prior between frames; camera corrects drift at frame rate. Standard on drones and AR headsets.

---

## I. ROS 2 SLAM Tools

| Tool | Type | Sensor | Map Output | Best For |
|------|------|--------|------------|----------|
| **slam_toolbox** | Graph-based | 2D LiDAR | OccupancyGrid | Nav2 default, indoor |
| **Cartographer** | Graph-based (submaps) | 2D/3D LiDAR | OccupancyGrid | Multi-floor, large |
| **RTAB-Map** | Graph-based (visual) | RGB-D / stereo / LiDAR | OccupancyGrid + 3D cloud | Visual SLAM in ROS |

---

## J. New Frontiers (Concept Previews)

**LiDAR-Inertial SLAM (LIO-SAM, FAST-LIO2):** Tightly couple LiDAR + IMU. IMU predicts between scans; LiDAR corrects via scan matching. Production-ready for autonomous vehicles and drones.

**Semantic SLAM:** Add semantic labels to the map during SLAM. Objects as landmarks instead of geometric features. Enables natural language navigation ("go to the red door").

**Multi-Robot SLAM:** Multiple robots map simultaneously. Distributed map building + inter-robot loop closure for map merging.

**Neural Radiance Fields (NeRF) & 3D Gaussian Splatting:** Represent scenes as neural networks (NeRF) or explicit 3D Gaussians (splatting). Photorealistic rendering. Being integrated into SLAM pipelines for dense 3D reconstruction.

---

## Summary of Key Equations

| Algorithm | Key Equation | Complexity |
|-----------|-------------|------------|
| Occupancy grid | l_t(c) = l_{t-1}(c) + log-odds(z_t) − l₀ | O(cells per beam) |
| EKF-SLAM state | x = [x_r, y_r, θ, l₁_x, l₁_y, ..., lN_x, lN_y]ᵀ | O(N²) per update |
| EKF-SLAM predict | P⁻ = F·P·Fᵀ + Q (only robot block changes) | O(N) |
| FastSLAM factor | p(x,m\|z,u) = p(x\|z,u) ∏ p(lᵢ\|x,z) | O(M×N) |
| Graph-SLAM | x* = argmin Σ eᵢⱼᵀ Ωᵢⱼ eᵢⱼ | O(N^1.5) sparse |
| Mahalanobis (data assoc.) | d²_M = (z−ẑ)ᵀ S⁻¹ (z−ẑ) | O(1) per landmark |

---

## References

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Ch. 9–13.
2. Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010). A Tutorial on Graph-Based SLAM. *IEEE ITS Magazine*, 2(4).
3. Montemerlo, M., Thrun, S., Koller, D., & Wegbreit, B. (2002). FastSLAM: A Factored Solution to the SLAM Problem. *AAAI*.
4. Campos, C., Elvira, R., Rodríguez, J.J.G., Montiel, J.M.M., & Tardós, J.D. (2021). ORB-SLAM3. *IEEE T-RO*, 37(6).
5. Gálvez-López, D. & Tardós, J.D. (2012). Bags of Binary Words for Fast Place Recognition. *IEEE T-RO*, 28(5).
6. Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press. Ch. 5.8.
7. Xu, W. et al. (2022). FAST-LIO2: Fast Direct LiDAR-Inertial Odometry. *IEEE T-RO*, 38(4).
8. Macenski, S. & Jambrecic, I. (2021). SLAM Toolbox: SLAM for the Dynamic World. *JOSS*, 6(61).
