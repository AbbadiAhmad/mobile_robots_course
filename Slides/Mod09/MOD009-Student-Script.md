# MOD-08: Planning & Navigation

## Student Script — Full Mathematical Foundations

**Course:** Mobile Robots  
**Instructor:** Dr. Ahmad Abbadi  
**Prerequisites:** MOD-03 (Kinematics), MOD-07 (Perception — obstacle maps)  
**Key References:**

- **[LA]** LaValle, S. (2006). *Planning Algorithms*. Cambridge University Press. (Free online)
- **[AMR]** Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press. Ch. 6.
- **[PR]** Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Ch. 6 (sensor model for DWA).

---

## A. The Planning Problem

### A.1 Problem Statement

Given a map with obstacles, a start configuration, and a goal configuration, find a collision-free path [LA, §1.1; AMR, §6.1]:

$$\text{Find: } \tau: [0, 1] \to \mathcal{C}_{\text{free}}, \quad \tau(0) = q_{\text{start}}, \quad \tau(1) = q_{\text{goal}}$$

Where C_free is the obstacle-free subset of the configuration space.

### A.2 Global vs. Local Planning

Real robots run **both simultaneously** [AMR, §6.3]:

- **Global planner** (1–5 Hz): Plans the overall route on the full map. Algorithms: A*, RRT, Dijkstra.
- **Local planner** (10–20 Hz): Reacts to immediate obstacles. Algorithms: DWA, TEB.

---

## B. Configuration Space (C-Space)

### B.1 Definition

The **configuration space** C is the space of all possible robot configurations q [LA, §3.1; AMR, §6.2.1]:

- Point robot: q = (x, y) → C is 2D
- Circular robot (radius r): q = (x, y), but obstacles are grown by r → robot treated as a point in the grown C-space
- Non-circular robot: q = (x, y, θ) → C is 3D (each orientation produces a different C-space obstacle shape)

**C-space obstacle** C_obs: the set of configurations where the robot collides with a workspace obstacle. For a circular robot with radius r, C_obs is the workspace obstacle **Minkowski-summed** with a disk of radius r [LA, §3.2]:

$$\mathcal{C}_{\text{obs}} = \mathcal{W}_{\text{obs}} \oplus B(r)$$

This effectively grows each obstacle boundary outward by r, and the robot becomes a point.

---

## C. Potential Fields

### C.1 Attractive + Repulsive Potentials

Define a scalar potential function over C-space [AMR, §6.2.4; Khatib, 1986]:

$$U(\mathbf{q}) = U_{\text{att}}(\mathbf{q}) + U_{\text{rep}}(\mathbf{q})$$

**Attractive potential** (pulls toward goal):

$$U_{\text{att}}(\mathbf{q}) = \frac{1}{2} k_a \|\mathbf{q} - \mathbf{q}_{\text{goal}}\|^2$$

**Repulsive potential** (pushes away from obstacles, active only within distance d₀):

$$U_{\text{rep}}(\mathbf{q}) = \begin{cases} \frac{1}{2} k_r \left(\frac{1}{d(\mathbf{q})} - \frac{1}{d_0}\right)^2 & \text{if } d(\mathbf{q}) \leq d_0 \\ 0 & \text{if } d(\mathbf{q}) > d_0 \end{cases}$$

Where d(**q**) = distance to nearest obstacle, k_a, k_r are gain constants, d₀ is the influence distance.

**Robot velocity:** Follow the negative gradient: **v** = −α ∇U(**q**).

### C.2 Local Minima Problem

The **fatal flaw** of potential fields: local minima where ∇U = 0 but q ≠ q_goal. Example: U-shaped obstacle creates a concavity where attractive and repulsive forces cancel. The robot gets permanently stuck. No local information can resolve this — motivating **global planners** that consider the entire map.

---

## D. Dijkstra's Algorithm

### D.1 Algorithm

Dijkstra finds the shortest path on a weighted graph [LA, §2.2; AMR, §6.2.2]:

```
DIJKSTRA(start, goal, graph):
  cost[start] = 0, cost[all others] = ∞
  parent[start] = None
  Q = priority queue with (start, 0)
  
  while Q not empty:
    current = Q.pop_min()
    if current == goal: return RECONSTRUCT(parent, goal)
    for each neighbor n of current:
      new_cost = cost[current] + edge_weight(current, n)
      if new_cost < cost[n]:
        cost[n] = new_cost
        parent[n] = current
        Q.push(n, new_cost)
  
  return FAILURE
```

**Complexity:** O((V + E) log V) with a binary heap, where V = vertices, E = edges.

**Properties:** Optimal (finds shortest path). Complete (finds a path if one exists). Explores in **all directions** equally — inefficient when the goal direction is known.

---

## E. A* Algorithm

### E.1 Key Insight: Add a Heuristic

A* extends Dijkstra with a heuristic h(n) that estimates the remaining cost to the goal [Hart, Nilsson & Raphael, 1968; LA, §2.2]:

$$f(n) = g(n) + h(n)$$

- g(n) = actual cost from start to n (same as Dijkstra)
- h(n) = estimated cost from n to goal (**new**)
- f(n) = estimated total cost through n → used as priority

### E.2 Admissibility

A heuristic is **admissible** if it never overestimates the true cost [LA, §2.2.1]:

$$h(n) \leq h^*(n) \quad \forall n$$

Where h*(n) is the true optimal cost from n to goal. If h is admissible, A* is **guaranteed optimal**.

**Common admissible heuristics for 2D grids:**

| Heuristic | Formula | Grid type |
|-----------|---------|-----------|
| Euclidean | h = √((x−g_x)² + (y−g_y)²) | Any |
| Manhattan | h = \|x−g_x\| + \|y−g_y\| | 4-connected |
| Diagonal | max(\|Δx\|, \|Δy\|) + (√2−1)·min(\|Δx\|, \|Δy\|) | 8-connected |

### E.3 A* Pseudocode

Identical to Dijkstra except the priority is f = g + h instead of g alone:

```
A_STAR(start, goal, graph):
  g[start] = 0, f[start] = h(start, goal)
  Q = priority queue with (start, f[start])
  
  while Q not empty:
    current = Q.pop_min()       // lowest f = g + h
    if current == goal: return RECONSTRUCT(parent, goal)
    for each neighbor n of current:
      new_g = g[current] + edge_weight(current, n)
      if new_g < g[n]:
        g[n] = new_g
        f[n] = new_g + h(n, goal)    // ← THE DIFFERENCE
        parent[n] = current
        Q.push(n, f[n])
  return FAILURE
```

A* typically explores 5–50× fewer nodes than Dijkstra on typical robot maps.

---

## F. RRT Family

### F.1 Rapidly-Exploring Random Trees (RRT)

RRT explores high-dimensional C-spaces by random sampling [LaValle & Kuffner, 2001; LA, §5.5]:

```
RRT(start, goal, C_free):
  T = {start}
  for i = 1 to N:
    x_rand = random sample from C
    x_near = nearest node in T to x_rand
    x_new = STEER(x_near, x_rand, δ)
    if COLLISION_FREE(x_near, x_new):
      T.add(x_new, parent=x_near)
      if ||x_new − goal|| < ε: return PATH(T, x_new)
  return FAILURE
```

**STEER** moves from x_near toward x_rand by step size δ:

$$\mathbf{x}_{\text{new}} = \mathbf{x}_{\text{near}} + \delta \cdot \frac{\mathbf{x}_{\text{rand}} - \mathbf{x}_{\text{near}}}{\|\mathbf{x}_{\text{rand}} - \mathbf{x}_{\text{near}}\|}$$

**Properties:** Probabilistically complete but NOT optimal (paths are jagged).

### F.2 RRT* (Asymptotically Optimal)

RRT* adds two operations [Karaman & Frazzoli, 2011]:

1. **Choose-Parent:** Before adding x_new, check all nodes within radius r_n = γ(log n / n)^{1/d}. Connect x_new to the parent that yields the lowest total cost from start.

2. **Rewire:** After adding x_new, check if nearby nodes would have lower cost routing through x_new. If so, change their parent to x_new.

**Result:** Asymptotically optimal — as n → ∞, the path cost converges to the true optimal.

### F.3 RRT-Connect

Grow two trees simultaneously — one from start, one from goal [Kuffner & LaValle, 2000]. Each iteration: grow one tree toward a random sample, then greedily extend the other tree toward the new node. If trees connect → path found. Much faster than single-tree RRT.

---

## G. Kinematic Feasibility

### G.1 Holonomic vs. Non-Holonomic

A **non-holonomic constraint** restricts the robot's velocity but not its position [LA, §13.1; AMR, §3.3]:

$$\dot{x}\sin\theta - \dot{y}\cos\theta = 0 \qquad \text{(Pfaffian form: } \mathbf{A}(\mathbf{q})\dot{\mathbf{q}} = 0\text{)}$$

This means: the velocity must be aligned with the heading. The robot **cannot slide sideways**.

For a diff-drive robot: the minimum turning radius is r_min = v/ω_max. Any planned path must have curvature κ ≤ 1/r_min everywhere.

### G.2 Kinematic RRT

Standard RRT extends in straight lines — infeasible for non-holonomic robots. **Kinematic RRT** samples control inputs (v, ω) instead [LA, §14.3]:

```
x_new = SIMULATE(x_near, v, ω, Δt):
  x' = x + v·cos(θ)·Δt
  y' = y + v·sin(θ)·Δt
  θ' = θ + ω·Δt
  return (x', y', θ')
```

The resulting path segments are arcs, not lines → always feasible.

**Alternative (post-processing):** Plan with A* (ignoring kinematics) → smooth with Dubins curves or cubic splines → verify feasibility.

---

## H. Costmaps

### H.1 From Binary to Continuous Costs

A **costmap** assigns a continuous cost c ∈ [0, 254] to each cell [AMR, §6.3.1]:

| Cost | Meaning |
|------|---------|
| 0 | Free space |
| 1–252 | Inflation zone (cost decays with distance from obstacle) |
| 253 | Inscribed radius (robot footprint touches obstacle) |
| 254 | Lethal (inside obstacle) |

**Inflation formula:** c(d) = 253 · exp(−k · (d − r_inscribed)), where d = distance to nearest lethal cell, k = cost scaling factor.

### H.2 Three Layers

1. **Static layer:** From pre-built map (loaded once)
2. **Obstacle layer:** From live sensors (updates at sensor rate)
3. **Inflation layer:** Computed from lethal cells (adds cost gradient)

Layers are stacked: combined = max(static, obstacle, inflation) per cell.

---

## I. Dynamic Window Approach (DWA)

### I.1 Algorithm

DWA searches the velocity space (v, ω) for the best command to execute immediately [Fox, Burgard & Thrun, 1997; AMR, §6.3.4]:

**Step 1 — Define the dynamic window** (reachable velocities):

$$v \in [v_c - \dot{v}_{\max}\Delta t,\; v_c + \dot{v}_{\max}\Delta t]$$
$$\omega \in [\omega_c - \dot{\omega}_{\max}\Delta t,\; \omega_c + \dot{\omega}_{\max}\Delta t]$$

Where v_c, ω_c are current velocities and v̇_max, ω̇_max are acceleration limits.

**Step 2 — Simulate trajectories:** For each candidate (v, ω) in the window, simulate forward for T seconds using the diff-drive model.

**Step 3 — Score each trajectory:**

$$\text{score}(v, \omega) = \alpha \cdot \text{heading}(v, \omega) + \beta \cdot \text{clearance}(v, \omega) + \gamma \cdot \text{velocity}(v, \omega)$$

- **heading:** angular difference to goal (prefer facing goal)
- **clearance:** distance to nearest obstacle along trajectory (prefer far from obstacles)
- **velocity:** forward speed (prefer faster progress)
- α, β, γ: tunable weights

**Step 4 — Execute:** Send the (v, ω) with highest score to the motors.

### I.2 Properties

DWA naturally respects kinematic constraints (only considers reachable velocities). Runs at 10–20 Hz for real-time obstacle avoidance. Limitations: short planning horizon (1–2 seconds) → can get stuck in local minima → needs global planner to guide it.

---

## J. Path Smoothing with Cubic Splines

Grid planners produce jagged, grid-aligned paths. **Cubic spline interpolation** produces smooth, feasible paths through waypoints [LA, §14.5]:

Between consecutive waypoints p_i and p_{i+1}, the spline is:

$$S_i(t) = a_i + b_i t + c_i t^2 + d_i t^3, \quad t \in [0, 1]$$

With boundary conditions ensuring C² continuity (continuous position, velocity, acceleration):
- S_i(1) = S_{i+1}(0) (positions match)
- S'_i(1) = S'_{i+1}(0) (velocities match)
- S''_i(1) = S''_{i+1}(0) (accelerations match)

Result: smooth curves with bounded curvature κ = S''/((1 + S'²)^{3/2}).

---

## K. Nav2 Architecture

Nav2 is the standard ROS 2 navigation framework. It integrates all planning components:

- **BT Navigator:** Behavior tree that orchestrates plan-follow-recover cycle
- **Planner Server:** Global planner (NavFn/A*, Smac, ThetaStar)
- **Controller Server:** Local planner (DWA, TEB, RegulatedPurePursuit)
- **Costmap Server:** Global costmap (full map) + local costmap (sensor window)
- **Recovery Server:** Spin, backup, wait, clear costmap
- **Map Server:** Loads and serves the occupancy grid

Behavior tree node types: **Sequence** (execute children left-to-right, fail if any fails), **Fallback** (try children left-to-right, succeed if any succeeds), **Action** (leaf — calls a server).

---

## L. Dynamic Replanning

When the environment changes (obstacle appears on global path), the system must detect and replan:

1. **Detect:** Controller reports path blocked (cannot follow)
2. **Recover:** Try recovery behaviors (spin, backup, clear costmap)
3. **Replan:** If recovery fails, request new global plan on updated costmap
4. **Resume:** Follow new path

The **sense-plan-act-replan** cycle runs continuously at different rates: sensing at 10–30 Hz, local planning at 10–20 Hz, global replanning at 1–5 Hz or on-demand.

**Temporal costmaps:** Dynamic obstacles fade after a configurable time (observation_persistence parameter). If the obstacle is no longer observed, it disappears from the costmap, freeing the path.

---

## References

1. LaValle, S. (2006). *Planning Algorithms*. Cambridge University Press.
2. Hart, P.E., Nilsson, N.J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. *IEEE Trans. SSC*, 4(2).
3. LaValle, S. & Kuffner, J. (2001). Rapidly-Exploring Random Trees: Progress and Prospects. *ISRR*.
4. Karaman, S. & Frazzoli, E. (2011). Sampling-based Algorithms for Optimal Motion Planning. *IJRR*, 30(7).
5. Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance. *IEEE RAM*, 4(1).
6. Khatib, O. (1986). Real-Time Obstacle Avoidance for Manipulators and Mobile Robots. *IJRR*, 5(1).
7. Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press. Ch. 6.
8. Macenski, S. et al. (2023). The Marathon 2: A Navigation System. *IROS*.
