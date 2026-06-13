# Publishing Proposal — *Motion Planning, Motion Control and Systems Engineering*
### A Tiered, Cross-Domain Book Series on Motion Planning, Control, Estimation, and Integration

---

## 1. Positioning and Rationale

The market currently teaches motion-related disciplines in isolation: robotics, vehicle dynamics, drones, and control theory each have strong standalone literature. Industry, however, increasingly needs engineers who understand the **shared motion stack** and can move between domains with minimal re-learning.

This series is built around a single organizing idea:

> **Most of what a motion engineer needs across vehicles, mobile robots, drones, manipulators, and humanoids is the same mathematical and engineering core. Domain-specific knowledge is a comparatively thin layer on top of that core.**

The pipeline that unifies every motion system is:

```
sensing → estimation → world model → planning → trajectory generation →
control → actuation → safety → embedded execution
```

The series teaches this pipeline once, at a practical (employable) depth, and then branches only where domains genuinely differ.

Two further principles shape the structure:

- **Reference, do not repeat.** Several layers are already covered thoroughly by established texts (probabilistic estimation and SLAM, sampling- and search-based planning, classical mobile-robot kinematics). For these, the series provides a working overview, explains the concepts in the context of the full stack, and points readers to the canonical source for depth. The series adds value precisely where existing books are weak: **integration, dynamic feasibility, real-system constraints, production safety, and cross-domain abstraction.**
- **Big picture first, detail on demand.** Each book introduces the full conceptual landscape — including advanced methods — before drilling into the parts that carry the most industrial value. Deep mathematical and research-grade treatment is deferred to a clearly separated advanced tier.

This reduces the previously proposed sixteen-volume concept to a focused core of **four books**, plus two optional companion tiers.

The system engineering part will be included later. 

---

## 2. Reading Conventions Used in This Proposal

Each chapter is tagged with an intended **depth** and **criticality**, so the publishing plan can balance page budget against value.

**Depth** — how far the chapter goes:
- **Overview (O):** concepts, vocabulary, and when/why to use a method; full treatment deferred to a referenced text.
- **Working (W):** enough to implement standard methods, tune them, and integrate them. This is the default target depth for the core series.
- **Deep (D):** derivations, edge cases, and extensions; reserved for high-value or differentiating chapters.

**Criticality** — value to a working engineer:
- **Critical:** core employability; expected of any motion engineer.
- **Important:** a strong differentiator between average and senior engineers.
- **Specialist:** relevant to a specific domain or role.

**Anchor:** the established reference the chapter builds on rather than reproduces.

---

## 3. Series Structure at a Glance

| Tier | Volume | Focus | Audience |
|---|---|---|---|
| **Core** | Book 1 — Foundations of Motion Systems | Math, physics, and the end-to-end architecture | All readers |
| **Core** | Book 2 — Estimation & Motion Planning | Sensing to feasible trajectory | All readers |
| **Core** | Book 3 — Motion Control | Trajectory to actuation, robustly | All readers |
| **Core** | Book 4 — Integration, Real-Time Execution & Safety | The production wrapper and co-design | Mid-to-senior readers |
| **Companion** | Domain Specializations | Vehicle, manipulator, ground/aerial/underwater | Role-specific |
| **Advanced** | Frontier & Research | Optimization, learning-based, multi-agent | Expert readers |

The four core books map directly onto the motion pipeline, so the series reads as one continuous system rather than separate subjects.

---

## 4. The Core Series

### BOOK 1 — Foundations of Motion Systems

**Goal.** Establish the shared vocabulary, mathematics, physical models, and system architecture common to all motion systems, so that a reader can navigate any motion domain and follow both the rest of the series and the wider literature.

**Chapter 1 — Mathematics for Motion.** Linear algebra, coordinate frames, rotations (SO(2), SO(3)), homogeneous transforms (SE(2), SE(3)), Jacobians, differential equations, numerical integration, optimization intuition, probability and Gaussian distributions. Includes the often-omitted industrial essentials: numerical stability, conditioning, and discretization error. *Depth: W (O for proofs). Criticality: Critical. Anchor: standard linear-algebra texts; Lynch & Park, "Modern Robotics" for screw theory and transforms.*

**Chapter 2 — Physics and Dynamics.** Kinematics versus dynamics, Newton–Euler and Lagrangian formulations, constraints, holonomic versus nonholonomic systems, degrees of freedom, energy and momentum, friction, and stability concepts. *Depth: W. Criticality: Critical. Anchor: Lynch & Park.*

**Chapter 3 — The Motion Systems Architecture.** The complete stack — sensing, localization, mapping, prediction, planning, trajectory generation, control, actuation, safety, diagnostics — presented as one data-and-timing flow with explicit interface points. This chapter is rarely found in academic treatments and is a primary differentiator of the series. *Depth: O (conceptual, high value). Criticality: Critical.*

**Chapter 4 — Models and Abstraction Across Domains.** The unifying comparison across vehicles, manipulators, aerial, and underwater systems: what changes between them (model, dominant constraint, actuation, planning space) and what stays the same. Model hierarchy from point-mass to kinematic to full dynamic models, when each is adequate, underactuation, and the configuration-space concept. *Depth: W. Criticality: Critical.*

**Chapter 5 — Computational Foundations.** Discretization, fixed- versus floating-point representation, solver basics, and the distinction between offline and real-time computation. Prepares the reader for the implementation themes that recur throughout the series. *Depth: O→W. Criticality: Important.*

---

### BOOK 2 — Estimation and Motion Planning

**Goal.** Cover the front half of the pipeline — turning sensor data into a feasible, collision-free trajectory. Both halves lean on established references; the book's contribution is cross-domain framing, dynamic feasibility, and the failure modes that textbooks omit.

#### Part A — State Estimation and Sensor Fusion

**Chapter 6 — Sensors and Measurement Models.** IMU, wheel odometry, GNSS, lidar, radar, cameras, and encoders; noise, bias, and calibration. *Depth: W. Criticality: Critical. Anchor: Thrun, Burgard & Fox, "Probabilistic Robotics."*

**Chapter 7 — Recursive Estimation.** Kalman filter, extended and unscented variants, particle filters, and an overview of factor-graph methods. *Depth: W for KF/EKF; O for factor graphs. Criticality: Critical. Anchor: "Probabilistic Robotics."*

**Chapter 8 — Localization and Mapping (Survey).** Odometry, SLAM concepts, and map representations, presented at overview depth with explicit deferral to canonical sources. *Depth: O. Criticality: Important. Anchor: "Probabilistic Robotics"; Siegwart & Nourbakhsh.*

**Chapter 9 — Estimation in Production.** Sensor synchronization, timestamp alignment, latency compensation, covariance tuning, observability failures, sensor dropout, and degraded modes. This material is underserved by existing literature and is among the highest-value content in the book. *Depth: W→D. Criticality: Critical.*

#### Part B — Motion Planning and Trajectory Generation

**Chapter 10 — Trajectory Representation.** Cartesian and Frenet frames, splines, clothoids, Bézier and B-spline curves, and minimum-jerk and minimum-snap trajectories. *Depth: W. Criticality: Critical.*

**Chapter 11 — Search-Based Planning (Survey).** A*, Hybrid A*, D*, and lattice planning. *Depth: O→W. Criticality: Important. Anchor: LaValle, "Planning Algorithms."*

**Chapter 12 — Sampling-Based Planning (Survey).** RRT, RRT*, and PRM. *Depth: O→W. Criticality: Important. Anchor: LaValle.*

**Chapter 13 — Optimization-Based Planning.** Trajectory optimization, direct collocation, CHOMP, STOMP, iLQR, and the use of MPC as a planner. *Depth: W. Criticality: Important (and rising). Anchor: current optimal-control literature.*

**Chapter 14 — Dynamic Feasibility: The Planning–Control Bridge.** Curvature, acceleration, jerk, friction, and actuator constraints, and how feasibility guarantees are expressed and verified. This chapter connects the two core layers and carries strong industrial value. *Depth: W→D. Criticality: Critical.*

**Chapter 15 — Behavior and Decision Making (Survey).** Prediction, interaction-aware planning, game-theoretic methods, behavior trees, finite-state machines, and POMDPs. Presented at overview depth, with depth deferred to specialist texts. *Depth: O. Criticality: Important (Critical for autonomous driving).*

---

### BOOK 3 — Motion Control

**Goal.** Convert a feasible trajectory into actuator commands that behave robustly on real hardware. Organized in four layers from fundamentals to advanced methods, with dedicated treatment of real-system constraints.

#### Layer 1 — Core Control Theory

**Chapter 16 — Feedback Fundamentals.** Feedback systems, stability, controllability, observability, discretization, delays, and frequency response. *Depth: W. Criticality: Critical.*

**Chapter 17 — Classical Control.** PID (in depth), cascaded structures, feedforward, disturbance rejection, gain scheduling, and frequency-domain analysis with stability margins. *Depth: D. Criticality: Critical.*

#### Layer 2 — Model-Based Control

**Chapter 18 — State-Space and Optimal Control.** State-space modeling, LQR, LQG, observers, the separation principle, and tuning philosophy. *Depth: D. Criticality: Critical.*

**Chapter 19 — Model Predictive Control.** Linear and nonlinear MPC, horizon selection, cost formulation, hard and soft constraints, real-time quadratic-program solving, warm starting, terminal constraints, and computational trade-offs. *Depth: W→D. Criticality: Critical. Anchor: Borrelli, Bemporad & Morari, "Predictive Control."*

#### Layer 3 — Nonlinear and Advanced Control

**Chapter 20 — Nonlinear and Robust Control.** Lyapunov methods, feedback linearization, sliding-mode control, backstepping, differential flatness, adaptive control, and an overview of robust and H-infinity methods. *Depth: W (O for H-infinity). Criticality: Important.*

#### Layer 4 — Real-System Constraints

**Chapter 21 — Control on Real Hardware.** Actuator saturation, delays, quantization, rate limits, dead zones, anti-windup, asynchronous operation, packet loss, and fault handling. This layer is what separates research implementations from production systems and is largely absent from existing control texts. *Depth: D. Criticality: Critical.*

---

### BOOK 4 — Integration, Real-Time Execution, and Safety

**Goal.** Provide the production wrapper around the stack — the rarest and most commercially differentiated volume. Covers embedded and real-time execution, middleware, functional safety, and the co-design of planning, control, and estimation.

**Chapter 22 — Real-Time and Embedded Execution.** Real-time operating systems, deterministic scheduling, worst-case execution time, fixed- and floating-point implementation, embedded C++ patterns, and edge and GPU inference. *Depth: W. Criticality: Critical.*

**Chapter 23 — Middleware and System Software.** DDS, ROS 2, AUTOSAR, time synchronization, and common communication patterns. *Depth: O→W. Criticality: Important.*

**Chapter 24 — Interface Contracts and Co-Design.** The contracts between planner, controller, and estimator — assumptions, feasibility guarantees, update rates, and latency budgets — and the move toward integrated, optimization-based motion systems. This chapter expresses the central thesis of the series. *Depth: W→D. Criticality: Critical.*

**Chapter 25 — Functional Safety and Reliability.** ISO 26262, SOTIF, ASIL decomposition, fault-tolerant time intervals, safe-state definition, fault-tolerant control, redundancy, fail-operational architecture, safety monitors, and runtime verification. *Depth: W. Criticality: Critical. Anchor: ISO 26262 and ISO 21448 standards.*

**Chapter 26 — Failure Modes and Degraded Operation.** Sensor dropout, planner dropout, late trajectory delivery, constraint-violation handling, graceful degradation, and validation through simulation including sim-to-real transfer. Addresses the gap that distinguishes engineers who know algorithms from those who can ship systems. *Depth: W→D. Criticality: Critical.*

---

## 5. Companion Tier — Domain Specializations

**Goal.** Deliberately compact volumes that assume the core series and cover only what each domain adds. These can ship as one combined volume or as separate short books, depending on commercial preference.

- **Vehicle and ADAS.** Kinematic and dynamic bicycle models, tire models including combined slip and transient dynamics, suspension and aerodynamic effects, low-friction and high-speed stability; lane keeping, adaptive cruise control, path tracking, parking, highway automation, and safety fallback. *Anchor: Rajamani, "Vehicle Dynamics and Control."*
- **Robotic Manipulators.** Forward and inverse kinematics, Jacobians and singularities, manipulator dynamics, computed-torque control, impedance and admittance control, force control, operational-space control, and redundancy resolution. *Anchor: Siciliano et al.; Lynch & Park.*
- **Ground Mobile Robots.** Differential drive, Ackermann, skid steer, and omnidirectional platforms; field-robotics concerns including rough terrain, slip estimation, and localization degradation. *Anchor: Siegwart & Nourbakhsh.*
- **Aerial Robotics.** Quadrotor dynamics, attitude control, geometric control on SE(3), thrust allocation, wind-disturbance rejection, and minimum-snap trajectories. *Anchor: Mellinger & Kumar; Lee et al.*
- **Underwater Robotics.** Six-degree-of-freedom hydrodynamics, added mass, buoyancy and restoring forces, damping, current disturbances, thruster allocation, and underwater localization. *Anchor: Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control."*

A unifying note runs through this tier: aerial and underwater systems are both rigid bodies on SE(3) with thrust-style actuation, and are presented adjacently to make that symmetry visible.

---

## 6. Advanced Tier — Frontier and Research

**Goal.** Separate research-grade and emerging material from the employability-focused core, so the core books remain approachable.

- **Optimization for Motion Systems.** Convex optimization, quadratic programming, sequential quadratic programming, interior-point methods, and real-time and sparse solvers. Among the highest-return advanced topics, as optimization now underlies MPC, planning, estimation, and control allocation alike.
- **Learning-Based Motion Systems.** Imitation learning, reinforcement learning, learning MPC, policy optimization, sim-to-real transfer, and differentiable control. Positioned as emerging rather than established production practice.
- **Multi-Agent Autonomous Systems.** Swarm robotics, cooperative planning, vehicle-to-everything coordination, and distributed optimization.

---

## 7. Reference Map

The series anchors to the following established works, covering them at overview or working depth and directing readers to them for full treatment:

| Layer | Canonical Anchor |
|---|---|
| Kinematics, dynamics, manipulators | Lynch & Park, *Modern Robotics*; Siciliano et al., *Robotics: Modelling, Planning and Control* |
| Estimation, SLAM | Thrun, Burgard & Fox, *Probabilistic Robotics* |
| Planning algorithms | LaValle, *Planning Algorithms* |
| Mobile robots | Siegwart & Nourbakhsh, *Introduction to Autonomous Mobile Robots* |
| Predictive control | Borrelli, Bemporad & Morari, *Predictive Control* |
| Vehicle dynamics | Rajamani, *Vehicle Dynamics and Control* |
| Marine/underwater | Fossen, *Handbook of Marine Craft* |
| Functional safety | ISO 26262; ISO 21448 (SOTIF) |

---

## 8.  Notes

- **Book 1 is the broadest-market title.** It is sufficient on its own to give a reader cross-domain orientation, and it drives demand for the rest of the series.
- **Books 1 through 4 form the employability core** and target the largest audience: practising and aspiring motion engineers.
- **The companion tier monetizes role-specific demand** without bloating the core.
- **The advanced tier captures the expert and research market** while keeping the core accessible.
- **The series' differentiation is integration, production realism, and cross-domain abstraction** — the areas where current literature is thinnest. This is the basis for positioning the series as *motion systems engineering* rather than another set of isolated robotics or control titles.

---

## 9. Open Decisions

The following choices will sharpen the next revision into a full per-chapter outline with page budgets:

- Target reader level: practitioners retraining, or graduate students.
- Implementation stance: simulation-and-Python for learning, production C++, or both with a bridging chapter.
- Companion-tier packaging: a single combined domains volume, or separate short titles.
- Whether estimation and planning remain combined in one volume or split, depending on intended page count.
