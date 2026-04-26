# MOD-07: Robot Perception

## Student Script — Full Mathematical Foundations

**Course:** Mobile Robots  
**Instructor:** Dr. Ahmad Abbadi  
**Prerequisites:** MOD-04 (Sensors), MOD-06 (Localization)  
**Key References:**

- **[SZ]** Szeliski, R. (2022). *Computer Vision: Algorithms and Applications*, 2nd ed. Springer. (Free online)
- **[HZ]** Hartley, R. & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision*, 2nd ed. Cambridge.
- **[AMR]** Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press. Ch. 4.
- **[PR]** Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Ch. 6.

---

## A. The Perception Pipeline

### A.1 What Perception Delivers

Perception transforms raw sensor data into actionable information for downstream modules:

| Consumer | What perception provides |
|----------|------------------------|
| **Localization (MOD-06)** | Detected landmarks with descriptors for matching |
| **Planning (MOD-08)** | Obstacle positions, free space, semantic labels |
| **SLAM (MOD-09)** | Visual features for loop closure, point cloud segments |

### A.2 The Five-Stage Pipeline

Every perception system follows this pattern regardless of platform [AMR, §4.1]:

**Sense → Preprocess → Detect → Classify → Interpret**

Each stage reduces data volume and increases information value. Example: camera produces 921,600 bytes/frame → preprocessing yields a clean image → detection finds 50 features → classification identifies 5 objects → interpretation produces 2 navigation decisions.

---

## B. Image Preprocessing

### B.1 Gaussian Blur (Noise Reduction)

Convolution with a 2D Gaussian kernel [SZ, §3.2]:

$$G(x, y) = \frac{1}{2\pi\sigma^2} \exp\left(-\frac{x^2 + y^2}{2\sigma^2}\right)$$

The smoothed image I' is obtained by convolution: I'(u, v) = (G * I)(u, v). A 5×5 kernel with σ = 1.0 is standard for removing sensor noise while preserving edges.

### B.2 Edge Detection

**Sobel operator** computes image gradients [SZ, §3.2.3]:

$$G_x = \begin{bmatrix} -1 & 0 & 1 \\ -2 & 0 & 2 \\ -1 & 0 & 1 \end{bmatrix} * I, \quad G_y = \begin{bmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{bmatrix} * I$$

Edge magnitude: M = √(G_x² + G_y²). Edge direction: θ = atan2(G_y, G_x).

**Canny edge detector** [SZ, §3.2.3] refines Sobel output through: (1) Gaussian smoothing, (2) gradient computation, (3) non-maximum suppression (thin edges to 1 pixel), (4) hysteresis thresholding (keep strong edges + weak edges connected to strong ones).

---

## C. Feature Extraction

### C.1 What Makes a Good Feature?

A feature is a distinctive, repeatable point in the image. Three types of image regions [SZ, §7.1]:

- **Flat region:** No intensity change in any direction → not a feature
- **Edge:** Change in one direction only → ambiguous position along edge
- **Corner:** Change in *both* directions → uniquely locatable → **good feature**

### C.2 Harris Corner Detector

The Harris detector analyzes the **structure tensor** (second moment matrix) [SZ, §7.1.1]:

$$\mathbf{M} = \sum_{(x,y) \in W} \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}$$

Where I_x, I_y are image gradients, and the sum is over a window W around each pixel.

**Harris corner response:**

$$R = \det(\mathbf{M}) - k \cdot (\text{trace}(\mathbf{M}))^2 = \lambda_1 \lambda_2 - k(\lambda_1 + \lambda_2)^2$$

Where λ₁, λ₂ are the eigenvalues of **M** and k ≈ 0.04–0.06.

**Interpretation via eigenvalues:**
- Both λ small → flat region (R ≈ 0)
- One λ large, one small → edge (R < 0)
- Both λ large → corner (R >> 0)

**Shi-Tomasi** variant: score = min(λ₁, λ₂) > threshold. Slightly more robust in practice.

### C.3 Feature Descriptors

**ORB (Oriented FAST and Rotated BRIEF)** [Rublee et al., 2011]:
- **Detector:** FAST corner detector with orientation from intensity centroid
- **Descriptor:** Rotated BRIEF — 256-bit binary string computed from pairwise pixel comparisons
- **Matching distance:** Hamming distance (XOR + popcount — extremely fast)
- Free, no patents, real-time capable. Standard in ORB-SLAM.

**SIFT (Scale-Invariant Feature Transform)** [Lowe, 2004; SZ, §7.1.2]:
- **Detector:** Difference-of-Gaussians (DoG) at multiple scales → finds features at characteristic scale
- **Descriptor:** 128-float gradient histogram, rotation-normalized
- **Matching distance:** L2 (Euclidean) distance
- Scale-invariant and rotation-invariant. The gold standard for accuracy but slower than ORB.

### C.4 Feature Matching

**Brute-force matching:** For each feature in image A, compute distance to all features in image B. Pick the closest as the match.

**Lowe's ratio test** [Lowe, 2004]: For each feature, find the two closest matches (d₁ ≤ d₂). Accept only if:

$$\frac{d_1}{d_2} < 0.75$$

Intuition: a good match should be *much* closer than the second-best. This removes ~50% of false matches.

---
Below is a clean **Markdown study script** you can give to students in a Mobile Robotics / Computer Vision course.

---

# Feature Detection & Description for Mobile Robots

## Study Script

---

# 1. Why robots need visual features

Mobile robots must understand the world from camera images to:

* Localize themselves (Visual Odometry / SLAM)
* Recognize places
* Track objects
* Navigate safely

Instead of comparing every pixel, robots extract **distinctive points** from images.

These are called **keypoints**.

---

# 2. What is a Keypoint?

A **keypoint** is a distinctive, repeatable pixel location that can be found again in another image.

Good keypoints:

* Corners of doors/windows
* Edges intersection
* Texture details

Bad keypoints:

* Plain walls
* Sky
* Uniform floor

Think of keypoints as **visual landmarks** for robots.

---

# 3. Corner Detection

Robots often use **corners** as keypoints because they are stable and easy to find again.

## 3.1 Harris Corner Detector

Idea:
A corner is a point where image brightness changes strongly in **all directions**.

We analyze how image intensity changes when moving a small patch.

Mathematically, we build the **structure tensor**:

```
M = [ Σ Ix²   Σ IxIy
      Σ IxIy  Σ Iy² ]
```

The Harris score:

```
R = det(M) − k(trace(M))²
```

Interpretation:

| Region    | Meaning                  |
| --------- | ------------------------ |
| Flat area | No change                |
| Edge      | Change in one direction  |
| Corner    | Change in two directions |

---

## 3.2 Shi–Tomasi Corner Detector

Improved Harris.

Instead of the Harris formula, Shi–Tomasi uses **eigenvalues** of the matrix M.

### What are eigenvalues (intuitive)?

A matrix transforms directions in space.
Eigenvalues measure **how much stretching happens along special directions**.

For corner detection:

| Case        | Eigenvalues      |
| ----------- | ---------------- |
| Flat region | λ₁ ≈ 0, λ₂ ≈ 0   |
| Edge        | λ₁ big, λ₂ small |
| Corner      | λ₁ big, λ₂ big   |

Shi–Tomasi rule:

```
Corner if min(λ₁, λ₂) > threshold
```

Meaning:
A true corner must change strongly in **all directions**.

This produces more stable corners for tracking.

---

# 4. FAST Corner Detector

FAST = Features from Accelerated Segment Test
Used in real-time robotics.

Idea:

1. Take a pixel.
2. Look at 16 pixels around it in a circle.
3. If many are much brighter or darker → corner.

Why fast?

* Only simple brightness comparisons.
* No heavy math.

Used in ORB and real-time SLAM.

---

# 5. From Keypoints to Feature Descriptors

Finding a keypoint is not enough.
We must **describe it** so we can match it between images.

A descriptor = numeric signature of the patch around a keypoint.

---

# 6. BRIEF Descriptor

BRIEF uses **binary brightness comparisons**.

At each keypoint:

1. Take a small patch.
2. Compare brightness of many pixel pairs.
3. Store results as binary string.

Example descriptor:

```
101101001011010…
```

Matching = compare binary strings using Hamming distance.

Pros:

* Extremely fast.

Cons:

* Not rotation invariant.

---

# 7. ORB (Oriented FAST and Rotated BRIEF)

ORB combines:

* FAST detector
* Orientation estimation
* Rotated BRIEF descriptor

This makes ORB a **complete feature pipeline**.

---

## 7.1 Orientation via Intensity Centroid

Treat brightness like mass.

Compute center of brightness:

```
Cx = Σ x·I(x,y) / Σ I(x,y)
Cy = Σ y·I(x,y) / Σ I(x,y)
```

Direction from keypoint → centroid = orientation.

Descriptor is rotated accordingly → rotation invariance.

---

## 7.2 ORB Robustness

| Transformation            | ORB performance |
| ------------------------- | --------------- |
| Rotation                  | Excellent       |
| Small scale change        | Good            |
| Large scale change        | Poor            |
| Strong perspective change | Weak            |

ORB is **fast but not perfect**.

Used widely in real-time robotics.

---

# 8. SIFT — Scale Invariant Feature Transform

SIFT is more robust but slower.

Designed to handle:

* Large scale changes
* Rotation
* Lighting changes
* Perspective (partially)

---

## 8.1 Scale-Space Detection

SIFT searches features across multiple blur levels.

Uses **Difference of Gaussian (DoG)** to detect features at their natural size.

This gives true **scale invariance**.

---

## 8.2 Orientation Assignment

Compute gradient directions around keypoint.
Build histogram of directions.
Dominant direction = keypoint orientation.

More robust than ORB.

---

## 8.3 SIFT Descriptor

Patch is divided into 4×4 grid.
Each cell stores gradient histogram (8 bins).

Descriptor size:

```
4 × 4 × 8 = 128 values
```

Very distinctive and robust.

---

# 9. SURF — Speeded Up Robust Features

SURF is a faster approximation of SIFT.

Uses:

* Box filters
* Integral images
* Hessian matrix detection

Descriptor size: 64 values.

---

# 10. Deep Features (Modern Approach)

Instead of hand-designed features, we train neural networks.

Examples:

* SuperPoint
* D2-Net
* R2D2
* LoFTR

Neural networks learn:

* Where keypoints are
* How to describe them
* How to match them

They are robust to:

* Day/night changes
* Weather
* Large viewpoint changes
* Motion blur

---

# 11. Summary Comparison

| Method        | Speed        | Robustness     |
| ------------- | ------------ | -------------- |
| ORB           | Very fast    | Medium         |
| SIFT          | Slow         | Very high      |
| SURF          | Medium       | High           |
| Deep features | GPU required | Extremely high |

---

# 12. Example in Mobile Robotics

## Visual Odometry Example

A robot moving in a corridor:

Step 1 — Capture frame at time t
Step 2 — Detect keypoints (ORB/SIFT)
Step 3 — Describe keypoints
Step 4 — Capture next frame (t+1)
Step 5 — Match features between frames
Step 6 — Estimate camera motion from matches

This allows the robot to estimate:

* How far it moved
* How much it rotated

This is the basis of:

* Visual Odometry
* Visual SLAM
* Autonomous navigation

---

# Final Takeaway

Pipeline used in mobile robots:

```
Image
  → Keypoint detection (Harris / FAST / SIFT)
  → Feature description (ORB / SIFT / Deep)
  → Feature matching
  → Motion estimation / SLAM
```

These techniques allow robots to **see, localize, and navigate** using cameras.

---

## D. Geometric Verification with RANSAC

### D.1 The Problem

After descriptor matching + ratio test, some matches are still wrong (outliers). RANSAC identifies and removes them [Fischler & Bolles, 1981; SZ, §8.1].

### D.2 RANSAC Algorithm

```
Algorithm: RANSAC(matches, model_type, threshold, max_iterations)
  best_inliers = ∅
  for i = 1 to max_iterations:
    1. Randomly select minimal set S (e.g., 4 matches for homography)
    2. Fit model M from S (compute transformation)
    3. Count inliers: all matches where error(match, M) < threshold
    4. If |inliers| > |best_inliers|: best_inliers = inliers, best_model = M
  Refit model using ALL best_inliers
  Return best_model, best_inliers
```

**Number of iterations needed** for probability p of finding an outlier-free sample with n points and outlier ratio ε:

$$k = \frac{\log(1-p)}{\log(1-(1-\varepsilon)^n)}$$

For p = 0.99, n = 4, ε = 0.30 (30% outliers): k = 16 iterations. For ε = 0.50: k = 72 iterations.

---

## E. Visual Odometry (Concept)

### E.1 Definition

**Visual Odometry (VO):** Estimate the robot's incremental motion from a sequence of camera images [Scaramuzza & Fraundorfer, 2011; AMR, §4.4].

### E.2 Pipeline

1. **Detect features** in frame k (ORB, Harris)
2. **Match features** between frame k and k+1 (descriptor matching + RANSAC)
3. **Estimate motion** (R, t) from matched feature correspondences
4. **Integrate trajectory:** T_total = T₁→₂ × T₂→₃ × ...

### E.3 Monocular vs. Stereo

**Monocular VO:** Cannot recover absolute scale (a scene could be 1× or 10× its true size and look identical from one viewpoint). Needs external information (IMU, known object size) for metric scale.

**Stereo VO:** Known baseline B between cameras → triangulation gives metric depth → true-scale motion. Standard for mobile robots.

**Stereo depth** (from MOD-04): Z = f·B / d, where d is disparity. Uncertainty: σ_Z ∝ Z² — depth accuracy degrades quadratically with distance.

### E.4 Limitations

VO fails with: pure rotation (no parallax for triangulation), textureless surfaces (no features), fast motion (motion blur), changing lighting. VO drifts over time — must be fused with IMU and other odometry sources.

---

## F. Object Detection

### F.1 Hough Transform for Lines

The Hough transform detects parametric shapes in edge images [SZ, §7.4.2].

**Line parametrization:** ρ = x cos θ + y sin θ, where ρ = perpendicular distance from origin, θ = angle.

**Algorithm:** Each edge pixel (x, y) votes for all lines (ρ, θ) passing through it. Accumulate votes in (ρ, θ) space. Peaks in the accumulator = detected lines.

Extends to circles: (x − a)² + (y − b)² = r². 3D accumulator (a, b, r).

### F.2 Deep Learning Detection (YOLO — Concept)

**YOLO (You Only Look Once)** [Redmon et al., 2016]: Divide image into S×S grid. Each cell predicts B bounding boxes with confidence and C class probabilities. Single forward pass through a CNN → all detections at once → real-time (30+ FPS).

Output per detection: bounding box (x, y, w, h), confidence score p ∈ [0, 1], class label.

---

## G. Point Cloud Processing (Concept)

### G.1 Pipeline

Raw cloud (100K points) → **Voxel downsample** (replace all points in 5cm cube with centroid → 20K) → **Ground removal** (RANSAC plane fit → remove floor → 5K) → **Euclidean clustering** (group nearby points → 3 object clusters) → **Bounding boxes** (fit boxes for planning).

### G.2 Ground Plane Removal via RANSAC

Fit plane ax + by + cz + d = 0 to the point cloud using RANSAC. The largest plane with normal ≈ (0, 0, 1) is the floor. Remove its inlier points. Remaining points are potential obstacles.

### G.3 Normal Estimation

For each point, fit a local plane to its k nearest neighbors. The plane's normal vector indicates surface orientation [Rusu & Cousins, 2011]:

- Normal ≈ (0, 0, 1) → horizontal surface (floor, table)
- Normal ≈ (1, 0, 0) or (0, 1, 0) → vertical surface (wall)
- Tilted normal → ramp, sloped surface

---

## H. Semantic Understanding

Semantic perception assigns **meaning** to detected objects and regions:

- **Classification:** One label per image/region — "this is a wheelchair"
- **Object detection:** Bounding box + class — "wheelchair at (3.2, 1.5)"
- **Semantic segmentation:** Per-pixel labels — "this pixel = floor, that pixel = wall"

Semantic information feeds directly into the costmap (MOD-08): floor → free (cost 0), wall → lethal (cost 254), person → high cost (yield), door open → free, door closed → lethal.

---

## References

1. Szeliski, R. (2022). *Computer Vision: Algorithms and Applications*, 2nd ed. Springer.
2. Hartley, R. & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision*. Cambridge.
3. Rublee, E. et al. (2011). ORB: An Efficient Alternative to SIFT or SURF. *ICCV*.
4. Lowe, D.G. (2004). Distinctive Image Features from Scale-Invariant Keypoints. *IJCV*, 60(2).
5. Fischler, M. & Bolles, R. (1981). Random Sample Consensus. *Comm. ACM*, 24(6).
6. Redmon, J. et al. (2016). You Only Look Once: Unified, Real-Time Object Detection. *CVPR*.
7. Scaramuzza, D. & Fraundorfer, F. (2011). Visual Odometry Tutorial. *IEEE RAM*, Parts I–II.
8. Rusu, R. & Cousins, S. (2011). 3D is Here: Point Cloud Library. *ICRA*.
9. Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press. Ch. 4.
