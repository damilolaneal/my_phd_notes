---
title: "Possible Direction Moving Forward"
date: 2026-01-02
---

My aim is to get published in *Transactions on Robotics* (T-RO) and *RSS*, not second-tier conferences.

---

### Part 1: The "Kill or Cure" Critique (ADCP-Assisted SLAM)

**The Hypothesis:** Utilizing water column velocity profiles (from ADCP) as a static map for localization constraints.

#### Theoretical Footing: Hydrodynamics & Stability
You are proposing to treat the ocean flow field $\vec{u}(\mathbf{x}, t)$ as a map $M(\mathbf{x})$. For this to work as a loop-closure constraint, the condition 

$$\vec{u}(\mathbf{x}, t_1) \approx \vec{u}(\mathbf{x}, t_2)$$

must hold.

*   **Fluid Dynamics Reality:** In most operational environments (littoral, shelf, or deep ocean), flow is rarely laminar and time-invariant. You are dealing with turbulent flows, tidal harmonics, and internal waves.
*   **The Reynolds Number Problem:** At the scale of an AUV, the Reynolds number is high. If you pass through a wake or an eddy, that feature is transient. It exists at $t\_1$ and is gone or displaced by $t\_2$.
*   **Correlation:** A flow profile is often spatially low-frequency (it changes slowly over kilometers) but temporally high-frequency (turbulence). This is the **worst combination for SLAM**. SLAM needs spatially distinct, high-gradient features (sharp corners, distinct textures) that remain static. A 0.5 m/s current flowing North looks identical to a 0.5 m/s current flowing North 100 meters away. **Result:** Massive aliasing and perceptual ambiguity.

#### The "Skeptic's Argument" (Why this fails)
1.  **The Reference Frame Circularity (The Fatal Flaw):**
    To map the water velocity $\vec{u}\_{water}$, you measure the relative velocity $\vec{v}\_{rel}$ (ADCP raw data).

    $$ \vec{u}_{water} = \vec{v}_{rel} + \vec{v}_{robot} $$

    To get $\vec{u}\_{water}$, you need an accurate estimate of $\vec{v}\_{robot}$. But your goal is to *find* $\vec{v}\_{robot}$ (localization) using $\vec{u}\_{water}$.
    Unless you have a DVL lock on the bottom (Bottom Track), you are solving for two unknowns with one equation. If you *do* have Bottom Track, the DVL gives you superior constraints, rendering the water column data redundant and noisy.

2.  **Non-Stationarity:**
    Treating a vector field as a static map in a Factor Graph requires the assumption of a static world. Ocean currents are time-variant. You would need to estimate the tidal phase or model the Navier-Stokes equations in real-time to "predict" what the current *should* be when you return. That is a Physics-Based Data Assimilation problem, not a SLAM problem.

#### The Verdict: **Pivot (Mostly)**
Do not make "Current-Assisted SLAM" your primary thesis contribution. It is a weak constraint prone to drift and circularity.

**However**, if you are stubborn or your specific environment involves strong, interaction-based flow (e.g., flow around a bridge piling or distinct bathymetry where flow *is* a feature of the structure), you must rigorize it.

**The Fix (If you keep it):**
Do not use it for Loop Closure. Use it for **Gaussian Process (GP) Flow Mapping** to aid the process model (Dead Reckoning).
*   **Formulation:** Model the flow field as a Vector-valued Gaussian Process with a Spatiotemporal Kernel:

    $$ k((\mathbf{x}, t), (\mathbf{x}', t')) = k_{space}(\mathbf{x}, \mathbf{x}') \cdot k_{time}(t, t') $$

*   **Integration:** In your Factor Graph, introduce "Flow Factors." The state vector includes robot pose $X\_i$ and local flow velocity $V\_{flow}$. The GP serves as a smoothness prior on $V\_{flow}$.
*   **Value:** This allows you to subtract the estimated current from your dynamic model, reducing integration error during DVL dropouts. It is a *navigation aid*, not a *localization solution*.

---

### Part 2: The Pivot to Innovation (Alternative Avenues)

If you want an ICRA/TRO paper, you need to solve the "Feature-Poor/Unstructured" problem. Here are three avenues that combine modern compute with classical rigor.

#### Avenue 1: Implicit Sonar Representations (The "Acoustic NeRF")
**The Gap:** Feature-based SLAM fails in unstructured environments (sand/mud) because we cannot extract Harris corners or ORB features from fuzzy sonar blobs. Dense mapping (Occupancy Grids) is memory intensive and doesn't capture the view-dependent nature of acoustic scattering.

**The Novelty:** Apply **Implicit Neural Representations (INRs)** or **Gaussian Splatting** to Forward Looking Sonar (FLS) or Multibeam data. Instead of storing a point cloud, you train a neural network to represent the continuous acoustic field.

**The Formulation:**
*   **Architecture:** An MLP $F\_\theta(\mathbf{x}, \mathbf{d}) \rightarrow (\sigma, I)$, taking coordinate $\mathbf{x}$ and viewing direction $\mathbf{d}$ (incidence angle is critical in sonar), outputting density $\sigma$ and intensity $I$.
*   **Differentiable Acoustic Rendering:** Replace the optical ray-marching integral with a sonar equation approximation. The loss function minimizes the difference between the *synthesized* sonar image and the *real* sonar image.
*   **SLAM:** "Inverting" the NeRF. Once the field is partially learned, freeze the network and backpropagate the error to update the camera (sonar) pose.

**Feasibility:** High theoretical difficulty, moderate implementation. Requires GPU on the AUV or post-processing.
*   **Impact:** Massive. "First Neural Implicit SLAM for Acoustics" is an instant ICRA hit.

#### Avenue 2: Riemannian Terrain Mapping with Submap Matching
**The Gap:** In featureless terrain (sand waves), point features drift. However, the *shape* of the seafloor (bathymetry) is often unique over a larger patch. Standard Bathymetric SLAM assumes a prior map. We want *Concurrent* mapping without a prior.

**The Novelty:** Move away from points. Use **Riemannian Manifold definitions of surface patches**. Treat the bathymetry collected over 10 seconds not as points, but as a distribution on a manifold (Lie Group).

**The Formulation:**
*   **Submaps:** Accumulate Multibeam/DVL range data into dense local submaps.
*   **Factors:** Instead of reprojection error, use a **Point-to-Plane** or **Distribution-to-Distribution** metric within the Factor Graph.
*   **Loop Closure:** Detect loop closures by aligning submaps using Continuous-Time Trajectory estimation (e.g., Gaussian Process Regression on the trajectory) to deskew the point clouds, then applying ICP (Iterative Closest Point) with a robust kernel (Geman-McClure) to handle outliers (sand movement).
*   **Math:** Define the error function on $SE(3)$ directly. Use a hierarchical graph: Level 1 (Pose-to-Pose odometry), Level 2 (Submap-to-Submap alignment factors).

**Feasibility:** High. Standard hardware (DVL/MBES). The innovation is in the robust factor graph formulation.

#### Avenue 3: Information-Theoretic Active SLAM (The "Curious" AUV)
**The Gap:** Most underwater SLAM is passive. The robot executes a lawnmower pattern, accumulates error, and prays for a loop closure. In feature-poor environments, the robot should *seek* information.

**The Novelty:** **Active SLAM using Imaging Sonar.** The robot alters its trajectory to minimize the uncertainty of its pose and map.

**The Formulation:**
*   **Metric:** Shannon Entropy or the Trace of the Posterior Covariance Matrix (D-Optimality).
*   **Control:** Model Predictive Control (MPC).

    $$ J = \sum_{k=0}^{N} \| \mathbf{x}_k - \mathbf{x}_{goal} \|^2_Q + \lambda \cdot \text{Trace}(\Sigma_{slam}(\mathbf{u}_k)) $$
    
*   **Mechanism:** If the robot is over sand (high uncertainty, high covariance), the cost function drives it toward a known structure (rock, pipeline) seen previously to "reset" the drift (close the loop), then return to the goal.
*   **Deep Learning Hook:** Use a CNN to predict "Loop Closure Probability" from the current sonar image to weight the cost function.

**Feasibility:** Very High (Real-time computational constraints).
**Impact:** RSS material. This moves from "mapping" to "autonomous exploration."

### Final Advice
Drop the water column velocity idea as a SLAM backbone. It’s physically unstable.
*   If you love math/optimization $\rightarrow$ **Avenue 2** (Submap Factor Graphs).
*   If you love Deep Learning/Vision $\rightarrow$ **Avenue 1** (Acoustic NeRF).
*   If you love Control Theory $\rightarrow$ **Avenue 3** (Active SLAM).

Pick one. Formulate the math. Bring me the derivation next week.