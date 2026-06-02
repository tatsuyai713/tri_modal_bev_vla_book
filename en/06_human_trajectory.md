# Chapter 6 Human Trajectory Teacher and Lane-Center-Free Planning

---

## 6.1 Structure and Limits of Lane Centerline Following

In conventional ADAS/autonomous driving, the "correct answer" for planning is often defined as the deviation from the lane centerline.

```text
Conventional target:
  lateral_error = current lateral position - lane center
  heading_error = current heading - lane heading

Penalty: minimize lateral_error + heading_error
```

This approach causes problems in the following scenarios.

---

## 6.2 Scenarios Where Lane Centerline Fails

### Scenario 1: Parked Vehicles

```text
Problem: A parked vehicle is on the left shoulder
The lane centerline passes through the parked vehicle,
making the "correct answer" drive through the obstacle

Human: slightly offset to the right to pass
Lane center following: heads toward the parked vehicle, requiring correction rules
```

### Scenario 2: Driving Alongside a Large Truck

```text
Problem: a large truck is driving in the adjacent lane
Human: naturally moves slightly away from the truck (psychological margin)
Lane center following: ignores the truck's presence and stays at lane center
-> passengers feel uneasy, and actual risk increases
```

### Scenario 3: Curves

```text
Problem: a tight curve
Human: a natural line from outside to inside of the curve, using the apex
(known as "out-in-out"; gentle on normal roads)
Lane center following: traverses the curve at a uniform lateral position
-> unnatural lateral G-force can occur
```

### Scenario 4: Stopping Position at Intersections

```text
Problem: stopping position while waiting to turn right at an intersection
Human: slightly before the intersection center, where oncoming traffic can pass
Lane center following: proceeds to the intersection center along the lane center
```

### Scenario 5: Sections Where Lane Markings Have Faded

```text
Problem: construction zones, faded lane markings
Cannot compute lane center, so control becomes unstable

Human: uses road edges, shoulder, and leading vehicle to judge naturally
```

---

## 6.3 The Concept of Using Human Trajectories as Teachers

The important insight to learn from Comma.ai/openpilot's E2E Lateral Planning is: **use not the lane centerline but the future trajectory from actual human driving logs as teacher signals**.

```text
Human Trajectory:
  - Obtain the ego vehicle's future positions from time t to several seconds ahead from driving logs
  - Convert to ego frame
  - Use this trajectory as the Planner's target
```

### Why This Is Powerful

```text
1. Contains tacit knowledge
   - Lateral offset for avoiding parked vehicles
   - Distance from large vehicles
   - Natural line through curves
   - Stopping position at intersections
   - These are judgments humans make naturally, hard to write as rules

2. Easy to scale
   - Large driving logs can be collected at low cost
   - Teacher data can be created without dedicated labeling

3. Vehicle-independent
   - Trajectories are in meters, ego-relative
   - The same trajectory data can be used when the vehicle type changes
```

---

## 6.4 Ego-Frame Trajectory Each Cycle

In this design, the Planner outputs a target trajectory in the current ego frame each processing cycle.

```text
Target trajectory:
  - Baseline: current time t = 0
  - Up to T steps ahead (e.g., 3 seconds, T=10 steps, 0.3-second intervals)
  - Each point: (x, y, yaw, v)

Note:
  - Does not carry over the previous frame's trajectory (recomputed each cycle)
  - This enables fast response to sudden scene changes
  - Discontinuities with the previous frame are smoothed by the Converter
```

---

## 6.5 Responsibility Separation: Converter vs Controller

Human-trajectory-taught Planners and the actual control commands to the vehicle are handled by different modules.

```text
Planner (subject of this chapter):
  - Outputs target trajectory [T, D] each cycle
  - Does not know the vehicle model
  - Vehicle-type-independent

Converter (subject of Chapter 7):
  - Target trajectory + Ego state -> target curvature (steering angle)
  - Deterministic conversion
  - Knows vehicle parameters (wheelbase, steering ratio, etc.)
```

Through this separation:
- The Planner is learnable and vehicle-independent
- The Converter is testable and has high transparency

---

## 6.6 Using Lane as Context, Not as Target

An important design decision in this design:

```text
Conventional: lane_center = TARGET (minimize deviation)
This design:  lane_center = CONTEXT (passed to CondFormer as reference)

The Planner:
  - Uses Lane Topology as information about "where to go"
  - But does NOT directly optimize for following the lane center
  - Minimizes the difference from human trajectories
```

### Why This Is Better

```text
By making lane center a Context:
  - Lateral offsets within the lane (e.g., avoiding parked vehicles) are learned naturally
  - Does not crash when lane markings are absent (because teacher is human)
  - Natural lane-change trajectories can be learned
  - When lane information quality drops, the human teacher compensates
```

---

## 6.7 How to Build Human Trajectory Labels

Method for creating teacher trajectories from driving logs.

```text
Step 1: Obtain the ego vehicle's future trajectory from logs
  - Precise trajectory via GPS + IMU + dead reckoning
  - From t = 0 to T=10 steps ahead
  - Obtained in global coordinates

Step 2: Convert to the current ego frame
  - Transform relative to the current ego position/yaw
  - ego_transform(global_traj, ego_pose_t) -> local_traj

Step 3: Add speed profile
  - Append the actual vehicle speed at each point
  - v_t = ||(pos_{t+1} - pos_t)|| / delta_t

Step 4: Quality filtering
  - Exclude or down-weight logs with hard steering / hard braking
  - Exclude scenes with sensor failures or driver intervention
  - Adjust for poor-quality scenes at night or in bad weather
```

---

## 6.8 Data Quality Management

```text
Scenes to exclude:
  - Driver intervention (hand-off, emergency braking)
  - Sensor loss (no LiDAR, GPS outage)
  - Abnormal lateral acceleration (|a_y| > 5 m/s^2)
  - Abnormal longitudinal acceleration (|a_x| > 8 m/s^2)
  - Scenes with traffic violations (speeding, etc.)

Scenes to down-weight:
  - Rain / low visibility
  - Nighttime (supplemented by LiDAR, etc.)
  - Construction zones

Scenes to up-weight:
  - Low-frequency scenarios (pedestrian crossings, cut-ins, etc.)
  - Natural lane changes
  - Avoiding parked vehicles
```

---

## 6.9 Multi-Modal Learning with K-candidate + MHP Loss

For a single scene, there is only one human trajectory (one log). However, the Planner outputs multiple candidates.

```text
Multiple Hypothesis Prediction (MHP) Loss:
  - Among K candidates, select the candidate k* closest to the ground truth
  - Apply trajectory error loss ONLY to that candidate k*
  - This makes all candidates compete to learn "one of the correct answers"

L_traj = min_k Sum_t || traj_k_t - gt_t ||_2^2
       = Sum_t || traj_{k*}_t - gt_t ||_2^2
```

### Why Multiple Modes Are Needed

```text
Even in the same scene, there are multiple "correct" actions:
  - Whether to avoid right or left
  - Whether to accelerate and go first, or decelerate and wait
  - Whether to closely follow the leading vehicle or keep a large margin

With K candidates, all of these can be learned.
The external evaluator selects the safer candidate.
```

---

## 6.10 Speed Planning and Longitudinal Teacher

The human trajectory includes a speed profile. This is used as the teacher for longitudinal planning.

```text
Longitudinal teacher signal:
  - gt_v: actual vehicle speed at each timestep
  - gt_ax: longitudinal acceleration

Planner's longitudinal loss:
  L_speed = Sum_t || v_pred_t - gt_v_t ||_2^2
  L_accel = Sum_t || a_pred_t - gt_ax_t ||_2^2
```

### Special Handling for Signal Response and Stop Lines

```text
In scenes of signal stops and temporary stops:
  - gt_v drops sharply to 0
  - Weight of L_speed is doubled compared to normal
  - Distance to stop line is added as input to CondFormer
  - Traffic light state (RED/YELLOW/GREEN/ARROW/UNKNOWN) is added as a speed-planning condition
```

### Stopline and Following-Gap Stop Precision

```text
Stop precision tends to vary under imitation-only training.
Required behavior:
  - Stop within the allowed zone before the stop line at red lights and stop signs
  - Stop with a consistent time/distance margin behind a stopped lead vehicle
  - Avoid unnecessary re-acceleration or harsh braking near the final stop

Therefore, IL learns the distribution of human stopping behavior, while closed-loop/offline RL minimizes stopping error.
```

```text
Additional teacher/state signals:
  - stopline_distance_m
  - traffic_light_state
  - lead_vehicle_distance_m
  - lead_vehicle_speed_mps
  - desired_stopping_gap_m
  - final_stop_position_error_m
```

### Generate Target Speed Profile Jointly with Trajectory

```text
The Planner should output a longitudinal profile on the same time grid as the lateral trajectory.

Per timestep outputs:
  - v_target_t
  - a_x_target_t
  - phase_t: ACCEL / CRUISE / DECEL

This lets the Converter compute target steering, speed, and acceleration
from the same selected candidate in a consistent rule-based way.
```

### Desired Speed Behavior Around Curves

```text
Desired pattern:
  1) Predictive deceleration before curve entry
  2) Near-constant speed during cornering
  3) Gradual acceleration at curve exit

Imitation-only training is often insufficient for stable emergence of this pattern
because driver style variance dominates and optimal preview behavior is implicit.
```

### Modern IL + Closed-Loop Optimization

```text
Recent E2E driving research does not usually rely on pure imitation alone.
The common direction is to combine:

  1. Large-scale IL (behavior cloning / trajectory imitation)
  2. Closed-loop simulation evaluation and improvement
  3. offline RL / constrained RL / planning-oriented losses for speed and comfort
  4. A final rule-based evaluator for safety and legal constraints

Here, RL does not mean unconstrained online exploration on real vehicles.
It means offline or closed-loop fine-tuning using simulators and logged datasets.
```

```text
Training policy:
  Phase A: Large-scale IL from human trajectories and speed profiles
  Phase B: Closed-loop simulation evaluation for curves, merges, stop lines, traffic lights, and lead-vehicle following
  Phase C: offline RL / constrained RL for speed and comfort optimization
  Phase D: Mixed IL + RL-objective refinement to preserve human-likeness

Typical reward:
  r = w_progress * progress
    - w_jerk * |jerk|
    - w_lat * ReLU(v^2*kappa - a_y_max)
    - w_lat_meas * ReLU(|v*yaw_rate| - a_y_max)
    - w_stop * |stop_position_error|
    - w_gap * |actual_stop_gap - desired_stop_gap|
    - w_rule * speed_limit_violation
    - w_risk * collision_or_high_risk
```

```text
Practical notes:
  - Pre-train RL mainly in simulation, then fine-tune with curated real logs/offline datasets.
  - Do not perform exploratory RL actions on real vehicles.
  - Keep a high penalty on speed-limit violations.
  - Heavily penalize signal violations, stopline overshoot, and insufficient stopped following gap.
  - Keep final rule-based speed validation in the external evaluator as a safety backstop.
```

### Phase 1 MVP: K-query VLA Planner (direct application of §5.5 design)

```text
The K-query Transformer Decoder is well-suited for roadmap Phase 1–2 (MVP through
closed-loop evaluation) due to its implementation simplicity and low compute cost.

  Configuration:
    - K=8–24 learnable queries
    - Transformer Decoder (4–6 layers)
    - MHP Loss: L_traj = min_k L2(traj_k, traj_human)
    - Speed profile head: (v_target, a_x, phase) per timestep

  Advantages:
    - K candidates generated in a single forward pass; easy to debug
    - Standard Transformer implementation with low GPU memory
    - MHP Loss is intuitive and converges reliably
    - §6.11 joint lat-lon output format applies directly

  Applicability:
    Ideal for roadmap Phases 1–3 (Offline Open-loop through Shadow Mode).
    After BEV perception, Evaluator, and Converter are stable,
    consider migrating to DiffusionDrive.
```

### Phase 2 Evolution: DiffusionDrive (Diffusion-Based Trajectory Generation) + External Evaluator

```text
After confirming stable K-query Planner operation, consider DiffusionDrive migration when:

  Migration conditions:
    - K fixed candidates provide insufficient coverage for rare scenarios
    - Need to improve stop accuracy, IDM following, or anticipatory curve speed
    - Richer candidate diversity desired for External Evaluator safety selection

  DiffusionDrive advantages over K-query:
    - (x, y, v, a_x, phase) generated jointly in a single denoising pass → §6.11 structurally guaranteed
    - Conditioning inputs (stopline/TL/lead_veh/curvature) make denoising loss implicitly
      learn all §6.12 multi-phase behaviors (no reward engineering needed)
    - Continuous distribution, M=16–32 samples → Evaluator always has selectable candidates
    - Real-time on NVIDIA Drive demonstrated (NVIDIA ECCV 2024)
```

```text
DiffusionDrive configuration in this design:

  Method: DiffusionDrive (conditional diffusion model) + External Evaluator candidate selection

  How §6.12 behaviors are learned (via conditioning inputs):
    The diffusion model receives the following as conditioning inputs:

      stopline_distance_m     → speed convergence to zero at stopline (§6.12 stop profile)
      traffic_light_state     → anticipatory deceleration at RED/YELLOW
      lead_vehicle_distance_m → IDM-style car-following behavior (§6.12 IDM)
      lead_vehicle_speed_mps  → acceleration/deceleration based on relative speed
      curvature_profile[t]    → anticipatory curve speed control (§6.12 curve speed)
      desired_speed_mps       → speed limit / cruise speed ceiling

    Human driver logs naturally exhibit S-curve profiles, IDM-style following, and
    anticipatory deceleration in response to these inputs; therefore the denoising
    loss alone implicitly learns all §6.12 multi-phase behaviors.

  Inference flow:
    1. Sample M trajectories conditioned on (BEV features, Lane Topology, speed inputs)
    2. External Evaluator scores all M samples using the §6.11 joint lat-lon score
    3. Best-scoring trajectory k* is forwarded to the Converter
```

```text
Training pipeline (3 phases):

  Phase A: Large-scale IL (diffusion denoising pretraining)
    Loss: L_diffusion = E[ ||eps - eps_theta(traj_noisy, t_noise, cond)||^2 ]
    cond = all conditioning inputs (speed conditions + BEV features + Lane Topology)
    §6.12 multi-phase behaviors are implicitly acquired here

  Phase B: Closed-loop simulation evaluation
    Closed-loop behavior measured in Waymax or CARLA
    Distribution shift (compounding error) detected; retraining targets identified

  Phase C: CMDP-constrained fine-tuning (if needed)
    Lagrangian CMDP narrows the sampling distribution
    External Evaluator constraint flags C_1..C_4 serve directly as cost signals
```
---

## 6.13 Joint Lateral-Longitudinal Optimization: Principles and Structural Advantages

### The Fundamental Coupling Problem

Approaches that plan lateral (steering) and longitudinal (speed/acceleration) independently appear simple but face a fundamental physical coupling.

```text
Why lateral and longitudinal are NOT independent:

[Lateral acceleration constraint]
  a_y = v^2 * kappa  <=  A_Y_MAX

  -> Given curvature kappa, the safe speed ceiling is determined
  -> Given speed v, the allowable curvature ceiling is determined
  -> The two are not independent; each constrains the other

  Example: kappa = 0.05 (1/m), A_Y_MAX = 3.0 m/s^2
    -> v_max = sqrt(3.0 / 0.05) = 7.75 m/s (~28 km/h)
    -> At 50 km/h (13.9 m/s), entering a kappa=0.05 curve violates lateral G

[Approach to stop line with right-turn lane]
  - Lane change (lateral) and stop (longitudinal) must be planned simultaneously
  - Decoupled: lateral plan may require a sharp lane change while longitudinal
    plan applies braking too late -> arrive at curve at full speed
  - Coupled: "gentle lane change while decelerating early" is selected at once
```

### Failure Modes of Traditional Decoupled Planning

```text
Decoupled approach (sequential):
  Step 1: Plan lateral trajectory (x_t, y_t) ignoring speed
  Step 2: Compute curvature kappa_t of that trajectory
  Step 3: Compute speed profile v_t from kappa_t retroactively

Problems:
  1. [Inconsistency] Step 1 selects a sharp curve but Step 3 speed transition
     is delayed -> enters curve too fast, lateral G exceeds A_Y_MAX

  2. [Selection failure] Among K lateral candidates, a "slightly wider curve"
     candidate would allow higher speed, but Step 1 cannot know this

  3. [Global comfort loss] Simultaneous peaks of lateral and longitudinal jerk
     (sharp steer + hard braking) cannot be penalized in a unified objective

  4. [Anticipation mismatch] Timing of anticipatory deceleration before a curve
     may be misaligned between lateral and longitudinal planners
```

### Mathematical Formulation of Joint Optimization

The Planner outputs both lateral and longitudinal on the **same time grid simultaneously**.

```text
Planner output (K candidates x T timesteps):
  trajectory[k][t] = {
    x_t:        forward distance (m)          <- lateral
    y_t:        lateral offset (m)            <- lateral
    v_target_t: target speed (m/s)            <- longitudinal
    a_x_t:      target long. accel (m/s^2)    <- longitudinal
    phase_t:    ACCEL / CRUISE / DECEL        <- longitudinal state
  }

Since physical constraints couple both dimensions,
candidate k* selection MUST be a 5-dimensional joint evaluation.
```

```text
Joint evaluation in the External Evaluator:

  Fail criteria (safety constraints):
    - v_t^2 * |kappa_t| > A_Y_MAX              (lateral G exceeded)
    - |a_x_t| > A_X_MAX                        (longitudinal accel exceeded)
    - |delta_a_x_t / dt| > JERK_X_MAX          (longitudinal jerk exceeded)
    - |delta(v_t^2*kappa_t)/dt| > JERK_Y_MAX   (lateral jerk exceeded)

  Scoring (ranking among passing candidates):
    comfort_score = - w1*E[|a_y|] - w2*E[|a_x|]
                    - w3*E[|jerk_x|] - w4*E[|jerk_y|]
    progress_score = total_distance / total_time
    total = w_c * comfort_score + w_p * progress_score

A candidate that excels in only one direction will still fail or score low.
Only a candidate that balances both achieves a high total score.
```

### Structural Advantages of Joint Learning

```text
1. Curvature-Aware Speed Learning
   - Humans reduce speed "in anticipation of" upcoming curves (predictive control)
   - This behavior is computed jointly in the human brain
   - Decoupled: curve detection -> speed computation pipeline introduces delay
   - Joint: the network learns "decelerate now because there's a curve ahead"

2. Global Comfort Optimization
   - Reward: r -= w_jerk_x*|jerk_x| + w_jerk_y*|jerk_y|
   - Avoids situations where lateral and longitudinal jerk peak simultaneously

3. Meaningful Candidate Comparison
   - Candidate A: slightly longer but gentle curve -> maintain speed -> time-efficient
   - Candidate B: short route but sharp curve -> requires deceleration -> slower
   - Correct comparison is only possible with 5-dimensional joint evaluation

4. Implementation Simplicity
   - A single Forward Pass yields all information
   - No separate speed planner module to manage
   - No interface inconsistency bugs between lateral and longitudinal

```

---

## 6.12 Detailed Speed Profile Design

### Three-Level Constraints: Speed, Acceleration, Jerk

Human comfort constraints exist at three levels of time differentiation.

```text
Level 1: Speed v (m/s)
  - Below legal speed limit
  - Curvature tolerance: v <= sqrt(A_Y_MAX / |kappa|)
  - Car-following constraint: v <= IDM speed limit

Level 2: Acceleration a = dv/dt (m/s^2)
  - Longitudinal: |a_x| <= A_X_MAX (e.g., 2.0 comfortable, 5.0 emergency)
  - Lateral: |a_y| = v^2 * |kappa| <= A_Y_MAX (e.g., 2.5 m/s^2)

Level 3: Jerk j = da/dt (m/s^3)
  - Longitudinal: |j_x| = |delta_a_x / dt| <= JERK_X_MAX
  - Lateral: |j_y| = |delta(v^2*kappa) / dt| <= JERK_Y_MAX
  - Jerk limits prevent "clunk braking" and "snap steering"

ISO 2631 motion sickness reference values:
  Longitudinal jerk < 2.0 m/s^3: Comfortable
  Longitudinal jerk < 5.0 m/s^3: Acceptable
  Longitudinal jerk > 5.0 m/s^3: Uncomfortable, remediation needed
  Lateral jerk < 1.5 m/s^3: Comfortable (low motion sickness risk)
```

### Kinematic Stop Profile (Stop Line / Red Signal)

A jerk-limited stop profile that brings speed from v0 to zero at distance d_stop ahead.

```text
Goal:
  - v(d_stop) = 0 (stop at stop line precisely)
  - No overshoot or undershoot (stop error < STOP_POS_TOL)
  - Satisfy jerk limits (S-curve profile)

[Constant deceleration (simplest)]
  a_decel = v0^2 / (2 * d_stop)
  v(s) = sqrt(v0^2 - 2 * a_decel * s)

  Problem: If d_stop is short, a_decel > A_X_MAX -> jerk limit violated

[S-curve (3-phase) stop profile]
  Phase 1 (jerk ramp-up): a increases 0 -> -A_X_MAX, duration t1 = A_X_MAX/J_MAX
  Phase 2 (constant decel): a = -A_X_MAX maintained
  Phase 3 (jerk ease-out): a increases -A_X_MAX -> 0, v -> 0

  Minimum braking distance (shortest comfortable stop):
    d_min = v0^2 / (2 * A_X_MAX) + A_X_MAX * t1^2 / 3  (approx.)

  Decision flow:
    if d_stop >= d_min:
      Use S-curve profile (comfortable)
    elif d_stop >= d_emergency_min:
      Constant maximum deceleration (uncomfortable but safe, notify passengers)
    else:
      AEB trigger condition -> hand off to AEB module
```

```python
def compute_stop_profile(v0, d_stop, a_max, j_max, dt=0.1):
    """
    Generate jerk-limited stop profile.
    Returns: [(v_t, a_t, phase_t)] for t in [0, T]
    """
    t1 = a_max / j_max
    v1 = v0 - 0.5 * j_max * t1 ** 2
    d1 = v0 * t1 - j_max * t1 ** 3 / 6
    d3 = v1 ** 2 / (2 * a_max)
    d2 = d_stop - d1 - d3

    if d2 < 0:  # Distance too short: fall back to constant deceleration
        a_decel = min(v0 ** 2 / (2.0 * max(d_stop, 0.1)), a_max * 2.0)
        return constant_decel_profile(v0, d_stop, a_decel, dt)

    # 3-phase S-curve profile
    profile = []
    v, a = v0, 0.0
    for phase, d_phase in [('DECEL_RAMP', d1), ('DECEL', d2), ('DECEL_EASE', d3)]:
        pos = 0.0
        while pos < d_phase and v > 0:
            if phase == 'DECEL_RAMP':
                a = max(a - j_max * dt, -a_max)
            elif phase == 'DECEL_EASE':
                a = min(a + j_max * dt, 0.0)
            v = max(v + a * dt, 0.0)
            pos += v * dt
            profile.append((round(v, 4), round(a, 4), 'DECEL'))
    return profile
```

### IDM (Intelligent Driver Model) for Car-Following Speed Limit

```text
IDM (Treiber et al. 2000) adaptive following model:

  desired_gap(v, v_lead) = s0 + v * T
                           + v * (v - v_lead) / (2 * sqrt(a_comf * b_comf))

  Parameters:
    s0    : minimum standstill gap (e.g., 2.0 m)
    T     : time headway (e.g., 1.5 to 2.0 s)
    a_comf: comfortable acceleration (e.g., 1.5 m/s^2)
    b_comf: comfortable deceleration (e.g., 2.0 m/s^2)
    v     : ego speed
    v_lead: lead vehicle speed (0 when stopped)

  Following acceleration:
    a_idm = a_comf * [1 - (v / v_free)^4 - (desired_gap / actual_gap)^2]

    v_free    : free-road target speed (set at or below legal limit)
    actual_gap: lead vehicle distance from BEV Dynamic Head

  Behavior:
    actual_gap >> desired_gap -> free driving (v -> v_free)
    actual_gap ~= desired_gap -> constant headway following
    actual_gap < desired_gap  -> hard braking
    v_lead = 0 and actual_gap < desired_gap -> ego stops
```

```text
Usage of IDM in this design:
  Method A (clipping in Converter):
    acc_idm = compute_idm(v, v_lead, actual_gap)
    target_accel = min(planner_accel, acc_idm)
    target_speed = min(planner_speed, v + acc_idm * lookahead_time)

  Method B (state input to Planner):
    Add to CondFormer input features:
      - lead_gap_m          (from BEV)
      - lead_speed_mps      (from BEV)
      - relative_speed_mps  (= v - v_lead)
    -> Planner naturally learns IDM-like following behavior via imitation

  Recommended: Use both Method A and Method B.
  Planner learns appropriate following while Converter acts as a final safety backstop.
```

### Anticipatory Curve Speed Control

Look ahead along the trajectory for the curvature peak and begin deceleration in time.

```text
Scenario:
  - Current position is straight (kappa ~= 0)
  - 50 m ahead: curve peak kappa_peak = 0.05 (1/m)
  - A_Y_MAX = 3.0 m/s^2 -> v_curve = sqrt(3.0/0.05) = 7.75 m/s
  - Current speed v0 = 13.9 m/s (50 km/h)

Anticipatory deceleration:
  v_target = sqrt(A_Y_MAX / kappa_peak) = 7.75 m/s
  d_available = 50 m
  d_required  = (v0^2 - v_target^2) / (2 * A_X_MAX)
              = (193.2 - 60.1) / 4.0 = 33.3 m

  d_available (50m) > d_required (33.3m): comfortable decel with margin:
    a_x = -(v0^2 - v_target^2) / (2 * d_available) = -1.33 m/s^2

  If d_available < d_required:
    Maximum deceleration still not enough -> route change or re-plan at lower speed
    -> External Evaluator marks this candidate as Fail

What the Planner learns:
  - Human drivers reduce speed "in anticipation of" the upcoming curve
  - This predictive control is learned via IL and refined with RL
  - Curve curvature profile ahead is available from BEV Lane Topology
  - Even if learning is insufficient, External Evaluator corrects (fail-safe)
```

### S-Curve (Jerk-Limited) Acceleration/Deceleration Profiles

```text
Acceleration (launch / merge / overtake):
  Phase 1: j = +J_MAX, a ramps 0 -> +A_X_MAX  (duration t1 = A_X_MAX/J_MAX)
  Phase 2: a = +A_X_MAX maintained (v increases)
  Phase 3: j = -J_MAX, a ramps +A_X_MAX -> 0 (jerk ease-out)

Deceleration (stop / lead vehicle slowing):
  Phase 1: j = -J_MAX, a ramps 0 -> -A_X_MAX  (brake application)
  Phase 2: a = -A_X_MAX maintained (v decreases)
  Phase 3: j = +J_MAX, a ramps -A_X_MAX -> 0  (creep to stop)

Typical parameters (passenger car, comfortable class):
  A_X_MAX    = 2.0 m/s^2
  A_Y_MAX    = 2.5 m/s^2
  JERK_X_MAX = 2.0 m/s^3
  JERK_Y_MAX = 1.5 m/s^3

Robotaxi / high comfort class:
  A_X_MAX    = 1.5 m/s^2
  A_Y_MAX    = 2.0 m/s^2
  JERK_X_MAX = 1.0 m/s^3
  JERK_Y_MAX = 1.0 m/s^3

Emergency avoidance (safety first):
  A_X_MAX    = 8.0 m/s^2  (AEB domain)
  A_Y_MAX    = 5.0 m/s^2  (near tire friction limit)
  JERK_X_MAX = unlimited
  JERK_Y_MAX = unlimited
```

### Speed Phase Management (ACCEL / CRUISE / DECEL)

```text
Phase transition conditions (priority order):

  [DECEL - highest priority]
    - traffic_light_state == RED and stopline_distance < d_brake_start(v)
    - traffic_light_state == YELLOW and within stoppable distance
    - stopline_distance_m < d_brake_start(v)
    - actual_gap < desired_gap(v)
    - v > v_limit_curve = sqrt(A_Y_MAX / kappa_next_peak) (anticipatory)
    - v > legal_speed_limit

  [CRUISE - middle priority]
    - |v - target_speed| < 0.5 m/s
    - None of the DECEL conditions active
    - actual_gap ~= desired_gap

  [ACCEL - lowest priority]
    - v < target_speed - 0.5 m/s
    - actual_gap > desired_gap * 1.5
    - Within speed limit and lateral G limit

Once the DECEL flag is set, maintain DECEL even if the lead vehicle accelerates.
Return to ACCEL only after all DECEL conditions are cleared, with a hysteresis delay.
```

---

## 6.13 Data Collection Scale

```text
Phase 1 (Prototype):
  - Tens of thousands to hundreds of thousands of km
  - Representative scenes within the ODD
  - Quality-focused

Phase 2 (Productization):
  - Millions of km
  - Reinforce tail cases (rare scenarios)
  - Fleet collection

Phase 3 (Fleet Learning):
  - Tens of millions of km or more
  - Massive collection via Shadow Mode
  - Continuous improvement loop
```

---

## 6.14 Positioning as Structured E2E

```text
End-to-End (pure):
  sensors -> NN -> steering angle
  Advantages: Simple
  Disadvantages: No intermediate representations, difficult product verification

Structured E2E (this design):
  sensors -> BEV world -> trajectory candidates -> evaluator -> converter
  Advantages: Has intermediate representations, product-verifiable, vehicle-independent
  Disadvantages: Complex design, need to manage inter-module interfaces
```

This design, which could also be called "End-to-Mid," achieves both the ability to learn from human trajectories and the verifiability required for a product.

---

## 6.15 Chapter Summary

```text
Elements designed in this chapter:
  1. Limits of lane centerline following (5 scenarios)
  2. Human trajectory teacher approach
  3. Per-cycle, ego-frame target trajectory
  4. Responsibility separation from the Converter
  5. Design of using lane as Context (not Target)
  6. Method for building human trajectory labels
  7. Data quality management
  8. K-candidate + MHP Loss
  9. Speed planning / longitudinal teacher (signal stops, IL+RL, phase A-D)
  10. Stopline and following-gap stop precision
  11. Trajectory generation architecture: K-query VLA Planner (MVP) →
      DiffusionDrive (diffusion model, production evolution); §6.12 behaviors
      (stop, IDM, curve speed) learned via conditioning inputs (6.10)
  12. Joint lateral-longitudinal optimization principles and advantages (6.11)
  13. Detailed speed profile design: S-curve, IDM, anticipatory curve control (6.12)
  14. Data collection scale
  15. Positioning as Structured E2E
```

The next chapter details the Trajectory-to-Steering Converter that converts the selected trajectory into actual steering commands.
