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
```

---

## 6.11 Data Collection Scale

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

## 6.12 Positioning as Structured E2E

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

## 6.13 Chapter Summary

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
  9. Speed planning / longitudinal teacher
  10. Data collection scale
  11. Positioning as Structured E2E
```

The next chapter details the Trajectory-to-Steering Converter that converts the selected trajectory into actual steering commands.
