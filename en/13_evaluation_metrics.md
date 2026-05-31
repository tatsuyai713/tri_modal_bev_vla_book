# Chapter 13 Evaluation Metrics and Benchmark Design

---

## 13.1 The Importance of Evaluation Design

Precisely defining what to measure determines the direction of development. Optimizing the wrong metric does not improve actual performance.

```text
Common failures:
  - Focusing on reducing Open-loop ADE -> Closed-loop collision rate does not improve
  - Achieving #1 on nuScenes benchmark -> cannot drive normally in real vehicle
  - Improving average value -> rare cases deteriorate
  - Metric improves -> practical performance is unchanged (wrong metric selection)

Countermeasures:
  - Choose metrics after understanding their meaning
  - Evaluate from multiple viewpoints (metrics)
  - Break down analysis by scenario
  - Ultimately incorporate human evaluation
```

---

## 13.2 Perception System (BEV Head) Evaluation Metrics

### Occupancy / Drivable Area

$$\text{IoU} = \frac{TP}{TP + FP + FN}$$

| Metric | Target | Meaning |
|---|---|---|
| mIoU (drivable 3-class) | BEV drivable label (3 classes) | Overall accuracy of drivability classes |
| IoU (DRIVABLE) | bev_drivable class=1 | Match for normal drivable area |
| IoU (MARGINAL) | bev_drivable class=2 | Match for drivable but not recommended area |
| IoU (occupancy) | BEV static occupancy | Match for static obstacles |
| IoU (lane) | BEV lane center | Match for lanes |
| mIoU | Average over all classes | Overall BEV quality |

### 3D Object Detection (Agent Detection)

$$\text{mAP} = \frac{1}{|C|} \sum_{c} AP_c$$

| Metric | Meaning |
|---|---|
| mAP | Mean of Average Precision per class |
| NDS (nuScenes Detection Score) | mAP + integrated velocity, orientation, and size estimation |
| ATE (Average Translation Error) | Position error [m] |
| ASE (Average Scale Error) | Size error (1 - IoU_3D) |
| AOE (Average Orientation Error) | Orientation error [rad] |
| AVE (Average Velocity Error) | Velocity error [m/s] |
| AAE (Average Attribute Error) | Attribute error (stopped/moving, etc.) |

---

## 13.3 Prediction System (Agent Future) Evaluation Metrics

### Trajectory Prediction

$$\text{ADE} = \frac{1}{T} \sum_{t=1}^{T} \| \hat{p}_t - p_t \|_2$$

$$\text{FDE} = \| \hat{p}_T - p_T \|_2$$

$$\text{minADE}_k = \mathbb{E} \left[ \min_{i \in [1,k]} \text{ADE}^{(i)} \right]$$

$$\text{minFDE}_k = \mathbb{E} \left[ \min_{i \in [1,k]} \text{FDE}^{(i)} \right]$$

$$\text{MR} = \frac{\text{Number of samples where all predictions exceed threshold}}{\text{Total samples}}$$

| Metric | Description | Typical Target |
|---|---|---|
| ADE | Average distance error over all timesteps | < 1.0 m |
| FDE | Distance error at final timestep | < 2.0 m |
| minADE@6 | Best ADE among 6 candidates | < 0.7 m |
| minFDE@6 | Best FDE among 6 candidates | < 1.5 m |
| MR | Fraction where all candidates exceed threshold | < 10% |

---

## 13.4 Planning System (Planner) Evaluation Metrics

### Trajectory Quality

| Metric | Calculation | Meaning |
|---|---|---|
| L2 error (1s/2s/3s) | L2 between predicted trajectory and human trajectory | Closeness to human driving |
| Collision Rate | Fraction where trajectory overlaps an obstacle | Safety |
| Off-route Rate (Hard) | Fraction entering NOT_DRIVABLE(0) area | Intrusion into undrivable area |
| Off-route Rate (Soft) | Fraction entering MARGINAL(2) area | Intrusion into not-recommended area |
| Comfort (\|a_x\|) | Longitudinal acceleration mean and max | Ride comfort |
| Comfort (\|a_y\|) | Lateral acceleration mean and max | Ride comfort |
| Comfort (jerk) | Jerk (rate of change of acceleration) | Ride comfort |

### Safety Metrics

| Metric | Meaning |
|---|---|
| TTC (Time-to-Collision) | Time to collision; fraction below threshold |
| THW (Time Headway) | Time distance to leading vehicle |
| Fallback Rate | Fraction of time Fallback was activated |

---

## 13.5 Correspondence with Benchmarks

### nuScenes

```text
URL: https://www.nuscenes.org/
Data: 1,000 scenes, 23 object categories, 6 cameras, LiDAR, Radar
Evaluation: Detection NDS, Tracking AMOTA, Prediction ADE/FDE, Planning
Correspondence with this design:
  - BEV Head: evaluable with nuScenes Detection NDS
  - Agent Future: evaluable with nuScenes Prediction
  - Planner: compare with IDM / human driving
  - Limitation: scene diversity is less than production data
```

### nuPlan

```text
URL: https://github.com/motional/nuplan-devkit
Data: 1,500 hours, Boston/Singapore/Pittsburgh/Las Vegas
Evaluation: Reactive closed-loop simulation
Metrics:
  - Ego-Progress: route progress
  - Ego-Expert: closeness to human driving
  - Comfort: acceleration
  - Collision: collision rate
  - TTC: time gap
Correspondence with this design:
  - Can be used as reference for Phase 3 Closed-loop Simulation
  - Reference implementations such as PDM-Closed are available
```

### NAVSIM

```text
URL: https://github.com/autonomousvision/navsim
Feature: Non-reactive simulation (other vehicles follow their recorded trajectories)
Metric: PDMS (Planning Distribution Matching Score)
  - NC (No-collision): no collision
  - DAC (Drivable Area Compliance): compliance with drivable area
    * NAVSIM DAC checks compliance with DRIVABLE(1) area. MARGINAL(2) intrusion is evaluated with reduced penalty
  - EP (Ego Progress): progress
  - TTC: time gap
  - Comfort: comfort
Correspondence with this design:
  - Suitable for Phase 1 evaluation
  - Simpler than Closed-loop as it is Non-reactive
```

### Bench2Drive

```text
URL: https://github.com/Thinklab-SJTU/Bench2Drive
Feature: CARLA-based closed-loop, 44 scenarios
Metric: Driving Score (RC + IS)
  - RC (Route Completion): route completion rate
  - IS (Infraction Score): violation penalty
Correspondence with this design:
  - Usable for Phase 3 Closed-loop evaluation
  - Enables near-real-vehicle evaluation
```

---

## 13.6 Differences Between Open-loop and Closed-loop Evaluation

```text
Problems with Open-loop evaluation:
  - Regardless of what the ego vehicle does, other vehicles continue following their recorded trajectories
  - If the ego vehicle deviates even slightly, the log continues as if the ego is suddenly off-road
  - Cannot evaluate behavior in genuinely dangerous situations

Example:
  Moved right to avoid a stopped vehicle ahead -> other vehicles' reactions remain as in the original log
  In reality this action would affect other vehicles, but Open-loop cannot evaluate this

Closed-loop evaluation:
  - Other vehicles react to the system's actions
  - Closer to reality
  - However, simulator fidelity (Sim-to-Real gap) is a concern

Recommendation:
  Evaluate with both. Use Open-loop for baseline performance, Closed-loop for overall behavior.
  Open-loop numbers alone are not sufficient to declare "good".
```

---

## 13.7 Shadow Mode-Specific Evaluation Metrics

In Shadow Mode, comparison with real driving is possible, enabling more direct metrics.

| Metric | Calculation | Meaning |
|---|---|---|
| Shadow ADE | ADE between system trajectory and actual driving | Overall agreement |
| Shadow FDE | Difference in final position | Directional agreement |
| Suspicious Rate | Fraction with Suspicious delta | Fraction of scenes requiring review |
| Critical Rate | Fraction with Critical delta | Fraction requiring immediate action |
| Fallback Rate | Fallback activation rate | Degree of over-triggering of safety circuit |
| Speed Profile Match | Correlation of speed profiles | Agreement of acceleration/deceleration patterns |
| Evaluator Reject Rate | Rate at which Evaluator rejected all candidates | Over-rejection by evaluator |

---

## 13.8 Evaluation of Language Conditioning

Evaluation specific to this system with language conditioning.

```text
Language instruction following rate:
  - DSL input "turn right" -> does output trajectory turn right?
  - DSL input "lane change (right)" -> does output trajectory change to right lane?
  - DSL input "stop" -> is output trajectory in stopping direction?

Evaluation method:
  - Prepare 50+ scenarios for each DSL command
  - Use rules to determine whether the trajectory heads in the intended direction
  - Also check the magnitude of trajectory change when language is varied

Rejection rate evaluation:
  - When given physically impossible instructions (e.g., "lane change right" on a narrow road)
  - Are all candidates Fallback?
  - Is a safe trajectory output while ignoring the impossible instruction?
```

---

## 13.9 Scenario Coverage and Per-Scenario Score

Evaluate not only by overall average but also broken down by scenario.

```text
Scenario classification:
  Basic scenarios:
    - Highway straight
    - Urban straight
    - Right turn
    - Left turn
    - Lane change (right/left)
    
  High-difficulty scenarios:
    - Parked vehicle avoidance
    - Pedestrian crossing
    - Bicycle crossing
    - Irregular intersection
    - Re-departure from signal stop
    
  Tail cases:
    - Construction zone
    - Faded lane markings
    - Fog / nighttime
    - Sensor loss
    - Stop-and-go in congestion

Reporting method for evaluation:
  - Report with per-scenario metrics table
  - Make weaknesses visible that overall averages would obscure
  - Check per-scenario changes before and after improvements
```

---

## 13.10 Regression Test Scenario Set Design

```text
Scenario set composition:
  - 100-200 scenarios
  - Balanced coverage of scenario classifications
  - Always include scenes that previously had issues

Scenario formats:
  A. Simulation scenarios
    - Scripted scenes in CARLA etc.
    - Fully reproducible
    
  B. Log Replay scenarios
    - Specific segments of real driving logs
    - Close to reality as they use actual sensor data

Update policy:
  - Add to scenario set whenever a new problem is discovered
  - However, too many scenarios leads to long test durations
  - Quarterly review: consolidate and prune older scenarios
```

---

## 13.11 Incorporating Human Evaluation

Supplement with human evaluation to capture quality that metrics alone cannot capture.

```text
Evaluation methods:
  A. Side-by-side evaluation
    - Present outputs of model A and B side by side for evaluators to choose
    - "Which is more natural?" "Which is safer?"
    
  B. Scenario review
    - Expert evaluators assess scenes collected in Shadow Mode
    - "Is this behavior acceptable / does it need improvement?"
    
  C. Expert ride
    - Expert drivers evaluate in a real vehicle
    - Rate "comfort", "confidence", and "naturalness" on a 5-point scale

Notes on human evaluation:
  - Ensure evaluator consistency (prepare guidelines)
  - Verify agreement among multiple evaluators
  - Counter evaluator fatigue and habituation
  - Combine with Shadow Mode
```

---

## 13.12 Chapter Summary

```text
Elements designed in this chapter:
  1. Importance of evaluation design and common failure patterns
  2. Perception system (BEV Head) evaluation metrics
  3. Prediction system (Agent Future) evaluation metrics (formulas for ADE/FDE/MR, etc.)
  4. Planning system (Planner) evaluation metrics
  5. Correspondence with major benchmarks (nuScenes, nuPlan, NAVSIM, Bench2Drive)
  6. Differences between Open-loop and Closed-loop and key caveats
  7. Shadow Mode-specific evaluation metrics
  8. Evaluation of language conditioning
  9. Scenario coverage and per-scenario score
  10. Regression test scenario set design
  11. Incorporating human evaluation
```

The next chapter covers the hardware platform, quantization, and OTA deployment.
