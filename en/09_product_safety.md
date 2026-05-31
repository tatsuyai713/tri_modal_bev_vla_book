# Chapter 9 Productization, Safety, Regulations, ODD, and Logging

---

## 9.1 Exposing Required Intermediate Representations for a Product

In a pure black-box E2E system, it is impossible to explain "why this action was taken."  
As a product, the following information must be observable from outside the system.

```text
Required observable intermediate representations:
  1. BEV World (drivable area, occupancy, uncertainty)
  2. Lane Topology (lane graph)
  3. Dynamic World (moving object positions and velocities)
  4. K candidate trajectories
  5. Reason for selection
  6. ODD assessment result
  7. Sensor health status
```

With these observable:
- Which information was wrong can be identified in accident analysis
- The external evaluator can perform rule checks
- Evidence can be used in safety cases
- Performance regression after OTA updates can be detected

---

## 9.2 Safety-Biased Selection Among K Candidates

```text
External Evaluator selection logic:
  1. Run checklist against all K candidates
  2. Select the candidate with the highest confidence among those that Pass
  3. If all candidates Fail -> MRM (Minimum Risk Maneuver)

Checklist:
  - No collision with static occupancy
  - Dynamic Risk Map risk value < threshold
  - Within drivable area
  - Within speed limit
  - Acceleration and curvature within comfort range
  - Satisfies stop line and crosswalk constraints
  - Within ODD

MRM (Minimum Risk Maneuver):
  - Continue driving at safe speed, or perform safe stop
  - A module that requires human design and verification
```

---

## 9.3 Rationale for Separating the External Evaluator

```text
Reasons for separating the external evaluator from the neural network:
  1. Auditability: rules are explicit and easy to audit and verify
  2. Updateability: can be updated when regulations, regions, or ODDs change
     without retraining the neural network
  3. Safety case evidence: evidence remains that "the evaluator checked"
  4. Fallback design: the evaluator should function even if the Planner fails completely

Caution:
  - The external evaluator is not perfect either (it can have bugs too)
  - The evaluator logic also requires unit tests and scenario tests
  - An overly conservative evaluator leads to an "unusable system" (too-conservative problem)
```

---

## 9.4 Approach to Regulatory Compliance

**This book does not provide legal interpretation of regulations.** However, design considerations are described here.

### ISO 26262 (Functional Safety)

```text
Scope of ISO 26262:
  - Functional safety of electrical and electronic systems
  - ASIL (Automotive Safety Integrity Level) A–D

Relationship to this design:
  - Neural Networks cannot be directly ASIL-certified
    under the standard ISO 26262 architecture
  - Design the External Evaluator + Fallback Layer at ASIL-B or ASIL-D
  - Treat the Neural Network as QM (Quality Management) and wrap it with the evaluator
  - Sensor redundancy (Tri-modal) contributes to ASIL decomposition

Implementation guidance:
  - Run ASIL-D safety monitoring on a CPU/MCU independent of the Planner
  - The safety monitoring layer continuously verifies Planner output
  - Final vehicle control commands pass through the safety monitoring layer
```

### SOTIF (ISO 21448)

```text
Scope of SOTIF:
  - Risks arising from "insufficient intended functionality"
  - Sensor performance limits, model performance limits

Relationship to this design:
  - The BEV uncertainty head indicates "regions where the model lacks confidence"
  - ODD monitoring detects "out-of-operating-range" conditions
  - Detection of unknown scenes (OOD detection) promotes safe-side behavior

Implementation guidance:
  - Reduce speed in scenes with high uncertainty
  - Log and record novel scenes
  - Speed reduction on ODD-out detection -> MRM
```

### UNECE R157 (ALKS)

```text
Scope of UNECE R157:
  - Automated Lane Keeping System (ALKS)
  - International standard for Level 3 automated driving

Key requirements:
  - Minimum Risk Maneuver (MRM) capability
  - Driver takeover request
  - Maximum speed 130 km/h (highway only)
  - Recording requirements (Event Data Recorder)
  - Notification to following vehicles (hazard lights, etc.)

Relationship to this design:
  - MRM: Anytime Planning Levels 3–4 address this
  - Recording: Log design (9.12) addresses this
  - Speed: External Evaluator speed checks address this
```

### Regional Regulatory Differences

| Region | Key Regulations/Standards | Notes |
|---|---|---|
| EU | UNECE R157, EU AI Act | Type approval required, AI Act applies |
| Japan | Road Traffic Act revision, MLIT guidelines | L3 limited to highways |
| USA | FMVSS, NHTSA AV Guidance | Significant variation by state law |
| China | GB/T 38186, Intelligent Connected Vehicle Policy | Geofenced test zones |

```text
Compliance approach:
  - Add Region / Policy conditioning to CondFormer
  - Add region_code parameter to DSL
  - Manage speed limits and priority rules via region-specific tables
```

---

## 9.5 Tri-Modal Sensor Redundancy

```text
Each sensor operates on an independent physical principle:
  - Camera: optics
  - LiDAR: laser time-of-flight
  - Radar: microwave Doppler

Even under a single failure mode (e.g., heavy rain):
  - Camera: reduced visibility
  - LiDAR: reduced point cloud density
  - Radar: strong in bad weather (continues to operate)

-> Tri-modal provides redundancy against single-sensor failure
```

### Design Assurance for Sensor Redundancy

```text
Requirement: continue operation on any single-sensor failure (with speed restriction)
Verification: sensor dropout injection tests (both training and real vehicle)
Logging: record the dropped sensor and continued operation
Safety: define the estimation accuracy with only remaining sensors as an ODD condition
```

---

## 9.6 Modality Gate Monitoring

By monitoring the output of the modality gate, sensor health can be diagnosed.

```text
Values to monitor:
  gate_camera: average gate value
  gate_lidar:  average gate value
  gate_radar:  average gate value

Anomaly detection:
  gate < 0.1 for 5 or more frames -> sensor quality degradation warning
  gate < 0.05 -> suspected sensor failure -> ODD-out flag

Usage:
  - ODD assessment
  - Log recording
  - Dashboard display (during development)
```

---

## 9.7 Online Self-Diagnosis via BEV Output

Use BEV information for self-diagnosis.

```text
Diagnostic checks:
  - bev_drivable class at ego position is NOT_DRIVABLE(0) or MARGINAL(2)
    -> calibration anomaly or localization anomaly
       (ego position should be DRIVABLE(1) during normal driving)

  - bev_uncertainty is high around ego position
    -> sensor quality degradation or out-of-model scene

  - bev_agent_occ is uniformly high across the entire area
    -> sensor noise or model runaway

  - dynamic_risk_map is uniformly high risk
    -> dynamic prediction anomaly

Diagnostic results:
  - Notify ODD Monitor of anomaly flags
  - Record anomaly logs
  - Reduce speed or trigger MRM for serious cases
```

---

## 9.8 ODD (Operational Design Domain) Definition and Monitoring

The ODD is the "range of operating conditions" within which the system is designed to operate normally.

### ODD Definition Example (L2+)

```text
Road types: highways, expressways
Speed range: 0–130 km/h
Weather: clear, light rain (precipitation < 10mm/h)
Visibility: 100m or more
Time of day: day and night (except dense fog)
Region: (areas where map exists)
Sensors: all sensors nominal (or up to 1 sensor failure tolerated)
Localization: coarse absolute position accuracy sufficient to search nearby Map candidates
Ego pose: lane membership, lateral offset, and heading error on local Lane Graph
           at accuracy that allows controllable operation
```

### ODD Monitoring Module

```text
Monitoring items:
  - Coarse localization quality
  - Ego pose confidence on local Lane Graph
  - Sensor health (gate values)
  - Statistics of BEV uncertainty
  - Consistency score between Map candidates and BEV observations
  - Weather (estimated from sensors, or online weather data)
  - Speed
  - Complexity of intersection geometry

On ODD-out detection:
  - Level 1: speed reduction (20% decrease)
  - Level 2: significant speed reduction (50% decrease)
  - Level 3: MRM (safe stop)
  - Level 4: driver takeover request
```

---

## 9.9 Decomposition of Test Units

```text
Component tests:
  - SensorSyncCalib: timestamp accuracy test
  - CameraBEVEncoder: IoU accuracy (compared with LiDAR)
  - LiDARBEVEncoder: 3D detection accuracy
  - RadarBEVEncoder: Doppler velocity accuracy
  - ModalityGateFuser: behavior under sensor dropout
  - TemporalBEVMemory: ego motion warp accuracy
  - BEV Information Heads: accuracy metrics for each HEAD
  - Lane Topology: connectivity graph accuracy
  - Dynamic Object Branch: moving-object detection and velocity estimation accuracy
  - Agent Future Predictor: minADE, minFDE, MR
  - VLA Planner: ADE/FDE of K candidate trajectories vs. human trajectories
  - External Evaluator: rejection rate of unsafe trajectories
  - Trajectory Converter: accuracy of trajectory-to-steering-angle conversion

Integration tests:
  - End-to-end: sensor input to control command (latency, accuracy)
  - Continued operation test with multiple sensor dropouts
  - ODD-out detection test

Scenario tests:
  - Standard scenario set (see Chapter 11)
  - Tail-case scenarios
  - Region-specific scenarios
```

---

## 9.10 Confidence Calibration

The Planner confidence is a "preference" score, not a safety probability.  
However, calibrated confidence is useful information.

```text
Calibration check:
  - What fraction of candidates with confidence = 0.9 are "actually close to the human trajectory"
  - Reliability diagram (x-axis: confidence, y-axis: accuracy)
  - Perfect calibration lies on the 45-degree line

Calibration methods:
  - Temperature scaling (simplest)
  - Platt scaling
  - Isotonic regression

Caution:
  - Even calibrated confidence must NOT be used as a safety probability
  - It is solely a "selection criterion after the safety check by the External Evaluator"
```

---

## 9.11 Explainability Design

```text
Level 1: For engineers
  - Attention visualization (attention map on BEV)
  - Comparison of Confidence vs. Evaluator score
  - Intermediate outputs of each module

Level 2: For users (future)
  - T_scene_text (decoding of internal scene tokens)
  - Brief display of selection reason
  - Visualization of "why I am here now"

Level 3: For safety cases
  - Scene replay of accidents and near-misses
  - Record of each module's operating state
  - What the evaluator checked and why it made the selection
```

---

## 9.12 Log Design

```text
Per-frame log (Medium layer, 10–20Hz):
  timestamp_ns
  ego_x, y, lat, lon, heading
  speed, yaw_rate, accel_x, accel_y, steer_angle

  sensor_health: {camera: [], lidar: float, radar: float}
  gate_values: {camera: float, lidar: float, radar: float}

  bev_uncertainty_max, bev_uncertainty_mean
  lane_mismatch_score
  ood_flags: {speed: bool, weather: bool, localization: bool, ...}

  n_candidates: int
  selected_candidate_id: int
  selected_reason: str
  all_candidate_pass: [bool * K]

  target_curvature, target_speed
  feasibility_ok
  fallback_active
  fallback_level: int

  processing_time_ms

Event log (on condition trigger):
  ODD-out event
  Sensor failure event
  Fallback activation event
  Calibration request event
  Map mismatch event

Shadow Mode log (additional fields):
  human_trajectory: [T, D]
  system_trajectory: [T, D]
  trajectory_diff: float (ADE)
  system_candidate_human_chose: bool
```

---

## 9.13 Development Loop for Productization

```text
Stage 1: Offline Open-loop
  - Performance evaluation on log data
  - Metrics such as ADE/FDE, Collision Rate, etc.

Stage 2: Closed-loop Simulation
  - Verification in simulators such as CARLA, nuPlan

Stage 3: Shadow Mode
  - Run the system on a real vehicle without controlling it
  - Compare system output with the human's actual driving

Stage 4: Limited Actuator Test
  - Real-vehicle control test under speed and ODD restrictions

Stage 5: ODD-Limited Road Test
  - Real driving test within the defined ODD

Stage 6: Regression & OTA
  - Periodic performance verification
  - OTA updates if issues arise
```

---

## 9.14 Chapter Summary

```text
Elements designed in this chapter:
  1. Exposing required intermediate representations for a product
  2. K-candidate safety-biased selection and External Evaluator
  3. Rationale for separating the external evaluator
  4. Regulatory compliance (ISO 26262, SOTIF, UNECE R157)
  5. Regional regulatory differences (EU / Japan / USA / China)
  6. Tri-modal sensor redundancy
  7. Modality gate monitoring
  8. BEV output self-diagnosis
  9. ODD definition and monitoring
  10. Decomposition of test units
  11. Confidence calibration
  12. Explainability (3 levels)
  13. Log design
  14. Productization development loop
```

The next chapter details the implementation modules and interface design.
