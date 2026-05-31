# Chapter 11 Implementation Roadmap and Shadow Mode Verification

---

## 11.1 MVP Build Order

Attempting to implement all features from the start has a high probability of failure. Expand incrementally from a minimal implementation.

```text
Step 1: BEVFusion Backbone (Camera + LiDAR)
  - Camera BEV Encoder + LiDAR BEV Encoder
  - Simple concatenation (no Gate)
  - Evaluate Occupancy/Detection on nuScenes

Step 2: BEV Information Heads
  - Static World heads (drivable, occupancy, lane)
  - Visualization and evaluation on BEV

Step 3: K-query Planner (no Language)
  - Train with human trajectory labels
  - Start with K=4
  - Evaluate ADE/FDE

Step 4: External Evaluator (static collision check only)
  - Reject unsafe trajectories
  - Confirm safe candidate selection

Step 5: Add Radar Branch
  - Radar BEV Encoder
  - Modality Gate
  - Confirm improvement in dynamic object velocity estimation via Radar

Step 6: Add Language Conditioning
  - Language input at DSL level (turn right, go straight, etc.)
  - Train CondFormer
  - Confirm trajectory change under language conditioning

Step 7: Add Internal Scene Tokenizer
  - Distillation training via VLM Teacher
  - Confirm effect of T_scene
```

---

## 11.2 Product-Oriented Incremental Features

```text
Phase A (Foundation):
  - BEVFusion warm start
  - BEV Heads
  - K-query Planner
  - Static Evaluator

Phase B (Dynamic):
  - Dynamic Object Branch
  - Agent Future Predictor
  - Dynamic Risk Map
  - Dynamic Evaluator

Phase C (Language):
  - Language Conditioning (DSL)
  - Scene Tokenizer
  - CondFormer

Phase D (Productization):
  - ODD Monitor
  - Fallback Manager
  - Log system
  - Shadow Mode

Phase E (Expansion):
  - Map-less degradation
  - Multi-region support
  - Fleet Learning pipeline
  - Continual Learning
```

---

## 11.3 Eight-Phase Roadmap

| Phase | Name | Goal |
|---|---|---|
| Phase 0 | Data and Label Validation | Verify training data quality |
| Phase 1 | Offline Open-loop Evaluation | Establish metrics and performance baseline |
| Phase 2 | Log Replay + External Evaluator | Verify safety selection logic |
| Phase 3 | Closed-loop Simulation | Comprehensive evaluation in simulator |
| Phase 4 | Shadow Mode | Collect system output on real vehicle |
| Phase 5 | Limited Actuator Test | Real vehicle control under restricted conditions |
| Phase 6 | ODD-Limited Road Test | Real driving within defined ODD |
| Phase 7 | Regression + OTA | Continuous performance monitoring and updates |

---

## 11.4 Phase 0: Data and Label Validation

```text
Check items:
  □ Is the ego frame transformation of LiDAR point clouds correct?
  □ Does Camera calibration agree with BEV?
  □ Is the coordinate system of human trajectory labels correct?
  □ Is the speed profile computation correct?
  □ Do label timestamps align?
  □ Verify class balance (dynamic vs. static, scenario types)
  □ Results of data quality filtering
  □ Verify fleet regional distribution
  □ Verify proportion of adverse weather and nighttime data
  □ Verify proportion of signal and intersection scenes
  □ Verify proportion and content of tail-case scenes
  □ Manual sample check of validation data (100+ records)
```

---

## 11.5 Phase 1: Offline Open-loop Evaluation

```text
Evaluation metrics:
  Planner:
    - minADE@K: ADE of the best candidate among K
    - minFDE@K: FDE of the best candidate among K
    - MR (Miss Rate): fraction exceeding threshold
    - Collision Rate (static): collision rate with static occupancy
    - Collision Rate (dynamic): collision rate with moving objects
    - Comfort: mean and max of |a_x|, |a_y|

  BEV Heads:
    - IoU (drivable, occupancy, lane)
    - BEV detection AP (agent)

  Agent Future:
    - minADE, minFDE, MR (per category)
    - NLL (Negative Log-Likelihood)

Evaluation by scenario bucket:
  - Straight driving (highway)
  - Straight driving (urban)
  - Right turn / left turn
  - Lane change
  - Parked vehicle avoidance
  - Pedestrian crossing
  - Intersection
  - Adverse weather
```

---

## 11.6 Phase 2: Log Replay + External Evaluator

```text
What is Log Replay:
  - Use recorded real driving logs as input and inspect candidate trajectories
    and evaluation results output by the system
  - The vehicle is not actually controlled (logs are replayed as-is)

Checks:
  - Does the External Evaluator select the correct candidate?
  - Does Fallback activate in unsafe scenes?
  - Does ODD-out detection work correctly?
  - Are logs output correctly?

Differential analysis:
  - Compare system-selected trajectory vs. human actual driving
  - Are candidates that the Evaluator marked "Fail" truly unsafe?
  - Is the trajectory the human actually drove included in the candidates?
```

---

## 11.7 Phase 3: Closed-loop Simulation

```text
Recommended simulators:
  CARLA (Open-source):
    - Realistic 3D environment based on Unreal Engine
    - Sensor models (Camera, LiDAR, Radar)
    - Traffic Manager (behavior of other vehicles)
    - Python API
    URL: https://github.com/carla-simulator/carla

  nuPlan (Waymo/nuScenes family):
    - Reactive simulation based on real driving logs
    - Strong for planning evaluation
    - Reference implementations such as PDM-Closed available
    URL: https://github.com/motional/nuplan-devkit

  SUMO:
    - Traffic simulation
    - Can be integrated with CARLA
    URL: https://sumo.dlr.de

Difference between Open-loop and Closed-loop:
  Open-loop: replay recorded data, ego actions are not reflected in the environment
  Closed-loop: ego actions affect the environment, other vehicles also react

  Important: Good open-loop evaluation does not guarantee good closed-loop results
  (other vehicles' reactions, situations created by the ego's own actions)
```

---

## 11.8 Phase 4: Shadow Mode

Shadow Mode is the phase where the system runs on a real vehicle but control is performed by a human, and the system's output is collected and analyzed.

### What to Verify in Shadow Mode

```text
□ Does the system's selected trajectory roughly match the human's actual driving?
□ How does the system output in scenes where the human performs special maneuvers?
□ Is the external evaluator excessively triggering "Fail"?
□ Is Fallback misfiring?
□ Accuracy of ODD-out detection
□ Are logs being recorded correctly?
□ Is sensor health monitoring functioning?
□ Is processing time within the target?
□ Diversity of candidate trajectories (are K candidates too similar?)
□ Response to language instructions (trajectory change with DSL input)
□ BEV quality in adverse weather and nighttime
□ Response to parked vehicles and pedestrians
```

### Classification of Shadow Mode Differences

```text
Benign:
  - System is behaving slightly more conservatively than the human
  - Minor lateral position differences
  -> Room for quality improvement, not urgent

Suspicious:
  - System trajectory differs significantly from the human's
  - Evaluator is Failing almost all candidates
  - Speed profile differs significantly
  -> Review detailed logs, consider model improvements

Critical:
  - System outputs a trajectory heading toward an obstacle
  - ODD-out condition not detected
  - Planner outputs NaN for all candidates
  -> Immediately investigate root cause; must fix before starting real vehicle control
```

---

## 11.9 Gate Conditions Before Starting Real Vehicle Control

```text
Gate conditions to verify in Shadow Mode:
  □ minADE@K < 0.5m (highway scenes)
  □ Collision Rate (static) < 0.1%
  □ ODD-out detection rate > 95%
  □ Processing time < 50ms (95th percentile)
  □ Fallback false activation rate < 5%
  □ Critical differences = 0 (most recent 1,000km)
  □ Sensor dropout test: confirm continued operation with 1 sensor missing
  □ Speed restriction: confirm normal Converter operation at test speed
  □ All External Evaluator unit tests pass
  □ All Trajectory Converter unit tests pass
```

---

## 11.10 Failure Rollback Mapping

Pre-define which phase to roll back to when issues arise.

| Issue Type | Roll Back To |
|---|---|
| Significant model accuracy degradation | Phase 1 (Open-loop re-evaluation) |
| Evaluator misjudgment | Phase 2 (Log Replay) |
| Deviation from simulation environment | Phase 3 (Closed-loop fix) |
| Shadow Mode Critical difference | Phase 4 (continue Shadow Mode) |
| Processing time overrun on real vehicle | Phase 5 (re-test after lightweight optimization) |
| Performance issues outside ODD | Phase 1 (revise ODD definition) |

---

## 11.11 Regression Testing

Regression tests for performance verification after OTA updates.

```text
Test layers:
  Layer 1: Offline metrics
    - Metrics on nuScenes or custom dataset
    - Block if worse than baseline

  Layer 2: Scenario test suite
    - Automated execution of standard scenario set (100+ scenarios)
    - All must pass

  Layer 3: Shadow Mode comparison
    - Compare Shadow Mode logs of the latest vs. previous version
    - Verify no degradation in important scenes

  Layer 4: Real-vehicle smoke test
    - Real vehicle test on specific route
    - Pass of baseline scenario set
```

---

## 11.12 Development Loop (10-Step Cycle)

```text
1. Identify problem (log and Shadow Mode analysis)
2. Identify root cause (data? architecture? implementation bug?)
3. Plan the fix
4. Implement and train
5. Offline evaluation (equivalent to Phase 1)
6. Log Replay check (equivalent to Phase 2)
7. Scenario tests
8. Confirm in Shadow Mode
9. Regression tests
10. OTA update or data feedback
-> Return to step 1
```

---

## 11.13 Scenario Coverage Definition

```text
Scenario dimensions:
  Road type: highway / arterial road / urban / residential / parking lot
  Intersection type: T-junction / crossroads / roundabout / dedicated right turn / none
  Weather: clear / cloudy / rain / fog / night / snow
  Sensor state: all nominal / LiDAR failure / partial camera failure / Radar failure
  Traffic density: low / medium / high / congested
  Surrounding agents: none / lead vehicle / vehicle alongside / pedestrian / bicycle / large vehicle
  Special: construction / parked vehicle / fallen object / waiting to turn right / cut-in
  Language instruction: none / turn right / turn left / change lane / stop
```

---

## 11.14 Shadow Mode Log Analysis Tool

Tool design for efficient analysis of Shadow Mode logs.

```python
# Shadow Mode differential analysis (overview)
def analyze_shadow_diff(log_files, threshold_m=0.5):
    results = []
    for log in log_files:
        human_traj = log.human_trajectory
        system_traj = log.system_trajectory
        
        ade = compute_ade(system_traj, human_traj)
        
        if ade > threshold_m * 2:
            severity = "critical"
        elif ade > threshold_m:
            severity = "suspicious"
        else:
            severity = "benign"
        
        results.append({
            "timestamp": log.timestamp,
            "ade": ade,
            "severity": severity,
            "scenario_type": log.scenario_type,
            "fallback_active": log.fallback_active,
        })
    
    return results, summarize_by_scenario(results)
```

---

## 11.15 Chapter Summary

```text
Elements designed in this chapter:
  1. MVP build order (7 steps)
  2. Product-oriented incremental features (Phases A–E)
  3. Eight-phase roadmap
  4. Phase 0: Data and label validation (12 check items)
  5. Phase 1: Offline open-loop evaluation (metrics and scenario breakdown)
  6. Phase 2: Log Replay + External Evaluator
  7. Phase 3: Closed-loop simulation (CARLA, nuPlan)
  8. Phase 4: Shadow Mode (check items and difference classification)
  9. Gate conditions before starting real vehicle control
  10. Failure rollback mapping
  11. Regression testing (4 layers)
  12. Development loop (10 steps)
  13. Scenario coverage definition
  14. Shadow Mode log analysis tool
```

The next chapter describes the design of data collection, fleet learning, and annotation pipelines (newly added chapter).
