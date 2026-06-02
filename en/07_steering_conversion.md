# Chapter 7 Converting the Selected Trajectory to Target Steering Angle

---

## 7.1 Role of the Converter

The Trajectory-to-Steering Converter (hereafter, Converter) is a deterministic converter that converts the target trajectory selected by the external evaluator into actual vehicle control commands (target curvature, target speed, steering angle).

```text
Role of the Converter:
  - Trajectory -> target curvature / target speed
  - The only module that knows the vehicle model (wheelbase, steering ratio, etc.)
  - Rule-based and deterministic (no learned parameters)
  - Can be unit-tested independently from the Planner
```

---

## 7.2 Difference from Conventional Tracking Controllers

```text
Conventional tracking controller:
  - Minimizes deviation from the target trajectory (lateral error, heading error)
  - Examples: PID control, Stanley Controller, LQR
  - Structure that "chases" the trajectory

This design's Converter:
  - Looks at the trajectory and computes the target curvature ahead (lookahead type)
  - Instead of "chasing," it "looks ahead and prepares"
  - This distinction is prominent at high speeds and when approaching curves
```

---

## 7.3 Input/Output Interface

```text
Inputs:
  selected_trajectory:  [T, D]  (one trajectory selected by the external evaluator)
  ego_state:            speed, yaw_rate, steer_angle, timestamp
  vehicle_params:       wheelbase, steer_ratio, steer_range, ...

Outputs:
  target_curvature:     float32, 1/m (curvature)
  target_speed:         float32, m/s
  target_accel:         float32, m/s^2
  speed_phase:          enum (ACCEL / CRUISE / DECEL)
  target_steer_angle:   float32, rad (optional, vehicle specific)
  limit_steer:          bool (steering angle limit flag)
  limit_accel:          bool (acceleration limit flag)
  feasibility_ok:       bool (feedback to Planner)
  fallback_active:      bool
```

---

## 7.4 Processing Flow (1 Cycle)

```python
def control_cycle(selected_traj, ego_state, vehicle_params):
    """1 control cycle processing"""
    
    # Step 1: Trajectory validity check
    if not validate_trajectory(selected_traj):
        return fallback_command()
    
    # Step 2: Timestamp correction
    # Warp to compensate for ego drift due to processing delay
    aligned_traj = compensate_latency(selected_traj, ego_state)
    
    # Step 3: Calculate lookahead distance
    lookahead_dist = calc_lookahead(ego_state.speed)
    
    # Step 4: Get lookahead point
    lookahead_point = get_point_at_distance(aligned_traj, lookahead_dist)
    
    # Step 5: Estimate curvature
    kappa = estimate_curvature(aligned_traj, method='pure_pursuit')
    
    # Step 6: Smooth curvature
    kappa_smooth = alpha_filter(kappa, prev_kappa, alpha=0.3)
    
    # Step 7: Lateral acceleration limit
    # Planned: a_y_plan = v^2 * kappa, measurement-based: a_y_meas = v * yaw_rate
    a_y_plan = ego_state.speed**2 * kappa_smooth
    a_y_meas = ego_state.speed * ego_state.yaw_rate
    if max(abs(a_y_plan), abs(a_y_meas)) > A_Y_MAX:
      kappa_smooth = sign(kappa_smooth) * A_Y_MAX / max(ego_state.speed**2, 0.01)
    
    # Step 8: Convert to steering angle via bicycle model
    steer_angle = bicycle_model_to_steer(kappa_smooth, vehicle_params)
    
    # Step 9: Steering angle / rate limit
    steer_angle = apply_steer_limit(steer_angle, vehicle_params.steer_range)
    steer_angle = apply_rate_limit(steer_angle, prev_steer, vehicle_params.steer_rate_max)
    
    # Step 10: Log
    log_control_cycle(kappa_smooth, steer_angle, ego_state, ...)
    
    speed_cmd = resolve_target_speed(aligned_traj, ego_state, speed_constraints)
    accel_cmd = resolve_target_accel(aligned_traj, ego_state, speed_constraints)
    accel_cmd = apply_jerk_limit(accel_cmd, prev_accel, JERK_MAX)

    return ControlCommand(
        target_curvature=kappa_smooth,
        target_speed=speed_cmd.target_speed,
        target_accel=accel_cmd.target_accel,
        speed_phase=accel_cmd.phase,
        target_steer_angle=steer_angle,
        feasibility_ok=True
    )
```

---

## 7.5 Lookahead Distance Calculation

Lookahead distance is determined as a function of speed.

```text
lookahead_dist = speed * lookahead_time + lookahead_min

Example:
  lookahead_time = 0.5s
  lookahead_min  = 3.0m
  speed = 10 m/s -> lookahead = 10 * 0.5 + 3.0 = 8.0m
  speed = 30 m/s -> lookahead = 30 * 0.5 + 3.0 = 18.0m

Reason:
  - At high speed, earlier response is needed (look further ahead)
  - At low speed, looking at a closer point is sufficient
  - Maintain minimum distance even when stopped (to prevent division by zero)
```

---

## 7.6 Curvature Estimation Methods

### Pure Pursuit Method (Most Common)

```python
# Target curvature by Pure Pursuit
dx = lookahead_point.x  # x of lookahead point (forward)
dy = lookahead_point.y  # y of lookahead point (lateral)
ld = sqrt(dx**2 + dy**2)
kappa = 2.0 * dy / ld**2
```

### Three-Point Arc Method

```python
# Fit a circle to three points: current point, midpoint, lookahead point
kappa = circle_curvature(p0=(0,0), p1=mid_point, p2=lookahead_point)
```

### Polynomial Fit Method

```python
# Fit trajectory to y = a*x^2 + b*x + c -> compute curvature
coeffs = np.polyfit(traj_x, traj_y, 2)
a = coeffs[0]
kappa = 2.0 * a  # approximate curvature
```

### Recommendations

```text
Low speed (~10 m/s): Pure Pursuit is stable
Medium speed (10~20 m/s): 3-point arc or Pure Pursuit
High speed (20 m/s~): Polynomial fit (smoother)

Sanity check via LaneNode.curvature_profile:
  Compare kappa computed by the Converter from the trajectory with
  the LaneNode.curvature_profile value of the lane the ego is traveling.
  A large discrepancy suggests one of the following:
    - Positional offset on BEV due to sensor misperception
    - Estimation error in HD Map / Lane Topology
  Discrepancy threshold: log |kappa_traj - kappa_lane| > 0.05 [1/m],
  raise bev_uncertainty when > 0.15 [1/m] persists.
```

---

## 7.7 Steering Angle Conversion via Bicycle Model

Conversion from curvature to steering angle is done with a bicycle model.

```text
Bicycle model:
  steer_angle = arctan(wheelbase * kappa)

Where:
  wheelbase: front-rear axle distance (m) (e.g., 2.8m)
  kappa: target curvature (1/m)
  steer_angle: front wheel steering angle (rad)

Conversion to EPS (Electric Power Steering) input:
  steer_input = steer_angle * steer_ratio + steer_offset

Where:
  steer_ratio: steering ratio (e.g., 14.0)
  steer_offset: center correction
```

### Steering Calibration

```text
In actual vehicles, "steer_angle = 0 when driving straight" may not hold (center offset).
Periodically measure the center correction value and update the calibration table.
```

---

## 7.8 Multi-Rate Design

While the Planner runs at 10~20 Hz, control output requires 50~100 Hz.

```text
Multi-rate design:
  Planner:    10-20 Hz -> saves selected trajectory to buffer
  Converter:  50-100 Hz -> uses buffered trajectory for control computation each cycle

Between new Planner trajectory updates:
  - Use the last selected trajectory warped by Ego motion
  - Warp: transform the previous frame's trajectory to align with current Ego position
  - Using trajectories up to 100~200ms old is OK (physical inertia)
```

---

## 7.9 Trajectory Validity Check

Check for when the Planner outputs an abnormal trajectory.

```text
Check items:
  1. No NaN / Inf
  2. Trajectory length is sufficient (minimum 3 points)
  3. First point is near the ego vehicle (xy < 1.0m)
  4. Distance between consecutive points is physically possible (speed < v_max)
  5. No sudden direction changes (yaw change / step < pi/4)
  6. Monotonically increasing timestamps

On anomaly detection:
  - Return fallback_command
  - fallback_active = True
  - Feedback to Planner
  - Log recording
```

---

## 7.10 Feasibility Feedback

When the Converter judges that "this trajectory cannot be realized," it feeds back to the Planner.

```text
Feedback signals:
  feasibility_ok:      bool
  infeasible_reason:   str  (e.g., "steer_rate_exceeded", "accel_too_large")
  feasibility_margin:  float (how much it is exceeded)

Use for Planner:
  - Increase penalty during training for candidates where feasibility_ok = False
  - Consider in the next External Evaluator selection
```

---

## 7.11 Options for Inverse Vehicle Model

When the bicycle model is too approximate, use a more accurate inverse vehicle model.

```text
Option 1: Lookup table
  - Create a table of (speed, curvature) -> steer_angle experimentally
  - Can include nonlinear characteristics
  - Requires periodic updates as it changes with temperature and tire wear

Option 2: Bicycle model + correction (recommended in this design)
  - Basics use the bicycle model
  - Add a correction term for residuals (discrepancy with actual yaw_rate)
  - Online estimation with Kalman Filter, etc.

Option 3: Small neural network
  - Learn the (speed, kappa) -> steer_angle conversion
  - Train on actual vehicle driving data
  - Use with caution in products due to reduced interpretability
```

---

## 7.12 Integration with Longitudinal Control (Speed Planning)

This chapter's Converter primarily handles lateral direction (steering angle), but also integrates with longitudinal control.

```text
Longitudinal control inputs:
  target_speed: obtained from the speed profile output by the Planner
  target_accel: differential of the speed profile

Longitudinal control outputs:
  throttle_cmd: accelerator opening
  brake_cmd:    brake pressure
  target_torque: torque (for electric vehicles)

Following leading vehicle:
  target_speed = min(planner_speed, ACC_target_speed)
  ACC_target_speed: radar-based leading vehicle speed limit

Signal response:
  stopline_dist: distance to stop line (from BEV)
  target_speed = v_profile_for_stop(stopline_dist, current_speed)
```

### Rule-Based Priority for Speed Constraints

```text
Input sources:
  1) Legal speed limits (map/sign/road-marking fused)
  2) User GUI overspeed tolerance
  3) Comfort and vehicle limits (a_x, a_y, jerk)
  4) Lead-vehicle following cap (ACC)

Decision flow:
  base_limit = min(valid_map_limit, valid_sign_limit, valid_road_mark_limit)
  user_margin = clamp(user_overspeed_tolerance, 0, policy_margin_max)
  legal_cap = min(base_limit, base_limit + user_margin)
  comfort_cap = speed_from_curvature(kappa, a_y_max)
  follow_cap = acc_following_cap(lead_vehicle_state)
  final_target_speed = min(planner_speed, legal_cap, comfort_cap, follow_cap)

  target_accel converges to final_target_speed, and speed_phase is derived from
  the sign of target_accel (ACCEL / CRUISE / DECEL).
```

### Lateral-G, Yaw-Rate, and Jerk Constraints

Even when the Planner is learned, the final vehicle commands must be monitored as physical quantities. The Converter evaluates lateral acceleration using both trajectory curvature and measured yaw rate.

```text
Lateral acceleration:
  a_y_plan = v_target^2 * kappa_target
  a_y_meas = ego_speed * ego_yaw_rate
  a_y_eval = max(|a_y_plan|, |a_y_meas|)

Constraint:
  a_y_eval <= A_Y_MAX
  Example: comfort range 2.0-3.0 m/s^2, with a separate emergency cap
```

```text
Longitudinal jerk:
  jerk_x = (target_accel - prev_target_accel) / dt

Lateral jerk:
  jerk_y = (a_y_eval - prev_a_y_eval) / dt

Constraints:
  |jerk_x| <= JERK_X_MAX
  |jerk_y| <= JERK_Y_MAX

Processing:
  - Smooth target_accel with a jerk limit
  - Smooth kappa / steer_angle with rate limits
  - Reduce target_speed if constraints are still exceeded
  - If still infeasible, set feasibility_ok=False and return to the External Evaluator
```

```python
def apply_dynamics_constraints(kappa, target_speed, target_accel, ego_state, dt):
    a_y_plan = target_speed**2 * kappa
    a_y_meas = ego_state.speed * ego_state.yaw_rate
    a_y_eval = max(abs(a_y_plan), abs(a_y_meas))

    if a_y_eval > A_Y_MAX:
        target_speed = min(target_speed, sqrt(A_Y_MAX / max(abs(kappa), 1e-4)))

    target_accel = clamp_rate(
        target_accel,
        prev_value=prev_target_accel,
        max_rate=JERK_X_MAX,
        dt=dt,
    )

    jerk_y = (a_y_eval - prev_a_y_eval) / dt
    if abs(jerk_y) > JERK_Y_MAX:
        kappa = clamp_rate(kappa, prev_kappa, KAPPA_RATE_MAX, dt)

    return kappa, target_speed, target_accel
```

---

## 7.13 Values to Log

```text
Log per control cycle:
  timestamp
  selected_traj_id
  lookahead_dist
  lookahead_point_x, y
  raw_kappa
  smoothed_kappa
  target_steer_angle
  actual_steer_angle (EPS feedback)
  steer_error = target - actual
  target_speed
  target_accel
  speed_phase
  actual_speed
  a_y = speed^2 * kappa
  a_y_meas = speed * yaw_rate
  jerk_x
  jerk_y
  feasibility_ok
  fallback_active
  infeasible_reason (if any)
  processing_time_ms
```

---

## 7.14 Chapter Summary

```text
Elements designed in this chapter:
  1. Role of the Converter and difference from conventional controllers
  2. Input/output interface
  3. Processing flow for 1 cycle (10 steps)
  4. Speed-dependent lookahead distance calculation
  5. Three curvature estimation methods (Pure Pursuit / 3-point arc / polynomial)
  6. Steering angle conversion via bicycle model
  7. Multi-rate design (Planner 10-20Hz, Converter 50-100Hz)
  8. Trajectory validity check
  9. Feasibility feedback
  10. Options for inverse vehicle model
  11. Integration with longitudinal control
```

The next chapter details weight reduction and real-time design for actual vehicle application.
