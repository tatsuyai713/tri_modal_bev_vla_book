# Chapter 5 Language Conditioning and VLA Planner

---

## 5.1 Why Input Language into the Planner?

Language instructions are not used only as a "conversation interface"; they are used as "behavior conditioning."

```text
Approach A (explanation generation):
  - The system explains in language why it acted as it did
  - It gives the user reassurance, but does not affect behavior
  - Because the explanation is post-hoc, faithfulness is difficult to guarantee

Approach B (behavior conditioning, used in this design):
  - Language instructions enter Planning as an input
  - An instruction such as "please turn right" becomes a condition that actually generates a right-turn trajectory
  - Behavior and language are learned in an integrated way
```

This design adopts Approach B. Language is a conditioning variable for behavior.

---

## 5.2 Separating External Language and Internal Scene Language

Language is divided into two types.

```text
External Language (T_inst):
  - Instructions from the user or navigation system
  - Examples: "Turn right at the next intersection", "Do not change lanes"
  - Input: Text Encoder -> T_inst [B, N_inst, D]

Internal Scene Language (T_scene):
  - A scene summary generated autonomously by the system
  - Examples: "There is a pedestrian on the right", "A cut-in may occur ahead"
  - Input: BEV features -> Scene Tokenizer -> T_scene [B, N_scene, D]
```

With this separation, scene understanding continues to function even when external instructions are ambiguous. Conversely, when the scene is simple, external instructions can still be prioritized.

---

## 5.3 Text Encoder and Command Encoder

### Three Levels of Language Encoding

```text
Level 1: Identifier (minimum cost)
  - action_id: int (RIGHT_TURN=1, CHANGE_LANE=2, ...)
  - Embedding(action_id) -> T_inst
  - Product recommendation: when structured commands are available from navigation

Level 2: Structured command (DSL)
  - {action: "turn", direction: "right", distance_m: 100.0}
  - LinearProjection(DSL features) -> T_inst
  - Product recommendation: combination of navigation and speech recognition

Level 3: Natural language (maximum flexibility)
  - "Please turn right at the next intersection"
  - Text Encoder (CLIP/T5) -> T_inst
  - Use cases: testing, research, future conversational UI
```

### Notes on Natural Language Processing

```text
Natural language is prone to NLU (natural language understanding) errors.
In a product system, natural language should not be used directly for safety control.
Instead, it should be converted to a DSL first (intent classification + parameter extraction).

Example:
  Input: "Stop by that convenience store"
  NLU: {action: "stop", location: "nearby_poi", poi_type: "convenience_store"}
  -> DSL -> Planner

### Speed Constraint Inputs (Map, Perception, and User GUI)

Besides language tokens, speed planning should consume explicit speed-constraint tokens.

```text
T_speed_map (navigation map):
  - legal_speed_limit_mps
  - school_zone_speed_limit_mps
  - construction_zone_speed_limit_mps
  - map_confidence

T_speed_env (perception):
  - sign_speed_limit_mps
  - road_mark_speed_limit_mps
  - detection_confidence
  - ttl_frames

T_user_pref (user GUI):
  - overspeed_tolerance_mps
  - profile_mode (eco / normal / assertive)
  - hard_cap_enabled
```

```text
Design rules:
  - Never exceed the legal limit.
  - User tolerance is allowed only as an operational margin within policy.
  - Conflicts among map/sign/road-marking are resolved conservatively,
    then finalized by the external evaluator.
```
```

---

## 5.4 CondFormer Design

CondFormer integrates BEV tokens, external language (T_inst), and internal scene tokens (T_scene).

```text
CondFormer:
  Input:
    - q_planner:   Planner query tokens [B, K, D]
    - BEV tokens:  [B, H*W, D]
    - T_inst:      [B, N_inst, D]
    - T_scene:     [B, N_scene, D]

  Layer 1: Cross-attention over BEV tokens
  Layer 2: Cross-attention over T_inst
  Layer 3: Cross-attention over T_scene
  Layer 4: Self-attention (integration)
  Layer 5: Instruction Priority Gate

  Output:
    - conditioned query tokens [B, K, D]
```

### Instruction Priority Gate

The weight of T_inst is dynamically amplified so that strong external instructions, such as turning right or stopping, are reliably reflected.

```python
# Instruction priority gate (outline)
inst_cls = T_inst[:, 0, :]  # CLS token
gate = torch.sigmoid(self.gate_linear(inst_cls))  # [B, 1]
gated_out = (1.0 + gate.unsqueeze(1)) * lang_cross_attn_out
```

---

## 5.5 K-Query VLA Planner Design

Using a Transformer Decoder structure, the planner generates K candidate trajectories in parallel.

```text
Input Tokens to Decoder:
  - q_planner:    Initial queries [B, K, D] (learnable parameters)
  - BEV tokens:   Spatial information
  - Agent tokens: Dynamic agents
  - Future risk:  Dynamic Risk Map tokens
  - CondFormer:   Language and scene conditions
  - Ego context:  Speed, yaw_rate, etc.

Decoder (4-6 layers):
  Each layer:
    - Self-attention over K queries
    - Cross-attention over BEV (static world)
    - Cross-attention over Agent future tokens
    - Cross-attention over Dynamic Risk tokens
    - Cross-attention over CondFormer output
    - Speed profile head (v, a_x, phase)
    - FFN
```

### Planner Output

```text
trajectories: [B, K, T, D]
  - D = (x, y, yaw, v) or (x, y, heading, speed, curvature)
  - T = 8-16 timesteps
  - K = 12-24 candidates

confidence: [B, K]
  - Learned "preference" score for each candidate
  - Not a safety probability (safety is judged by the external evaluator)

speed_profiles: [B, K, T, 3]
  - (v_target, a_x_target, phase) per timestep
  - phase in {ACCEL, CRUISE, DECEL}
  - Generated jointly with trajectories and sent to the Converter
```

### Meaning of Confidence

```text
confidence != safety probability
confidence = how close the model predicts this candidate is to a human trajectory

-> Safety is checked separately by the external evaluator
-> confidence must not be used as a safety guarantee
```

### Evolution Architecture: DiffusionDrive (§6.10)

```text
The K-query Planner is well-suited for MVP/Phase 1 due to its implementation
simplicity and low computational cost. After confirming stable operation,
consider migrating to DiffusionDrive when the following conditions are met:

  K-query Planner strengths:
    - Generates K candidates in a single forward pass; simple to implement
    - MHP Loss is intuitive and easy to debug
    - Standard Transformer architecture; smaller GPU memory footprint

  Conditions to consider migration to DiffusionDrive:
    - Rare scenarios not covered by K fixed candidates arise frequently
    - Need to improve longitudinal precision (stop profiles, IDM, curve speed)
    - Richer candidate diversity desired for External Evaluator safety selection

  See §6.10 for the full DiffusionDrive design.
```

---

## 5.6 Candidate Trajectory Output Format

```text
Each point in each candidate trajectory:
  t:           Timestamp (seconds, relative to the current time)
  x:           Forward displacement (m)
  y:           Lateral displacement (m, left is positive)
  yaw:         Heading angle change (rad)
  v:           Target speed (m/s)

Coordinate system:
  - Based on the current ego frame
  - Recomputed at each frame (not accumulated from the previous frame)
  - This makes it vehicle-agnostic
```

### Speed Profile Consistency

```text
The speed profile of each candidate trajectory should satisfy the following constraints:
  - Acceleration between points is within physically feasible limits
  - Speed converges to 0 at locations where stopping is required (traffic lights, stop lines)
  - Lateral acceleration in curves satisfies roughly a_y = v^2 * kappa <= 3 m/s^2 (comfort)
  - Relative speed and following distance to the lead vehicle are appropriate
  - It does not exceed advisory_speed_mps from the lookahead lane
    (curvature-based advisory speed obtained from LaneNode)
```

---

## 5.7 Connection to the External Evaluator

```text
The External Evaluator evaluates the K candidates:

Inputs:
  - K candidate trajectories
  - Static World (drivable, occupancy, stopline, crosswalk)
  - Traffic light tokens (state, confidence, controlled_lane_ids)
  - Dynamic Risk Map
  - Agent Futures
  - ODD status
  - confidence [B, K]
  - speed constraints from map/sign/road-marking/user settings

Checks:
  1. Static collision check (overlap with non-drivable area or occupied area)
  2. Dynamic collision check (overlap with Dynamic Risk Map)
  3. Lane violation check (outside drivable area)
  4. Speed limit check (legal speed vs speed profile)
  5. Advisory speed check (LaneNode.advisory_speed_mps vs speed profile)
  6. Comfort check (trajectory curvature vs lane curvature profile, lateral-G, acceleration, jerk limits)
  7. Traffic light check (whether signal state is consistent with maneuver direction)
  8. Stopline precision check (whether stopping occurs within the allowed zone before the stop line)
  9. Following stop gap check (whether stopped gap to the lead vehicle is within target range)
  10. ODD check (localization accuracy, sensor health)
  11. Final rule-based speed check

Final rule-based speed check:
  legal_limit = min(valid_map_limit, valid_sign_limit, valid_road_mark_limit)
  user_margin = clamp(user_overspeed_tolerance, 0, policy_margin_max)
  operational_cap = min(legal_limit, legal_limit + user_margin)
  Any candidate with v_target(t) > operational_cap is marked Fail.

Dynamics comfort check:
  a_y_plan(t) = v_target(t)^2 * kappa(t)
  a_y_meas(t) = ego_speed(t) * ego_yaw_rate(t)
  jerk_x(t) = delta a_x(t) / delta t
  jerk_y(t) = delta a_y(t) / delta t
  Candidates exceeding lateral-G or jerk limits are marked Fail or heavily down-scored.

Signal / stop precision check:
  On RED or without a valid directional ARROW, candidates crossing the stop line are Fail.
  On YELLOW, stop if within stopping distance; otherwise avoid harsh braking inside the intersection.
  stop_position_error = planned_stop_x - target_stop_x
  Candidates with |stop_position_error| outside tolerance are down-scored or marked Fail.
  When following a stopped lead vehicle, candidates with actual_stop_gap below desired_stop_gap are Fail.

Selection logic:
  - Among candidates that pass all checks, select the one with the highest confidence
  - If no candidate passes all checks -> MRM (Minimum Risk Maneuver)
```

---

## 5.8 Why Output a Trajectory Instead of Direct Steering Angle

```text
Reasons not to output steering angle (delta) directly:

1. Vehicle independence
   - Even for the same planned trajectory, steering gain and wheelbase differ by vehicle type
   - If the planner outputs a trajectory, each vehicle's Converter can convert it to the proper steering angle

2. Checkability by the external evaluator
   - A trajectory has clearer physical meaning than a steering angle for evaluator checks
   - "This trajectory collides" can be stated, but "this steering angle collides" is harder to evaluate

3. Consistency with human trajectory learning
   - The teacher signal is the trajectory from human driving logs
   - Steering angle requires an indirect conversion through a vehicle model

4. Comparison in Shadow Mode
   - When comparing system output with actual human driving in Shadow Mode,
     a trajectory-based output is easier to understand visually
```

---

## 5.9 Planner Loss Design

```text
Loss = L_traj + L_conf + L_smooth + L_comfort + L_risk

L_traj: Error against human trajectory (best-of-K)
  - ADE/FDE of the closest mode k
  - sum_t || pred_k* - gt ||_2

L_conf: Make the best mode high confidence
  - BCE(confidence[k*], 1.0) for best k
  - BCE(confidence[k!=k*], 0.0) for others
  - Or cross-entropy

L_smooth: Trajectory smoothness
  - sum_t || pred_t+1 - 2*pred_t + pred_t-1 ||_2 (second difference)

L_comfort: Limits on lateral acceleration and longitudinal acceleration
  - ReLU(|a_y| - a_y_max)^2
  - ReLU(|a_x| - a_x_max)^2

L_risk: Soft collision penalty with Dynamic Risk Map
  - sum_t risk_t(pred_kt_x, pred_kt_y)
```

---

## 5.10 MVP Configuration

Minimum configuration for building a product prototype in a short period.

```text
Planner:
  - Layers: 4
  - K: 8-12
  - T: 8-10
  - D: 2 or 4 (x, y, yaw, v)

CondFormer:
  - Layers: 3
  - Language level: 2 (DSL)
  - T_scene: N_scene = 4

BEV tokens to Planner:
  - Important regions only in Route Corridor +/- 3 m, not the entire BEV
  - N_tokens = 500-2000

External evaluator:
  - Static collision check (required)
  - Dynamic Risk check (recommended)
  - Speed and comfort checks
```

---

## 5.11 Debug Outputs and Explainability

```text
Debug outputs:
  T_scene_text:  Readable text decoded from internal scene tokens
                 Example: "pedestrian crossing ahead / right turn required"

  top_k_reasons: Selection reasons for the top 3 candidates
                 Example: ["safe, matches route", "safe but suboptimal comfort",
                      "rejected: static collision"]

  attention_viz: CondFormer attention map (which BEV cells it attends to)

  conf_vs_rank:  Comparison between confidence and evaluator score
```

These debug outputs are used both for development analysis and for building the safety case.

---

## 5.12 Domain-Specific Language (DSL) Design for Language Instructions

In product systems, language instructions are processed through structured commands (DSL) rather than using natural language directly.

### DSL Specification Example

```json
{
  "command_type": "navigation",
  "action": "turn",
  "direction": "right",
  "trigger": {
    "type": "intersection",
    "distance_m": 150.0,
    "confidence": 0.92
  },
  "constraints": {
    "max_speed_kmh": 30,
    "caution_level": "normal"
  }
}
```

### Benefits of DSL

```text
1. Removing ambiguity
   - The natural-language instruction "please turn right" is ambiguous about where to turn
   - DSL can make the location and conditions explicit

2. Adding safety constraints
   - max_speed and caution_level can be set by the control side through DSL

3. Verifiability
   - DSL syntax can be checked for correctness
   - Natural language cannot provide this property directly

4. Multilingual support
   - Natural language requires language-specific processing for Japanese, English, Chinese, etc.
   - DSL is language-independent
```

### NLU-to-DSL Conversion

```text
User: "Turn right at that convenience store"
    ↓ NLU (Intent Detection + Entity Extraction)
DSL: {
  "command_type": "navigation",
  "action": "turn",
  "direction": "right",
  "trigger": {
    "type": "poi",
    "poi_type": "convenience_store",
    "distance_m": 50.0
  }
}
    ↓ Level-2 Command Encoder
T_inst -> Planner
```

---

## 5.13 Multi-Step Planning and Hierarchical Planner

The design in this chapter generates a one-step trajectory, but long-distance planning should be hierarchical.

```text
Global Planner (Route level):
  - Plans the route to the destination (kilometer scale)
  - Input: coarse GPS position + map candidates + destination
  - Output: route waypoints, road segment candidates

Tactical Planner (100 m scale):
  - The VLA Planner in this chapter
  - Input: BEV world + language + local route + local lane topology
  - Output: K candidate trajectories (several seconds ahead)

Reactive Layer (immediate avoidance):
  - Reacts to sudden obstacles
  - Handled by the external evaluator and Converter
  - Or by a dedicated AEB/avoidance layer
```

The VLA Planner in this design corresponds to the Tactical Planner.  
Road segment candidates from the Global Planner are matched with current observations in the Lane Topology Branch, then input to CondFormer as a local Lane Graph and `target_lane_sequence`.
The Reactive Layer is handled by the External Evaluator and Converter.

---

## 5.14 Chapter Summary

```text
Elements designed in this chapter:
  1. Policy of using language as behavior conditioning
  2. Separation of external language (T_inst) and internal scene language (T_scene)
  3. Three levels of language encoding (identifier / DSL / natural language)
  4. CondFormer (4-5 layers with instruction priority gate)
  5. K-query VLA Planner Decoder
  6. Candidate trajectory output format (including speed profile)
  7. Connection to the External Evaluator
  8. Reason for outputting trajectories
  9. Planner loss design (5 terms)
  10. MVP configuration
  11. DSL design and NLU-to-DSL conversion
  12. Hierarchical Planner (Global / Tactical / Reactive)
```

The next chapter details the design of human trajectory teachers and lane-center-free Planning.
