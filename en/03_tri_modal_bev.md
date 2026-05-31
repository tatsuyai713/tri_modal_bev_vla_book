# Chapter 3 Tri-modal BEV World Model

---

## 3.1 Sensor Synchronization and Calibration

The accuracy of sensor fusion depends directly on the accuracy of synchronization and calibration. No matter how sophisticated the fusion architecture is, accuracy degrades if input timestamps are misaligned or if extrinsic parameters are incorrect.

### Time Synchronization

```text
Recommended: IEEE 1588 PTP (Precision Time Protocol)
Accuracy target: 1 ms or less
Issue: Camera often uses GPU-side hardware timestamps while LiDAR uses controller-side timestamps
Mitigation: Software timestamp correction + ego motion interpolation
```

### Camera Extrinsic Parameters

```text
camera_to_ego transformation matrix [4, 4]:
  - Units: meters, radians
  - Estimation method: checkerboard calibration
  - Required accuracy: position < 1 cm, angle < 0.1 degree
  - Note: periodic re-calibration due to temporal drift from vehicle vibration
```

### LiDAR-Camera Extrinsic

```text
lidar_to_camera calibration:
  - Methods: test target or automatic target-less
  - Accuracy: under 1 cm (guideline)
  - The nuScenes calibration format is a useful reference
```

### Radar Extrinsic

```text
radar_to_ego transformation matrix:
  - Radar is typically fixed and embedded in the bumper
  - Yaw offset is particularly important (direct cause of azimuth errors)
  - Ego velocity correction (ego velocity subtraction) is mandatory
```

### Calibration Expiration Detection

```text
Monitor the following alerts:
- Misalignment between LiDAR point cloud and camera segmentation in BEV
- Positional offset between radar targets and camera objects
- Statistical drift in offset after temperature changes
-> Raise a calibration request flag when a threshold is exceeded
```

---

## 3.2 Camera BEV Encoder

### Processing Flow

```text
Multi-view images [B, N_cam, C, H_img, W_img]
    |
    v
Backbone (Swin-T, ResNet50, EfficientDet, etc.)
    | Per-camera features [B, N_cam, C_feat, H_feat, W_feat]
    v
Neck (FPN)
    | Multi-scale features [B, N_cam, C_neck, H', W']
    v
View Transformer (LSS/BEVFormer style)
    | Camera BEV feature [B, C_bev, H_bev, W_bev]
    v
```

### View Transformer Selection

| Method | Approach | Strengths | Weaknesses |
|---|---|---|---|
| LSS (Lift-Splat-Shoot) | Depth prediction -> point cloud -> pillar scatter | Intuitive, fast | Dependent on depth accuracy |
| BEVFormer | Deformable Attention, 2D reference from BEV queries | Flexible, easy to integrate temporal | High training cost |
| GKT | Frustum sampling + transformer | Memory efficient | Complex implementation |
| BEVPoolv2 | pre-computation + scatter | Fast inference | Requires pre-computation |

This design uses a BEVFormer-style or LSS+Transformer hybrid as the baseline.

### Backbone Selection

```text
Lightweight: EfficientNet-B2, MobileNetV3, GhostNet
Standard:    ResNet-50, ResNet-101
High-accuracy: Swin-B, InternImage
Recommended: ImageNet or NuImages pretrained -> fine-tune on BEV
```

---

## 3.3 LiDAR BEV Encoder

### PointPillars-Based Processing

```text
LiDAR points [B, N_sweep, N_pts, D]
    |
    v
Voxelization (voxel_size = [0.1m, 0.1m, 0.2m], etc.)
    | pillars [B, N_pillars, N_pts_per_pillar, D]
    v
Pillar Feature Network (PFN)
    | pillar features [B, N_pillars, C_pillar]
    v
Scatter (pillars to BEV grid)
    | LiDAR pseudo image [B, C_pillar, H_bev, W_bev]
    v
2D Backbone (2nd conv stage)
    | LiDAR BEV feature [B, C_lidar, H_bev, W_bev]
    v
```

### Use of Sparse Convolution

Point clouds are sparse, and sparse convolution can be more efficient than dense 2D convolution in some cases.

```text
Recommended: spconv (SubMConv3d, SparseConv3d)
Usage examples: CenterPoint, VoxelNet, SECOND
Configurations that use Sparse 3D conv before projecting to BEV are also possible
```

### Multi-Sweep Stacking

Stacking multiple sweeps is effective for distinguishing stationary objects from dynamic ones.

```text
Sweep stacking:
  N_sweeps = 3 to 5
  Add a time offset feature to each sweep: [x, y, z, intensity, delta_t]
  delta_t = current_time - sweep_time
  This causes dynamic objects to exhibit motion-blur-like differences in features
```

---

## 3.4 Radar BEV Encoder

### Characteristics of Radar

```text
What Radar excels at:
  - Relative velocity of dynamic objects (Doppler)
  - Operation in adverse weather (rain, fog, sandstorms)
  - Detection of metallic objects at long range

Weaknesses of Radar:
  - Low spatial resolution
  - Ghost targets (multi-path reflections, clutter)
  - No height information (in most cases)
  - Limited shape information
```

### Radar BEV Encoder Implementation

```text
Method A: Pillar approach (similar to LiDAR)
  - Scatter radar_points into pillars on BEV
  - Use velocity information (vx, vy) as additional channels
  - Relatively fast and easy to fuse with LiDAR

Method B: Tensor approach (project Range-Doppler to BEV)
  - Some Radars output a Range-Azimuth-Doppler tensor
  - Learn a transformer to convert directly to BEV
  - More information but higher processing cost

Recommended in this design: Method A (Pillar approach)
  Input features: [x, y, vr_x, vr_y, rcs, t_offset]
```

### Ego Speed Correction

Radar Doppler observes "relative velocity," which includes ego speed. To convert to world frame velocity, ego speed must be subtracted.

```python
# Radar ego velocity correction (outline)
vr_world_x = radar_point.vr_x + ego_state.speed * cos(ego_yaw)
vr_world_y = radar_point.vr_y + ego_state.speed * sin(ego_yaw)
```

---

## 3.5 Tri-modal BEV Fusion with Modality Gate

Simple summation or concatenation cannot handle sensor dropout.
A modality gate enables dynamic weighting based on the reliability of each sensor.

### Modality Gate Structure

```text
Gate(m) = sigmoid(linear(quality_m))
  m in {camera, lidar, radar}
  quality_m: sensor quality score (SNR, point cloud density, image exposure, etc.)

Fused BEV = Gate(camera) * F_cam + Gate(lidar) * F_lid + Gate(radar) * F_rad
```

### Sensor Quality Scores

```text
Camera quality:
  - exposure: detect overexposure and darkness
  - blur score: detect blur and raindrops
  - coverage: check if camera view is normal

LiDAR quality:
  - point_density: number of points per unit area
  - range_coverage: point cloud density by distance
  - blockage: occlusion (dirt, mounting angle offset)

Radar quality:
  - target_count: number of detected targets (suspect when too few)
  - interference_flag: detect interference from other radars
```

### Behavior Under Sensor Dropout

```text
LiDAR failure:
  - Gate(lidar) -> 0 (automatic shutoff)
  - Continue with Camera BEV and Radar BEV
  - Notify external evaluator that long-range accuracy and shape accuracy are degraded

Partial camera failure:
  - Apply feature mask for affected cameras
  - Increase uncertainty in corresponding BEV regions

Radar dropout:
  - Gate(radar) -> 0
  - Reliability of dynamic object velocity information decreases
  - Adjust velocity-based evaluation in external evaluator to be more conservative
```

---

## 3.6 Temporal BEV Memory

A single-frame BEV alone cannot capture the following information.

```text
Missing from single-frame BEV:
  - Object motion (velocity, acceleration)
  - Objects that appear stationary but are moving
  - Objects that disappeared (returning from temporary occlusion)
  - Midway through a lane change
  - Traffic light state transitions
```

Integrating temporal BEV can compensate for these gaps.

### Temporal Memory Implementation Choices

| Method | Approach | Characteristics |
|---|---|---|
| ConvGRU | Apply GRU to BEV features | Simple implementation, efficient |
| BEVFormer Temporal Attention | Reference past-frame BEVs via self-attention | Flexible, high accuracy |
| Sparse Temporal Token | Retain and reuse only important BEV tokens | Computationally efficient |

### Ego Motion Warp

The previous frame's BEV must be warped to match the current frame using ego motion.

```python
# BEV warp (outline)
# prev_bev: previous-frame BEV (B, C, H, W)
# ego_delta: change in ego position from previous to current frame (dx, dy, dyaw)

# Create an affine transform matrix for the BEV grid
theta = ego_delta_to_affine(dx, dy, dyaw)  # (B, 2, 3)
warped_prev_bev = F.grid_sample(prev_bev, theta, mode='bilinear')
```

### How Many Frames to Keep

```text
L = 3 to 5 frames is standard
  - Too few (L=1): dynamic object velocity estimation accuracy drops
  - Too many (L>6): computation cost and influence of stale information increase
  - At 10 Hz operation with L=4: 400 ms of temporal context
  - At 20 Hz operation with L=4: 200 ms of temporal context
```

---

## 3.7 Building BEV Tokens for the Planner

BEV features tend to be high-resolution (200x200 = 40,000 tokens), and passing all of them to the Planner would be too costly.

### Token Reduction Policy

```text
Method A: Sparse Pooling
  - Reduce unimportant tokens by average pooling
  - Select using importance scores (Motion Salience and uncertainty)

Method B: Route-aware Sampling
  - Retain BEV tokens near the route at high density
  - Sample outside the route at coarser resolution

Method C: Hierarchical BEV
  - Short range (0-30 m): high resolution
  - Medium range (30-80 m): medium resolution
  - Long range (80 m+): low resolution

This design: B + C combination is recommended
  - Target number of Planner input tokens: approximately 2,000 to 4,000
```

---

## 3.8 BEV Information Heads

Multiple types of information needed for planning are estimated in parallel from BEV features.

### Drivable Area Head

`bev_drivable` is defined not as a binary passability map but as a **3-class segmentation**. Following the same concept as Tesla FSD's drivable area visualization, a third class explicitly represents regions that can be physically entered but should not normally be driven on.

Example: The area before a curb is non-drivable (abrupt height change), but the sidewalk beyond the curb can be entered from a separate entrance without a curb, and is physically flat. This "sidewalk" part corresponds to the MARGINAL class.

```text
bev_drivable: [B, H, W]  <- class label at inference (argmax)
raw logits:  [B, 3, H, W] <- output during training / loss computation

Class definitions:
  0: NOT_DRIVABLE  = Not drivable
       Walls, high steps, guardrails, obstacles, etc.
       Regions that cannot be physically entered

  1: DRIVABLE      = Drivable (normal driving area)
       Road lanes, travel lanes, inside intersections, etc.
       Regions that should be used in normal driving

  2: MARGINAL      = Physically passable but should not be driven on
       Shoulders, sidewalks beyond steps, open space adjacent to parking areas, etc.
       Regions that are physically flat and enterable, but should not normally be driven on
       May be used in emergency avoidance or special situations (obstacle avoidance, emergency escape)

Loss: cross-entropy loss (3-class, with class weights)
  weight: NOT_DRIVABLE=1.0, DRIVABLE=1.0, MARGINAL=3.0 (emphasize the rare class)

Ground truth generation:
  DRIVABLE: road area from HD Map
  NOT_DRIVABLE: buildings/off-road from HD Map + abrupt steps (>15 cm) detected by LiDAR
  MARGINAL: shoulder/sidewalk from HD Map + area between DRIVABLE/NOT_DRIVABLE
           Gray-zone with step height 5 to 15 cm is also included in MARGINAL
```

#### Handling in the External Evaluator
```text
Treatment of trajectories on MARGINAL class:
  - Not a Hard Fail, unlike NOT_DRIVABLE
  - Penalize by lowering evaluation score (soft penalty)
  - Do not select candidates passing through MARGINAL when DRIVABLE candidates exist
  - All DRIVABLE candidates fail due to collision, etc. -> select MARGINAL candidate (emergency avoidance)
```

### Occupancy Head

```text
bev_occupancy: [B, H, W]
  - Values: [0, 1] continuous (occupancy probability) or binary
  - Represents static object occupancy
  - Dynamic objects are handled separately by bev_agent_occ
```

### Lane Head

```text
bev_lane: [B, H, W, C]
  - C=1: lane existence probability
  - C=2: left edge / right edge
  - C=3: lane type (solid / dashed / boundary)
  - Loss: segmentation loss
```

### Agent Occupancy Head

```text
bev_agent_occ: [B, H, W]
  - BEV cells occupied by dynamic objects
  - Includes pedestrians, cyclists, and vehicles
  - Separate from static bev_occupancy
```

### Agent Velocity Field Head

```text
bev_agent_vel: [B, H, W, 2]
  - Velocity (vx, vy) of dynamic objects at each BEV cell
  - Has no meaning in regions without occupancy (mask processing applied)
  - Complementarily leverages Radar Doppler velocity
```

### Uncertainty Head

```text
bev_uncertainty: [B, H, W]
  - Model prediction uncertainty
  - Designed to be high during sensor dropout, long range, and adverse weather
  - Estimated by Evidential Deep Learning or MC Dropout
  - An important input to the external evaluator
```

### Stopline and Crosswalk Heads

```text
bev_stopline: [B, H, W]
  - Probability of stop line presence
  - Important input for intersection control

bev_crosswalk: [B, H, W]
  - Probability of crosswalk presence
  - Important input for pedestrian handling
```

### Agent Detection Head (3D Object Detection)

The Agent Detection in this design has a **two-level classification head**.

- **Main class** (5 classes): used for behavior planning and collision avoidance
- **Vehicle subtype** (VEHICLE only): used for display and UI purposes

From an annotation cost perspective, the main classes are intentionally kept to a minimum. There is no need to distinguish a dog from a cat; it is sufficient to be able to properly avoid them as "dynamic obstacles." Vehicle subtypes, however, are useful for display to the driver and occupants (UI panels, visualization tools, etc.), so they are estimated separately for VEHICLE only.

```text
Main classes (5 classes) <- used for behavior planning:
  0: VEHICLE        = All vehicles (including passenger cars, trucks, buses, motorcycles)
  1: PEDESTRIAN     = Pedestrians (including walking, standing, crouching)
  2: CYCLIST        = Bicycles and electric scooters (differ from VEHICLE in speed and maneuverability)
  3: ANIMAL         = Common animals (dogs, deer, etc. Assumed to have similar speed/size to bicycles)
  4: UNKNOWN        = Unclassifiable obstacles (construction materials, fallen objects, special objects, etc.)

Vehicle subtypes (5 classes) <- for display only, when VEHICLE is detected:
  0: CAR            = Passenger cars, kei cars, minivans, SUVs
  1: TRUCK          = Trucks and vans (large and small)
  2: BUS            = Buses and minibuses
  3: MOTORCYCLE     = Motorcycles and mopeds
  4: SPECIAL        = Special vehicles such as construction machinery and agricultural equipment

Design intent:
  - Planner references main classes only (reducing annotation cost by limiting to 5 classes)
  - UI panels and log viewers reference vehicle_subtype to display vehicle-type icons, etc.
  - Misclassification of subtypes does not affect the Planner (display issue only)
  - Subtype is null for classes other than VEHICLE

Annotation policy:
  - Subtype may be omitted (treated as null) when main class alone is sufficient
  - Hard-to-classify objects should have main class set to UNKNOWN, and unknown_dynamic assigned if moving
```

#### Handling Unknown Dynamic Objects

Dynamic objects that do not fit into the above 5 classes (e.g., a trash can rolling on the road, or construction machinery that suddenly appears) are handled not by class classification but through **BEV temporal change (Temporal BEV)**.

```text
Unknown dynamic object detection logic:
  Step 1: Monitor time-series changes in bev_agent_occ
         Compute the difference in Occupancy between the current frame and the previous L frames

  Step 2: Dynamic object determination
         Occupancy in the same BEV cell moves over time -> treat as dynamic object
         Magnitude of velocity field bev_agent_vel exceeds threshold (e.g., 0.5 m/s)

  Step 3: Even if unclassified by Detection Head, estimate the following:
         - Position (cell in BEV)
         - Velocity vector (from bev_agent_vel)
         - Occupancy size (rough estimate from number of occupied BEV cells)

  Step 4: Pass to external evaluator with "UNKNOWN_DYNAMIC" flag
         - Planner treats as unclassified obstacle conservatively
         - Raise bev_uncertainty to encourage conservative evaluation

Design intent:
  Even if class classification is not accurate, "something is moving"
  can be detected from BEV temporal changes. That fact alone is sufficient for the Planner.
```

```text
Detection Head output:
  agent_states: List[AgentState]
    - bbox_bev:        (cx, cy, w, l, yaw)                           <- BEV coordinate system
    - velocity:        (vx, vy) m/s
    - heading:         float, rad
    - category:        "vehicle" / "pedestrian" / "cyclist" / "animal" / "unknown"
    - vehicle_subtype: "car" / "truck" / "bus" / "motorcycle" / "special" | null  <- VEHICLE only, for display
    - motion_state:    "parked" / "stationary" / "moving" / "accelerating" / "decelerating" / "turning"
    - existence_score: float [0, 1]
    - track_id:        str  <- inter-frame continuous identification ID
    - occlusion_level: "none" / "partial" / "full"
    - unknown_dynamic: bool  <- unknown dynamic object flag detected from BEV time series
  agent_tokens: (N_agent, C_agent)  <- embedding vectors for downstream Future Predictor

Loss:
  Main class:  CE loss (5 classes)
  Subtype:     CE loss (5 classes, only samples where category=="vehicle")
  Position regression: L1/Smooth L1
  Velocity regression: L1
Backbone: directly reuses BEV feature (no additional encoder)
Query count: maximum 200 agents (following nuScenes baseline)
```

---

## 3.9 Incorporating External Language Instructions

External language instructions (navigation commands, user commands) come from a channel independent of BEV.

```text
Input: "Please turn right at the next intersection"
    |
    v
Text Encoder (CLIP or T5 based)
    | T_inst: [B, N_tokens, D_lang]
    v
Input to CondFormer
```

### Types of Language Instructions

```text
Level 1: Identifier (lightest)
  - Numeric ID or one-hot
  - Examples: TURN_RIGHT, CHANGE_LANE_LEFT, KEEP_LANE

Level 2: Structured command (DSL)
  - Instruction format designed as a Domain-Specific Language
  - Example: {action: turn, direction: right, at: next_intersection}

Level 3: Lightweight natural text
  - Short text encoder (CLIP style)
  - Example: "Please turn right"

Recommended: Level 2 (DSL) as the baseline, Level 3 supplementing in read-only mode
Reason: Reduces ambiguity in natural language and limits impact on safety control
```

---

## 3.10 Internal Scene Tokenizer

Holds the scene context as a "linguistic compressed representation" for conditioning the Planner.

### Why Internal Language Tokens Are Necessary

```text
External language alone cannot capture:
  - Complexity of the current intersection
  - Intentions of nearby pedestrians and cyclists
  - Possibility of sudden braking by the vehicle ahead
  - Risks on the ego vehicle's route

Passing low-level BEV features directly:
  - Requires passing all BEV tokens to the Planner
  - High computation cost
  - Semantic structure is lost

Internal scene tokens:
  - Represent a scene summary in latent space
  - Can be passed to the Planner as a small number of tokens
  - Scene summaries can be distilled with a VLM teacher
```

### Scene Tokenizer Structure

```text
Input: BEV feature + BEV information heads outputs
    |
    v
Scene Query Tokens (fixed count: N_scene = 8 to 16)
    | Cross-Attention over BEV
    v
Scene Token Features [B, N_scene, D_scene]
    | (optional) Debug text decoder
    v
T_scene_text (string for debugging)
```

### Distillation by VLM Teacher

DriveLM or LLaVA is used to assist in training the scene tokens.

```text
Teacher: large VLM (DriveLM, LLaVA-v2)
  - Input: raw camera images
  - Output: scene description text + risk analysis

Student: Internal Scene Tokenizer
  - Target: produce similar "meaning" in latent space
  - Loss: feature distillation + optional text decode loss
```

---

## 3.11 CondFormer (Conditioning Transformer)

Integrates BEV information, external language instructions (T_inst), and internal scene tokens (T_scene) to build a conditioning vector for the Planner.

### CondFormer Structure

```text
Input:
  - BEV tokens (spatial)
  - T_inst (external language)
  - T_scene (internal scene)
  * Agent information is not used here; agent futures (agent_futures) are referenced directly by the Planner

CondFormer:
  Layer 1: Cross-attention over BEV
  Layer 2: Cross-attention over T_inst
  Layer 3: Cross-attention over T_scene
  Layer 4: Self-attention (integration of all conditions)
  Layer 5 (optional): Instruction priority gate
    -> Amplify the weight of T_inst when a strong external instruction is present
```

### Instruction Priority Gate

```text
To reliably reflect strong instructions such as a turn-right command,
add a gate that amplifies attention weights according to the strength of the external language instruction.

instruction_priority_gate = sigmoid(linear(T_inst_cls))
CondFormer_out = (1 + instruction_priority_gate) * language_attention_out
                + standard_attention_out
```

---

## 3.12 Connection to the VLA Planner

The CondFormer output is used to initialize the Q (query) of the VLA Planner, or as key/value input for cross-attention.

```text
Planner input:
  - Spatial BEV tokens (static + dynamic + lane summary)
  - Dynamic Agent future tokens
  - CondFormer output (language + scene conditions)
  - Ego context tokens (speed, yaw_rate, steer, etc.)

Planner output:
  - K trajectory candidates [B, K, T, D]
  - Confidence scores [B, K]
```

---

## 3.13 Lane Topology World Model (Details)

Lane topology (the network structure of lanes) is one of the most important pieces of static information that forms the basis of planning decisions.

### 3.13.1 Why Lane Topology Is Necessary

```text
BEV segmentation alone cannot determine:
  - Where this lane leads ahead
  - Whether it is a right-turn-only or straight-through lane
  - Which lane to enter at an intersection
  - Which lane is next on the route
  - Whether a U-turn is possible

Lane Topology supplements all of these.
```

### 3.13.2 Tesla-Style Learning for Lane Guidance

As a lesson from Tesla's public information, there is "attention-style" lane learning that does not rely on HD Maps.
Lane boundary existence probabilities and geometry (curves) are predicted directly from BEV features, and the connection graph ahead is also predicted.

```text
Lane geometry prediction:
  - Represent lane centerline with Bezier curves or polynomials
  - Output coordinates of each control point and existence probability
  - Ground truth: HD Map or lane boundaries estimated from LiDAR

Topology prediction:
  - Predict whether each lane node connects to the next node via classification
  - Adjacency matrix as binary prediction task
```

### 3.13.3 Lane Topology Branch Design

```text
Inputs:
  - Current BEV feature [B, C, H, W]
  - Temporal BEV memory [B, T_hist, C, H, W] (ego motion warped)
  - Map candidates [B, N_map, D_map] (when HD/SD Map is available)
  - Ego motion [B, T_hist, 3] (dx, dy, dyaw)
  - Ego state [speed, yaw_rate, accel, steer]
    |
    v
Map Candidate Encoder + Ego Motion Encoder
    |
    v
BEV-map cross-attention / temporal fusion
    |
    v
Lane Query Tokens (N_lane queries)
    | Deformable Cross-Attention over BEV
    v
Lane Node Features [B, N_lane, D_lane]
    | MLP heads
    v
Lane Geometry Output: [B, N_lane, N_ctrl_pts, 2]  (Bezier control points in BEV)
Lane Existence Score: [B, N_lane]
    | GNN (Graph Neural Network)
    v
Lane Topology Edges [B, N_lane, N_lane]  (adj matrix: successor/predecessor/adjacent)
Ego Pose in Lane Graph:
  - current_lane_candidates
  - lateral_offset
  - heading_error
  - lane_graph_confidence
```

This Branch uses the current BEV observation and temporal BEV memory as primary inputs, with map candidates as auxiliary information for cross-referencing, rather than directly applying the map as ground truth.
Ego motion is used not only to warp past BEV to current coordinates, but also to temporally maintain lane connections that were briefly missing from observations.

### 3.13.4 Lane Graph Representation

```text
LaneNode:
  lane_id: str
  center_line: List[(x, y)]        # BEV coordinates (ego coordinate system, unit m)
  curvature_profile: List[float]   # Curvature kappa [1/m] at each point on center_line
                                   # Same length. Positive = left curve, negative = right curve
  max_curvature: float             # Maximum value of |kappa| [1/m] (for speed limit check)
  advisory_speed_mps: float        # Curvature-based advisory speed sqrt(a_lat_max/|kappa|)
                                   # Refer to speed_limit when kappa ~= 0
  lane_type: str                   # "normal" / "turn" / "merge" / "split"
  direction: str                   # "forward" / "left_turn" / "right_turn" / "u_turn"
  speed_limit: float               # Regulatory speed m/s, -1 if unknown
  traffic_light_controlled: bool
  existence_score: float           # Model confidence

LaneEdge:
  src_id: str
  dst_id: str
  edge_type: str              # "successor" / "predecessor" / "left_adj" / "right_adj"
  weight: float               # Connection strength
```

#### Curvature Profile Calculation Method

`curvature_profile` is calculated at each point of `center_line` using the three-point arc method.

```python
import numpy as np

def compute_curvature_profile(center_line: list) -> list:
    """Compute curvature kappa [1/m] at each point of a polyline using the three-point arc method"""
    pts = np.array(center_line)  # (N, 2)
    kappas = [0.0]               # First point has no neighbors, so 0
    for i in range(1, len(pts) - 1):
        p0, p1, p2 = pts[i-1], pts[i], pts[i+1]
        # Curvature of the circle passing through 3 points
        d01 = np.linalg.norm(p1 - p0)
        d12 = np.linalg.norm(p2 - p1)
        d02 = np.linalg.norm(p2 - p0)
        area = abs(np.cross(p1 - p0, p2 - p0))  # Half of parallelogram area
        if d01 * d12 * d02 < 1e-6:
            kappas.append(0.0)
        else:
            R = (d01 * d12 * d02) / (2.0 * area + 1e-9)  # Circumscribed circle radius
            # Sign: left curve positive (determined by sign of cross product)
            sign = np.sign(np.cross(p1 - p0, p2 - p1))
            kappas.append(float(sign / R))
    kappas.append(0.0)           # Last point
    return kappas

A_LAT_MAX = 2.0  # m/s^2 (lateral acceleration comfort upper limit)

def advisory_speed(max_kappa: float, speed_limit: float) -> float:
    """Calculate advisory speed from curvature"""
    if abs(max_kappa) < 1e-4:  # Close to straight line
        return speed_limit
    v_adv = (A_LAT_MAX / abs(max_kappa)) ** 0.5
    return min(v_adv, speed_limit)
```

#### Uses of the Curvature Profile

```text
1. Speed constraint input to Planner:
   - Calculate advisory_speed_mps from max_curvature of look-ahead lanes in target_lane_sequence
   - Pass to Planner's speed profile generation
   - Encourages early deceleration before corners

2. Comfort check by External Evaluator:
   - Verify that local curvature of the trajectory does not significantly exceed the lane's curvature_profile
   - Soft penalty if |kappa_traj - kappa_lane| > 0.05 [1/m]

3. Sanity check by Converter:
   - Cross-check curvature computed by Converter from trajectory with LaneNode.curvature_profile
   - Large discrepancy is detected as a sign of sensor misperception or map inconsistency
```

### 3.13.5 Map-Assisted Lane Topology Estimation

When HD Map or SD Map is available, map information is incorporated as surrounding lane candidates and cross-referenced with online estimation.
However, in this design, the map is not fixed as a strong prior; only candidates consistent with the lane structure in the BEV observation space from sensors are adopted into the local lane topology.

```text
Map candidates:
  - Retrieve surrounding road segments from rough positioning
  - If HD Map: use lane geometry and connection candidates
  - If SD Map: use road centerlines, intersection structures, and regulatory information as candidates
  - May be inaccurate due to missed map updates, construction, or temporary restrictions

Observed BEV lane evidence:
  - Estimate lane boundaries, stop lines, and drivable areas from Camera/LiDAR/Radar integrated BEV
  - Warp Temporal BEV memory to current coordinates via ego motion and integrate
  - Supplement temporary occlusion or detection drops with temporal information

Estimation method:
  - Cross-reference Map candidates and BEV observations via cross-attention or matching cost
  - Consistent candidates are used for disambiguation of lane IDs, connections, and regulatory information
  - Inconsistent candidates are weakened or treated as map inconsistencies
  - Without map: estimate from BEV observation + Temporal memory + ego motion alone
  - Output is the local Lane Graph consistent with current observations, and ego pose on that Graph
```

### 3.13.6 Route-Aware Lane Selection

Cross-referencing the navigation route with the current lane topology to select which lane the ego vehicle should head toward.

```text
Route waypoints: (global lat/lon sequence)
    | Search nearby Map candidates using rough positioning
    v
Route Segment Candidates: (road_segment_id_1, road_segment_id_2, ...)
    | Cross-reference with local Lane Topology Graph
    v
Route Lane Sequence Candidates: (lane_id_1, lane_id_2, lane_id_3, ...)
    | Shortest path search on Lane Topology Graph
    v
target_lane_sequence: Input to Planner
```

### 3.13.7 Handling Left-Turn and Right-Turn Lanes

Lane selection at intersections requires special attention.

```text
Right turn:
  - Move to right-turn-only lane if available
  - Learn waiting position (where to wait for oncoming traffic) from training data
  - Timing for checking pedestrian crosswalk

Left turn (Japan / right-hand drive):
  - Wait for oncoming traffic
  - Entry position into intersection center
  - Checking crosswalk and signals
```

### 3.13.8 Handling Exits, Splits, and Merges

```text
Highway exit:
  - Reduce speed and move toward exit lane
  - Decide direction early before the split point

Split (Y-junction):
  - Determine which direction to go from the route
  - Recognize "split" node in lane topology

Merge:
  - Adjust acceleration to calculate merge timing
  - Check following vehicle speed via Dynamic Risk Map
```

### 3.13.9 Uncertainty Handling When Map Candidates and Observations Are Inconsistent

```text
When the gap between Map candidates and the local Lane Graph estimated from BEV observations is large:
  - Calculate map_mismatch_score
  - When threshold is exceeded: raise bev_uncertainty
  - External evaluator: select more conservative trajectory
  - ODD determination: raise map inconsistency flag
  - Lane Topology Branch: re-estimate local Lane Graph with observation priority
  - Log: record map inconsistency occurrence, add to map update or retraining queue
```

### 3.13.10 Output Interface

```json
{
  "lane_nodes": [
    {
      "lane_id": "lane_001",
      "center_line": [[0.0, 0.2], [5.0, 0.1], [10.0, 0.0]],
      "curvature_profile": [0.0, 0.012, 0.0],
      "max_curvature": 0.012,
      "advisory_speed_mps": 12.9,
      "lane_type": "normal",
      "direction": "forward",
      "speed_limit": 13.9,
      "traffic_light_controlled": false,
      "existence_score": 0.92
    }
  ],
  "lane_edges": [
    {
      "src_id": "lane_001",
      "dst_id": "lane_002",
      "edge_type": "successor",
      "weight": 0.98
    }
  ],
  "target_lane_sequence": ["lane_001", "lane_002", "lane_003"],
  "ego_pose_in_lane_graph": {
    "current_lane_candidates": ["lane_001"],
    "lateral_offset_m": 0.12,
    "heading_error_rad": -0.01,
    "confidence": 0.91
  },
  "map_mismatch_score": 0.05
}
```

### 3.13.11 Lightweight Update Strategy

Lane Topology does not need to be recomputed on every frame.

```text
Slow update (1-5 Hz):
  - Full re-estimation of Lane Topology
  - Consistency check between Map candidates and BEV observations
  - Ego pose estimation on local Lane Graph

Medium update (10 Hz):
  - Update of target_lane_sequence (on route changes)
  - Update of map_mismatch_score

Fast layer (20 Hz+):
  - Cache and use the last Lane Topology estimation result
  - Use warped by ego motion
  - Update lateral offset and yaw error relative to local Lane Graph
```

---

## 3.14 Limits and Notes on BEV Accuracy by Sensor

| Sensor | BEV Accuracy Limits | Notes |
|---|---|---|
| Camera | Degrades at long range, nighttime, and backlight | Evaluate performance variation by lighting conditions in advance |
| LiDAR | Point cloud decreases in rain and fog | Monitor point cloud density in adverse weather |
| Radar | Ghost targets, low resolution | Radar-only BEV has no shape information |

It is important to understand these limitations in advance and design the sensor quality gate and uncertainty estimation accordingly.

---

## 3.15 Important Implementation Notes (Unifying the BEV Coordinate System)

If the BEV coordinate system definition is not unified across all modules, fusion will not work correctly.

```text
Items that must share a common definition:
  - x: forward, y: left (or x: right, y: forward — unify to one convention)
  - BEV grid origin: ego center or rear axle
  - BEV grid resolution: meters/pixel (e.g., 0.5 m/pixel with 200x200 -> 100 m x 100 m)
  - Positive direction of yaw: counter-clockwise or clockwise

Common implementation failures:
  - Camera and LiDAR use separate coordinate systems, causing misalignment in BEV
  - Yaw sign is reversed, causing lane left-right flip
  - Modules with rear axle origin and ego center origin coexist
```

**This coordinate system definition must be documented at the start of the project and placed where all implementers can refer to it.** The cost of fixing it later is extremely high.

---

## 3.16 Chapter Summary

```text
Elements designed in this chapter:
  1. Sensor synchronization and calibration
  2. Camera BEV Encoder (view transformer)
  3. LiDAR BEV Encoder (PointPillars + sparse conv)
  4. Radar BEV Encoder (pillar approach + velocity)
  5. Modality-gated Tri-modal BEV Fusion
  6. Temporal BEV Memory (with ego motion warp)
  7. BEV token reduction for Planner
  8. BEV Information Heads (9 types, including Agent Detection Head)
  9. Incorporating External Language Instructions
 10. Internal Scene Tokenizer (VLM teacher)
 11. CondFormer (Conditioning Transformer)
 12. Lane Topology World Model (detailed design)
```

The next chapter details the Dynamic Object World Model and future risk estimation.
