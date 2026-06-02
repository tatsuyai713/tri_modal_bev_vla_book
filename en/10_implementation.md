# Chapter 10 Implementation Modules and Interface Design

---

## 10.1 Why Interface Definition Is the Most Critical Step

When multiple people or teams implement modules, unclear interface definitions lead to problems such as:

```text
Coordinate system mismatch:
  - Camera developer codes x=right
  - LiDAR developer codes x=forward
  - When fused on BEV, left and right are reversed

Timestamp mismatch:
  - Camera uses capture time at 30Hz
  - LiDAR uses scan start time
  - Fused with over 100ms offset

BEV origin mismatch:
  - Rear axle reference and ego center reference mixed together
  - Lane detections drawn at wrong positions

Resolution confusion:
  - BEV 0.4m/pixel and 0.5m/pixel mixed
  - Planning distances off by 20%
```

**Document the interface definitions at the start, and have all implementers reference them. The cost of fixing this later is extremely high.**

---

## 10.2 Critical Definitions That Must Be Established First

### Ego Coordinate System

```text
Definition (unified across the entire project):
  - x: forward
  - y: left
  - z: up
  - yaw: counter-clockwise positive
  - units: meter, radian
  - Note: aligning with NuScenes format is convenient
```

### BEV Grid

```text
BEV Grid:
  x_min: -20.0m
  x_max:  80.0m  (80m forward, 20m backward)
  y_min: -40.0m
  y_max:  40.0m  (40m each side)
  resolution: 0.4m/pixel
  H: (x_max - x_min) / resolution = 250
  W: (y_max - y_min) / resolution = 200

  * Many actual models use H=W=200
  -> In this book, unless otherwise noted, examples use 200x200, 0.5m/pixel
  -> range: ±50m x ±50m

origin:
  - (0, 0): ego center (current frame)
  - forward is +x, left is +y
```

### Timestamp

```text
Unit: nanoseconds (int64)
Reference: GPS time (or PTP-synchronized time)
All sensors and all frames are tagged with the same reference timestamp
Processing delay: each module records its processing completion time
```

---

## 10.3 Repository Structure

```text
tri_modal_bev_vla/
├── configs/
│   ├── bev_grid.yaml          # BEV grid definition (shared across all modules)
│   ├── sensor_layout.yaml     # Sensor layout and calibration
│   ├── planner.yaml           # Planner hyperparameters
│   └── vehicle_params.yaml    # Vehicle parameters (wheelbase, etc.)
│
├── data/
│   ├── datasets/
│   │   ├── nuscenes_loader.py
│   │   ├── waymo_loader.py
│   │   └── custom_loader.py
│   ├── preprocessing/
│   │   ├── sync_calibrate.py
│   │   └── human_traj_builder.py
│   └── augmentation/
│       └── sensor_dropout.py
│
├── models/
│   ├── camera_bev/
│   │   ├── backbone.py
│   │   ├── neck.py
│   │   └── view_transformer.py
│   ├── lidar_bev/
│   │   ├── voxelizer.py
│   │   └── lidar_encoder.py
│   ├── radar_bev/
│   │   └── radar_encoder.py
│   ├── fusion/
│   │   ├── modality_gate.py
│   │   └── temporal_memory.py
│   ├── heads/
│   │   ├── static_world_head.py
│   │   ├── lane_topology_head.py
│   │   ├── dynamic_object_head.py
│   │   ├── agent_future_predictor.py
│   │   └── dynamic_risk_map.py
│   ├── language/
│   │   ├── text_encoder.py
│   │   ├── scene_tokenizer.py
│   │   └── cond_former.py
│   └── planner/
│       ├── vla_planner.py
│       └── confidence_head.py
│
├── evaluation/
│   ├── external_evaluator.py
│   ├── ood_monitor.py
│   └── metrics.py
│
├── control/
│   └── traj_to_steer.py
│
├── training/
│   ├── losses.py
│   ├── train.py
│   └── staged_training.py
│
├── tests/
│   ├── test_bev_coord.py     # Coordinate system consistency tests
│   ├── test_sensor_sync.py
│   ├── test_evaluator.py
│   └── test_converter.py
│
└── tools/
    ├── visualize_bev.py
    ├── visualize_traj.py
    └── log_analyzer.py
```

---

## 10.4 SensorPacket Interface

```python
@dataclass
class SensorPacket:
    images: torch.Tensor          # [B, N_cam, C, H, W]
    lidar_points: torch.Tensor    # [B, N_sweeps, N_pts, D]
    radar_points: torch.Tensor    # [B, N_sweeps, N_rpts, D_r]
    ego_state: EgoState
    calibration: CalibrationData
    speed_constraints: "SpeedConstraintInputs"
    timestamp_ns: int

@dataclass
class EgoState:
    speed: float                  # m/s
    yaw_rate: float               # rad/s
    accel_x: float                # m/s^2
    accel_y: float                # m/s^2
    steer_angle: float            # rad
    timestamp_ns: int

@dataclass
class CalibrationData:
    camera_intrinsics: torch.Tensor   # [N_cam, 3, 3]
    camera_extrinsics: torch.Tensor   # [N_cam, 4, 4]  ego <- cam
    lidar_to_ego: torch.Tensor        # [4, 4]
    radar_to_ego: List[torch.Tensor]  # [N_radar, [4,4]]
```

---

## 10.5 BEV Feature Interface

```python
@dataclass
class BEVFeature:
    features: torch.Tensor       # [B, C, H, W]
    valid_mask: torch.Tensor     # [B, H, W] bool
    timestamp_ns: int
    
    # BEV grid parameters (must match configs/bev_grid.yaml)
    x_min: float = -50.0        # m
    x_max: float =  50.0        # m
    y_min: float = -50.0        # m
    y_max: float =  50.0        # m
    resolution: float = 0.5     # m/pixel
```

**Important:** Always load BEV grid definitions from `configs/bev_grid.yaml`. Do not hardcode them.

---

## 10.6 Temporal BEV Memory Interface

```python
@dataclass
class TemporalBEVMemory:
    frames: List[BEVFeature]     # time-series BEV frames [L]
    ego_motion_log: List[EgoMotion]  # warp transforms to each frame
    
    def warp_to_current(self, frame_idx: int) -> BEVFeature:
        """Warp the BEV of the specified frame to the current ego frame and return it"""
        ...

@dataclass  
class EgoMotion:
    dx: float    # longitudinal displacement m
    dy: float    # lateral displacement m
    dyaw: float  # heading change rad
    dt: float    # time delta s
    timestamp_ns: int
```

---

## 10.7 World Outputs Interface

```python
@dataclass
class WorldOutputs:
    static_world: StaticWorldOutputs
    lane_topology: LaneTopologyOutputs
    dynamic_world: DynamicWorldOutputs
    agent_world: AgentWorldOutputs
    risk_outputs: RiskOutputs
    uncertainty: torch.Tensor    # [B, H, W]

@dataclass
class StaticWorldOutputs:
    bev_drivable: torch.Tensor   # [B, H, W]  class label: 0=NOT_DRIVABLE, 1=DRIVABLE, 2=MARGINAL
    bev_drivable_logits: torch.Tensor  # [B, 3, H, W]  raw logits (for loss computation)
    bev_occupancy: torch.Tensor  # [B, H, W]
    bev_lane: torch.Tensor       # [B, H, W, C]
    bev_stopline: torch.Tensor   # [B, H, W]
    bev_crosswalk: torch.Tensor  # [B, H, W]
    traffic_lights: List["TrafficLightState"]

@dataclass
class TrafficLightState:
    bbox: tuple                     # 2D/3D candidate; concrete type fixed in implementation
    state: str                      # "RED" / "YELLOW" / "GREEN" / "ARROW" / "UNKNOWN"
    confidence: float
    controlled_lane_ids: List[str]
    controlled_stopline_id: str
    arrow_direction: str            # "left" / "right" / "straight" / "none"
    time_since_seen_ms: float

@dataclass
class DynamicWorldOutputs:
    bev_agent_occ: torch.Tensor           # [B, H, W]
    bev_agent_vel: torch.Tensor           # [B, H, W, 2]
    bev_occupancy_flow: torch.Tensor      # [B, H, W, 2]
    bev_motion_prob: torch.Tensor         # [B, H, W]
    bev_future_agent_occ: torch.Tensor    # [B, T_future, H, W]
```

```python
@dataclass
class MapCandidateInputs:
    """Nearby Map candidates obtained from coarse localization. None or empty if no HD/SD Map available."""
    road_segments: List[str]
    lane_candidate_tokens: torch.Tensor    # [B, N_map, D_map]
    route_segment_candidates: List[str]
    source: str                            # "hd_map" / "sd_map" / "none"
    confidence: float

@dataclass
class EgoPoseInLaneGraph:
    current_lane_candidates: List[str]
    lateral_offset_m: float
    heading_error_rad: float
    confidence: float

@dataclass
class LaneNode:
    lane_id: str
    center_line: List[tuple]          # [(x, y), ...] in ego frame [m]
    curvature_profile: List[float]    # kappa [1/m] per point, same length as center_line
    max_curvature: float              # max |kappa| [1/m] in this node
    advisory_speed_mps: float         # sqrt(A_LAT_MAX / max_curvature), capped by speed_limit
    lane_type: str                    # "normal" / "turn" / "merge" / "split"
    direction: str                    # "forward" / "left_turn" / "right_turn" / "u_turn"
    speed_limit: float                # [m/s], -1 if unknown
    traffic_light_controlled: bool
    existence_score: float

@dataclass
class LaneEdge:
    src_id: str
    dst_id: str
    edge_type: str   # "successor" / "predecessor" / "left_adj" / "right_adj"
    weight: float

@dataclass
class LaneTopologyOutputs:
    lane_nodes: List[LaneNode]
    lane_edges: List[LaneEdge]
    target_lane_sequence: List[str]
    ego_pose_in_lane_graph: EgoPoseInLaneGraph
    map_mismatch_score: float

@dataclass
class SpeedConstraintInputs:
    map_speed_limit_mps: float
    sign_speed_limit_mps: float
    road_mark_speed_limit_mps: float
    source_confidence: Dict[str, float]  # {"map":..., "sign":..., "road_mark":...}
    user_overspeed_tolerance_mps: float
    policy_margin_max_mps: float
```

---

## 10.8 Planner Outputs Interface

```python
@dataclass
class PlannerOutputs:
    trajectories: torch.Tensor   # [B, K, T, D]
    confidence: torch.Tensor     # [B, K]  (not equal to safety probability)
    
    # D: (x, y, yaw, v) — ego frame
    # K: number of candidates
    # T: number of timesteps

@dataclass
class SelectedTrajectory:
    trajectory: torch.Tensor     # [T, D]
    candidate_id: int
    selection_reason: str
    all_passed: List[bool]       # [K] — evaluator results per candidate
    timestamp_ns: int
```

---

## 10.9 Control Command Interface

```python
@dataclass
class ControlCommand:
    target_curvature: float      # 1/m
    target_speed: float          # m/s
    target_accel: float          # m/s^2
    speed_phase: str             # "ACCEL" / "CRUISE" / "DECEL"
    target_steer_angle: float    # rad (optional, vehicle-specific)
    
    limit_steer: bool = False    # whether steering limit was activated
    limit_accel: bool = False    # whether acceleration limit was activated
    feasibility_ok: bool = True  # trajectory feasibility
    fallback_active: bool = False
    fallback_level: int = 0
    
    timestamp_ns: int = 0
```

---

## 10.10 Key Tensor Shape Reference

| Tensor | Example Shape | Description |
|---|---|---|
| images | [1, 6, 3, 900, 1600] | 6 cameras |
| lidar_points | [1, 3, 50000, 5] | 3 sweeps, 50K points, (x,y,z,i,t) |
| radar_points | [1, 2, 500, 6] | 2 sweeps, 500 points, (x,y,vx,vy,rcs,t) |
| cam_bev_feat | [1, 256, 200, 200] | Camera BEV |
| lid_bev_feat | [1, 256, 200, 200] | LiDAR BEV |
| rad_bev_feat | [1, 64, 200, 200] | Radar BEV |
| fused_bev | [1, 256, 200, 200] | Fused BEV |
| bev_tokens | [1, 40000, 256] | Flat BEV tokens |
| sparse_bev | [1, 3000, 256] | Sparse BEV tokens |
| T_inst | [1, 20, 256] | Language tokens |
| T_scene | [1, 8, 256] | Scene tokens |
| trajectories | [1, 16, 10, 4] | K=16, T=10, D=4 |
| confidence | [1, 16] | Per-candidate |
| agent_futures | [1, 32, 6, 12, 5] | 32 agents, 6 modes |
| dynamic_risk | [1, 8, 200, 200] | T_future=8 |

---

## 10.11 Common Implementation Failures

```text
Failure 1: Inconsistent coordinate systems
  - Camera and LiDAR have reversed x/y directions
  - BEV fusion produces left-right mirroring
  -> Fix: explicitly define x=forward, y=left in bev_grid.yaml and reference it in all implementations

Failure 2: BEV origin confusion
  - Rear axle reference and ego center reference mixed together
  - Object positions are offset
  -> Fix: unify origin to ego center (front axle center, ground level)

Failure 3: Yaw sign error
  - Defined as counter-clockwise positive but implemented as clockwise
  - Lanes drawn in mirrored positions
  -> Fix: document yaw convention and verify with unit tests

Failure 4: Using confidence as a safety probability
  - Misinterpreting confidence = 0.9 as "90% probability of being safe"
  - Selecting the high-confidence candidate without the External Evaluator
  -> Fix: always pass through the External Evaluator before selection

Failure 5: Forgetting to warp the Temporal BEV
  - Using the previous frame's BEV without warping by ego motion
  - Time-series BEV becomes "blurred"
  -> Fix: always use TemporalBEVMemory.warp_to_current()

Failure 6: Not checking feasibility_ok
  - Planner keeps selecting the same candidate even when the Converter returns False
  -> Fix: consider feasibility in the External Evaluator's next selection

Failure 7: BEV resolution mismatch
  - Model trained at 0.4m/pixel run at inference with 0.5m/pixel
  - Spatial scale changes and all metrics degrade
  -> Fix: load resolution from configs, do not hardcode

Failure 8: Forgetting Radar ego velocity correction
  - Radar vr not corrected to world frame
  - Dynamic object speed offset by ego vehicle speed
  -> Fix: perform ego velocity subtraction before RadarBEVEncoder input
```

---

## 10.12 Required Unit Tests and Visualization Checks

```text
Coordinate system tests:
  - Project LiDAR point cloud onto BEV and verify roads appear in correct orientation
  - Verify objects ahead appear in the x > 0 region

Temporal BEV tests:
  - After straight driving 10m, warp the previous BEV and compare with current BEV
  - Verify static object positions do not shift

Planner output tests:
  - Draw candidate trajectories on BEV and verify they stay within drivable area
  - Verify candidates are physically continuous (check speed and acceleration)

External Evaluator tests:
  - Input known unsafe trajectories and confirm all candidates Fail
  - Input known safe trajectories and confirm they Pass

Converter tests:
  - Straight trajectory -> verify steer_angle ≈ 0
  - Constant-curvature trajectory -> verify conversion to correct steer_angle
```

---

## 10.13 MVP Implementation Order

```text
Step 1: Document the BEV grid definition (most critical)
  -> Create configs/bev_grid.yaml for everyone to reference

Step 2: Finalize the SensorPacket data format
  -> Write type definitions first

Step 3: Implement CameraBEVEncoder and verify with visualization
  -> Overlay with LiDAR and verify (does the road align?)

Step 4: Implement LiDARBEVEncoder and verify as standalone

Step 5: Implement RadarBEVEncoder (starting from a simple version)

Step 6: Implement ModalityGateFuser and verify 3-modal fusion

Step 7: Implement TemporalBEVMemory and run warp tests

Step 8: BEV Information Heads (start with Static World)

Step 9: Lane Topology Branch
  -> Input: Map candidates + ego motion + Temporal BEV
  -> Output: local Lane Graph consistent with current observations, and ego pose

Step 10: Dynamic World Branch + Agent Future

Step 11: VLA Planner (start with K=4)

Step 12: External Evaluator (start with static collision check)

Step 13: Trajectory Converter

Step 14: Integration test -> move to Shadow Mode data collection
```

---

## 10.14 Chapter Summary

```text
Elements designed in this chapter:
  1. Importance of interface definitions (coordinate systems, timestamps, resolution)
  2. Four definitions that must be established first
  3. Repository structure
  4. Data class definitions for each module
  5. Key tensor shape reference
  6. Common implementation failures (8 types)
  7. Required unit tests and visualization checks
  8. MVP implementation order (14 steps)
```

The next chapter details the implementation roadmap and Shadow Mode verification.
