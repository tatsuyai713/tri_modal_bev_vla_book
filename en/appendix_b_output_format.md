# Appendix B System Output Format Specification

---

## B.1 Main Output JSON Schema

The system output for one cycle is defined in the following JSON format.

```json
{
  "version": "1.0",
  "timestamp_ns": 1700000000000000000,
  "cycle_id": 12345,
  "fallback_active": false,

  "candidates": [
    {
      "id": 0,
      "safety_score": 0.92,
      "selected": true,
      "points": [
        {"t_s": 0.5, "x_m": 3.5, "y_m": 0.02, "v_mps": 10.2},
        {"t_s": 1.0, "x_m": 8.8, "y_m": 0.05, "v_mps": 11.0},
        {"t_s": 1.5, "x_m": 14.5, "y_m": 0.08, "v_mps": 11.5},
        {"t_s": 2.0, "x_m": 20.5, "y_m": 0.10, "v_mps": 11.8},
        {"t_s": 2.5, "x_m": 26.8, "y_m": 0.12, "v_mps": 12.0},
        {"t_s": 3.0, "x_m": 33.2, "y_m": 0.15, "v_mps": 12.0}
      ]
    },
    {
      "id": 1,
      "safety_score": 0.85,
      "selected": false,
      "points": [...]
    }
  ],

  "control": {
    "steer_rad": -0.023,
    "accel_mps2": 0.5,
    "valid": true,
    "source": "planner"
  },

  "bev": {
    "grid_size_m": 0.2,
    "grid_resolution": [512, 512],
    "center_offset": [0, 0],
    "drivable_area": "<base64 encoded int8 tensor, 512x512>",
    "drivable_area_note": "0=NOT_DRIVABLE, 1=DRIVABLE, 2=MARGINAL",
    "static_occupancy": "<base64 encoded float32 tensor, 512x512>",
    "lane_center": "<base64 encoded float32 tensor, 512x512>",
    "stopline": "<base64 encoded float32 tensor, 512x512>",
    "dynamic_occ": "<base64 encoded float32 tensor, 512x512>",
    "uncertainty": "<base64 encoded float32 tensor, 512x512>",
    "occ_flow": "<base64 encoded float32 tensor, 512x512x2>"
  },

  "lane_topology": {
    "source": "bev_map_fusion",
    "map_source": "hd_map",
    "map_mismatch_score": 0.05,
    "ego_pose_in_lane_graph": {
      "current_lane_candidates": ["lane_001"],
      "lateral_offset_m": 0.12,
      "heading_error_rad": -0.01,
      "confidence": 0.91
    },
    "target_lane_sequence": ["lane_001", "lane_002", "lane_003"],
    "lane_nodes": [
      {
        "lane_id": "lane_001",
        "center_line_ego": [[0.0, 0.2], [10.0, 0.1], [30.0, 0.0]],
        "curvature_profile": [0.0, 0.008, 0.0],
        "max_curvature": 0.008,
        "advisory_speed_mps": 15.8,
        "lane_type": "normal",
        "direction": "forward",
        "speed_limit": 16.7,
        "existence_score": 0.94
      }
    ],
    "lane_edges": [
      {
        "src_id": "lane_001",
        "dst_id": "lane_002",
        "edge_type": "successor",
        "weight": 0.97
      }
    ]
  },

  "agents": [
    {
      "id": "agent_001",
      "class": "vehicle",
      "vehicle_subtype": "car",
      "bbox_ego": {
        "cx_m": 15.2, "cy_m": 0.0,
        "length_m": 4.8, "width_m": 1.9,
        "yaw_rad": 0.01
      },
      "velocity_ego": {"vx_mps": 11.5, "vy_mps": 0.1},
      "stopped": false,
      "parked_probability": 0.02,
      "relevance_score": 0.85,
      "futures": [
        {
          "mode_id": 0,
          "probability": 0.72,
          "points": [
            {"t_s": 0.5, "x_m": 21.0, "y_m": 0.0},
            {"t_s": 1.0, "x_m": 26.8, "y_m": 0.0}
          ]
        },
        {
          "mode_id": 1,
          "probability": 0.28,
          "points": [
            {"t_s": 0.5, "x_m": 20.5, "y_m": -3.5},
            {"t_s": 1.0, "x_m": 26.0, "y_m": -3.8}
          ]
        }
      ]
    }
  ],

  "debug": {
    "sensor_health": {
      "camera_min": 0.98,
      "lidar": 1.0,
      "radar": 0.95
    },
    "modality_gate_weights": {
      "camera": 0.50,
      "lidar": 0.38,
      "radar": 0.12
    },
    "bev_uncertainty_mean": 0.05,
    "bev_uncertainty_p95": 0.15,
    "planner_latency_ms": 42.3,
    "total_latency_ms": 68.5,
    "instruction": {
      "external_text": "turn right at next intersection",
      "external_dsl": "TURN_RIGHT",
      "scene_description": "Clear highway, one vehicle ahead"
    },
    "evaluator": {
      "candidate_results": [
        {"id": 0, "passed": true, "score": 0.92},
        {"id": 1, "passed": true, "score": 0.85},
        {"id": 2, "passed": false, "reason": "static_collision"},
        {"id": 3, "passed": false, "reason": "off_drivable"}
      ],
      "all_failed": false
    },
    "odd": {
      "within_odd": true,
      "odd_id": "highway_daytime_v1",
      "confidence": 0.97,
      "reasons": []
    }
  }
}
```

---

## B.2 Field Definitions

### candidates[].points Fields

| Field | Type | Unit | Description |
|---|---|---|---|
| t_s | float | seconds | Elapsed time from the current moment |
| x_m | float | meters | Position in ego forward direction |
| y_m | float | meters | Position in ego left direction |
| v_mps | float | m/s | Target speed at that point in time |

### control Fields

| Field | Type | Unit | Description |
|---|---|---|---|
| steer_rad | float | radians | Steering angle (+left, -right) |
| accel_mps2 | float | m/s² | Acceleration (+accelerate, -decelerate) |
| valid | bool | - | Whether the command is valid |
| source | string | - | "planner", "fallback", "emergency_stop" |

### bev Fields

| Field | Type | Description |
|---|---|---|
| grid_size_m | float | Size of one BEV grid cell [m] |
| grid_resolution | [int, int] | Grid resolution [Bx, By] |
| center_offset | [float, float] | Ego-frame offset of the grid center [m] |
| drivable_area | base64 int8 | Drivability class label. 0=NOT_DRIVABLE / 1=DRIVABLE / 2=MARGINAL (physically passable but should not normally be driven on) |
| static_occupancy | base64 float32 | Static obstacle logit values |
| lane_center | base64 float32 | Lane center logit values |
| stopline | base64 float32 | Stop line logit values |
| dynamic_occ | base64 float32 | Dynamic occupancy logit values |
| uncertainty | base64 float32 | BEV uncertainty in [0,1] |
| occ_flow | base64 float32 | Occupancy flow [dx, dy] m/s |

### agents[] Fields

| Field | Type | Description |
|---|---|---|
| id | string | Agent ID |
| class | string | Main class: vehicle / pedestrian / cyclist / animal / unknown |
| vehicle_subtype | string \| null | Vehicle subtype (only when class=vehicle): car / truck / bus / motorcycle / special. For display purposes only. Not used for planning. |
| bbox_ego | object | Bounding box in ego coordinate frame |
| velocity_ego | object | Velocity in ego coordinate frame (vx_mps, vy_mps) |
| stopped | bool | Stopped flag |
| parked_probability | float | Parking probability in [0,1] |
| relevance_score | float | Relevance score for the Planner in [0,1] |
| unknown_dynamic | bool | Unknown dynamic object flag detected from BEV time series |
| futures | array | Future trajectory candidates (mode_id / probability / points) |

### lane_topology Fields

| Field | Type | Description |
|---|---|---|
| source | string | Method used to generate the local lane graph: "bev_only", "bev_map_fusion", etc. |
| map_source | string | Type of map candidates used: "hd_map", "sd_map", "none", etc. |
| map_mismatch_score | float | Mismatch score between map candidates and BEV observations |
| ego_pose_in_lane_graph | object | Ego vehicle position and pose in the local lane graph |
| target_lane_sequence | string[] | Local lane sequence passed to the Planner |
| lane_nodes | object[] | Local lane nodes aligned with the current observation (includes curvature_profile / max_curvature / advisory_speed_mps) |
| lane_edges | object[] | Connectivity relationships: successor / adjacent, etc. |

### ego_pose_in_lane_graph Fields

| Field | Type | Unit | Description |
|---|---|---|---|
| current_lane_candidates | string[] | - | Candidate lane IDs the ego vehicle belongs to |
| lateral_offset_m | float | m | Lateral offset from the lane centerline |
| heading_error_rad | float | rad | Yaw angle difference from the lane tangent direction |
| confidence | float | - | Confidence of self-localization |

---

## B.3 Binary Format (for High-Frequency Logging)

JSON is for debugging and visualization. Use the binary format for high-frequency logging (10 Hz or above).

```python
import struct

# Header (fixed length)
HEADER_FORMAT = "QIff??ff"
# Q: timestamp_ns (8 bytes)
# I: cycle_id (4 bytes)
# f: selected_trajectory[0].x (4 bytes)
# f: selected_trajectory[0].y (4 bytes)
# ?: fallback_active (1 byte)
# ?: valid (1 byte)
# f: steer_rad (4 bytes)
# f: accel_mps2 (4 bytes)
# Total: 26 bytes

HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

def pack_output(output: "PlannerOutputs") -> bytes:
    traj = output.selected_trajectory
    header = struct.pack(
        HEADER_FORMAT,
        output.timestamp_ns,
        output.cycle_id,
        float(traj[0, 0]),  # x
        float(traj[0, 1]),  # y
        output.fallback_active,
        output.control.valid,
        output.control.steer_rad,
        output.control.accel_mps2,
    )
    return header

def unpack_output(data: bytes) -> dict:
    ts, cid, x, y, fb, valid, steer, accel = struct.unpack(
        HEADER_FORMAT, data[:HEADER_SIZE]
    )
    return {
        "timestamp_ns": ts,
        "cycle_id": cid,
        "first_x": x,
        "first_y": y,
        "fallback_active": fb,
        "valid": valid,
        "steer_rad": steer,
        "accel_mps2": accel,
    }
```

---

## B.4 Shadow Mode Log Format

```json
{
  "shadow_log_version": "1.0",
  "session_id": "sess_2024_0101_abc123",
  "vehicle_id": "VH_001",

  "cycles": [
    {
      "cycle_id": 12345,
      "timestamp_ns": 1700000000000000000,

      "system": {
        "selected_traj_first_x": 3.5,
        "selected_traj_first_y": 0.02,
        "steer_rad": -0.023,
        "accel_mps2": 0.5,
        "fallback_active": false,
        "bev_uncertainty_mean": 0.05
      },

      "human": {
        "actual_steer_rad": -0.019,
        "actual_accel_mps2": 0.6,
        "velocity_mps": 11.2
      },

      "diff": {
        "steer_diff_rad": 0.004,
        "accel_diff_mps2": 0.1,
        "traj_ade_m": 0.15,
        "severity": "benign"
      },

      "tags": ["highway", "daylight", "clear_weather"]
    }
  ]
}
```

---

## B.5 Maintaining Interface Compatibility

```text
Versioning policy:
  - JSON schema uses semantic versioning (MAJOR.MINOR)
  - MINOR change: adding fields (backward compatible)
  - MAJOR change: deleting or changing types of fields (breaking change)

Compatibility checking:
  - Simply adding new fields -> MINOR version bump
  - Consumers are designed to ignore unknown fields
  - Type changes and field deletions require a MAJOR version bump

Implementation:
  def parse_output(data: dict) -> PlannerOutputs:
      version = data.get("version", "1.0")
      major = int(version.split(".")[0])
      if major != 1:
          raise ValueError(f"Unsupported schema version: {version}")
      # parse logic
      ...
```
