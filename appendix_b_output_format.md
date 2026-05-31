# 付録B システム出力フォーマット仕様

---

## B.1 メイン出力 JSON スキーマ

1サイクルのシステム出力は以下のJSONフォーマットで定義する。

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

## B.2 フィールド定義

### candidates[].points のフィールド

| フィールド | 型 | 単位 | 説明 |
|---|---|---|---|
| t_s | float | 秒 | 現在時刻からの経過時間 |
| x_m | float | メートル | Ego前方方向の位置 |
| y_m | float | メートル | Ego左方向の位置 |
| v_mps | float | m/s | その時点での目標速度 |

### control フィールド

| フィールド | 型 | 単位 | 説明 |
|---|---|---|---|
| steer_rad | float | ラジアン | ステアリング角（+左, -右） |
| accel_mps2 | float | m/s² | 加速度（+加速, -減速） |
| valid | bool | - | 有効な指令かどうか |
| source | string | - | "planner", "fallback", "emergency_stop" |

### bev フィールド

| フィールド | 型 | 説明 |
|---|---|---|
| grid_size_m | float | BEVグリッドの1セルのサイズ [m] |
| grid_resolution | [int, int] | グリッドの解像度 [Bx, By] |
| center_offset | [float, float] | グリッド中心のEgo座標系オフセット [m] |
| drivable_area | base64 int8 | 走行可否クラスラベル。0=NOT_DRIVABLE（走行不可）/ 1=DRIVABLE（走行可）/ 2=MARGINAL（走行可だが走行すべきでない） |
| static_occupancy | base64 float32 | 静的障害物のlogit値 |
| lane_center | base64 float32 | 車線中心のlogit値 |
| stopline | base64 float32 | 停止線のlogit値 |
| dynamic_occ | base64 float32 | 動体占有のlogit値 |
| uncertainty | base64 float32 | BEV不確実性 ∈ [0,1] |
| occ_flow | base64 float32 | 占有フロー [dx, dy] m/s |

### agents[] フィールド

| フィールド | 型 | 説明 |
|---|---|---|
| id | string | エージェントID |
| class | string | メインクラス: vehicle / pedestrian / cyclist / animal / unknown |
| vehicle_subtype | string \| null | 車種サブタイプ（class=vehicleのみ）: car / truck / bus / motorcycle / special。表示用途。計画には使用しない |
| bbox_ego | object | Ego座標系でのバウンディングボックス |
| velocity_ego | object | Ego座標系での速度 (vx_mps, vy_mps) |
| stopped | bool | 停止中フラグ |
| parked_probability | float | 駐車確率 ∈ [0,1] |
| relevance_score | float | Plannerへの関連度スコア ∈ [0,1] |
| unknown_dynamic | bool | BEV時系列から検出した未知動体フラグ |
| futures | array | 将来軌跡候補（mode_id / probability / points） |

### lane_topology フィールド

| フィールド | 型 | 説明 |
|---|---|---|
| source | string | "bev_only", "bev_map_fusion" など、局所Lane Graphの生成方式 |
| map_source | string | "hd_map", "sd_map", "none" など、利用したMap候補の種類 |
| map_mismatch_score | float | Map候補とBEV観測の不整合スコア |
| ego_pose_in_lane_graph | object | 局所Lane Graph上の自車位置・姿勢 |
| target_lane_sequence | string[] | Plannerへ渡す局所レーン列 |
| lane_nodes | object[] | 現在観測に整合した局所レーンノード（curvature_profile / max_curvature / advisory_speed_mps を含む） |
| lane_edges | object[] | successor / adjacent などの接続関係 |

### ego_pose_in_lane_graph フィールド

| フィールド | 型 | 単位 | 説明 |
|---|---|---|---|
| current_lane_candidates | string[] | - | 自車が属する候補レーンID |
| lateral_offset_m | float | m | レーン中心線に対する横方向オフセット |
| heading_error_rad | float | rad | レーン接線方向に対するヨー角差 |
| confidence | float | - | 自己位置推定の信頼度 |

---

## B.3 バイナリフォーマット（高周波ロギング用）

JSONはデバッグ・可視化用。高周波（10Hz以上）のロギングにはバイナリフォーマットを使う。

```python
import struct

# ヘッダ（固定長）
HEADER_FORMAT = "QIff??ff"
# Q: timestamp_ns (8バイト)
# I: cycle_id (4バイト)
# f: selected_trajectory[0].x (4バイト)
# f: selected_trajectory[0].y (4バイト)
# ?: fallback_active (1バイト)
# ?: valid (1バイト)
# f: steer_rad (4バイト)
# f: accel_mps2 (4バイト)
# 合計: 26バイト

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

## B.4 Shadow Mode ログのフォーマット

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

## B.5 インタフェース互換性の維持

```text
バージョン管理方針:
  - JSONスキーマは semantic versioning (MAJOR.MINOR)
  - MINOR変更: フィールドの追加（後方互換）
  - MAJOR変更: フィールドの削除・型変更（互換性破壊）

互換性の確認:
  - 新しいフィールドを追加するだけなら MINOR バージョンアップ
  - 読み手側は知らないフィールドは無視する設計にする
  - 型変更・フィールド削除は MAJOR バージョンアップ

実装:
  def parse_output(data: dict) -> PlannerOutputs:
      version = data.get("version", "1.0")
      major = int(version.split(".")[0])
      if major != 1:
          raise ValueError(f"Unsupported schema version: {version}")
      # パース処理
      ...
```
