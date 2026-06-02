# 第10章 実装モジュールとインターフェース設計

---

## 10.1 なぜインターフェース定義が最重要か

モジュールを複数人・複数チームで実装する際、インターフェースの定義が不明確だと次のような問題が起きる。

```text
座標系のズレ:
  - Camera担当はx=rightでコーディング
  - LiDAR担当はx=forwardでコーディング
  - BEV上で融合すると左右が逆に重なる

タイムスタンプのズレ:
  - Cameraは30Hzで取得時刻ベース
  - LiDARはスキャン開始時刻ベース
  - 100ms以上ズレた状態で融合される

BEV原点の不一致:
  - rear axle基準と ego center基準が混在
  - 車線検出がずれた位置に描画される

解像度の取り違え:
  - BEV 0.4m/pixel と 0.5m/pixel の混在
  - 計画距離が20%ずれる
```

**インターフェース定義を最初に文書化し、全実装者が参照する。後から直すコストは非常に高い。**

---

## 10.2 修正必須の重要定義

### Ego座標系

```text
定義（プロジェクト全体で統一）:
  - x: 前方 (forward)
  - y: 左方向 (left)
  - z: 上方向 (up)
  - yaw: counter-clockwise positive
  - 単位: meter, radian
  - 参考: NuScenes形式と一致させると便利
```

### BEV Grid

```text
BEV Grid:
  x_min: -20.0m
  x_max:  80.0m  (前方80m、後方20m)
  y_min: -40.0m
  y_max:  40.0m  (左右各40m)
  resolution: 0.4m/pixel
  H: (x_max - x_min) / resolution = 250
  W: (y_max - y_min) / resolution = 200
  
  ※ 実際のモデルは H=W=200 の場合も多い
  → 本書では特記ない限り 200x200, 0.5m/pixel を例として使う
  → range: ±50m x ±50m

origin:
  - (0, 0): ego 中心（current frame）
  - 前方が+x, 左が+y
```

### タイムスタンプ

```text
単位: nanoseconds (int64)
基準: GPS時刻（または PTP同期時刻）
全センサ・全フレームに同じ基準のタイムスタンプを付与
処理遅れ: 各モジュールが処理完了時刻を記録
```

---

## 10.3 リポジトリ構成

```text
tri_modal_bev_vla/
├── configs/
│   ├── bev_grid.yaml          # BEV grid定義（全モジュール共通）
│   ├── sensor_layout.yaml     # センサ配置・キャリブレーション
│   ├── planner.yaml           # Plannerハイパーパラメータ
│   └── vehicle_params.yaml    # 車両パラメータ（wheelbase等）
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
│   ├── test_bev_coord.py     # 座標系の一貫性テスト
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

## 10.4 SensorPacket インターフェース

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

## 10.5 BEV Feature インターフェース

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

**注意:** BEV grid の定義は必ず `configs/bev_grid.yaml` から読み込む。ハードコードしない。

---

## 10.6 Temporal BEV Memory インターフェース

```python
@dataclass
class TemporalBEVMemory:
    frames: List[BEVFeature]     # 時系列BEVフレーム [L]
    ego_motion_log: List[EgoMotion]  # 各フレームへのwarp変換
    
    def warp_to_current(self, frame_idx: int) -> BEVFeature:
        """指定フレームのBEVを現在egoフレームにwarpして返す"""
        ...

@dataclass  
class EgoMotion:
    dx: float    # 前後変位 m
    dy: float    # 左右変位 m
    dyaw: float  # 方位変化 rad
    dt: float    # 時間差 s
    timestamp_ns: int
```

---

## 10.7 World Outputs インターフェース

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
    bbox: tuple                     # 2D/3D候補。実装で型を固定
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
    """粗い測位から取得した周辺Map候補。HD/SD Mapがない場合はNoneまたは空。"""
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
    center_line: List[tuple]          # [(x, y), ...] Ego座標系 [m]
    curvature_profile: List[float]    # κ [1/m] per point, same length as center_line
    max_curvature: float              # max |κ| [1/m] in this node
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
    source_confidence: Dict[str, float]   # {"map":..., "sign":..., "road_mark":...}
    user_overspeed_tolerance_mps: float
    policy_margin_max_mps: float
```

---

## 10.8 Planner Outputs インターフェース

```python
@dataclass
class PlannerOutputs:
    trajectories: torch.Tensor   # [B, K, T, D]
    confidence: torch.Tensor     # [B, K]  (≠ safety probability)
    
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

## 10.9 Control Command インターフェース

```python
@dataclass
class ControlCommand:
    target_curvature: float      # 1/m
    target_speed: float          # m/s
    target_accel: float          # m/s^2
    speed_phase: str             # "ACCEL" / "CRUISE" / "DECEL"
    target_steer_angle: float    # rad (optional, vehicle-specific)
    
    limit_steer: bool = False    # 舵角制限が発動したか
    limit_accel: bool = False    # 加速度制限が発動したか
    feasibility_ok: bool = True  # 軌跡の実現可能性
    fallback_active: bool = False
    fallback_level: int = 0
    
    timestamp_ns: int = 0
```

---

## 10.10 主要テンソル形状一覧

| テンソル | 形状例 | 説明 |
|---|---|---|
| images | [1, 6, 3, 900, 1600] | 6カメラ |
| lidar_points | [1, 3, 50000, 5] | 3sweep, 50K点, (x,y,z,i,t) |
| radar_points | [1, 2, 500, 6] | 2sweep, 500点, (x,y,vx,vy,rcs,t) |
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

## 10.11 よくある実装の失敗

```text
失敗1: 座標系の不統一
  - Camera と LiDAR で x/y の方向が逆
  - BEV fusion が左右反転する
  → 対策: bev_grid.yaml でx=forward, y=leftを明示し、全実装で参照

失敗2: BEV origin の取り違え
  - rear axle基準と ego center基準の混在
  - 物体位置がずれる
  → 対策: origin は ego center (front axle 中央, 地面レベル) に統一

失敗3: yaw の符号ミス
  - counter-clockwise positive で定義しているのに clockwise で実装
  - 車線が左右逆に描画される
  → 対策: yaw convention を文書化し、単体テストで確認

失敗4: confidence を安全確率として使用
  - confidence = 0.9 を「90%の確率で安全」と誤解
  - External Evaluator なしで高confidence候補を選択
  → 対策: 必ず External Evaluator を通してから選択

失敗5: Temporal BEV のwarp忘れ
  - 過去フレームのBEVをego motionでwarpせずに使う
  - 時系列BEVが「ブレた」状態になる
  → 対策: TemporalBEVMemory.warp_to_current() を必ず使用

失敗6: feasibility_ok をチェックしない
  - ConverterがFalseを返しているのにPlannerが同じ候補を選択し続ける
  → 対策: External Evaluator の次回選択でfeasibilityを考慮

失敗7: BEV resolution の不一致
  - 0.4m/pixelで学習したモデルを0.5m/pixelで推論
  - 空間スケールが変わり全メトリクスが悪化
  → 対策: resolution を configs から読み込み、ハードコードしない

失敗8: Radar ego velocity補正の忘れ
  - Radarのvr がworld frameに補正されていない
  - 動体速度が自車速度分ずれる
  → 対策: RadarBEVEncoder入力前に ego velocity subtraction を行う
```

---

## 10.12 必須の単体テストとVisualizeチェック

```text
座標系テスト:
  - LiDAR点群をBEVに投影し、道路がBEV上で正しい向きに現れるか
  - 前方の物体が x > 0 の領域に現れるか

Temporal BEV テスト:
  - 直進10m後、前フレームのBEVをwarpして現フレームBEVと比較
  - 静的物体の位置がずれないか

Planner出力テスト:
  - 候補軌跡をBEV上に描画し、走行可能域内に収まるか
  - 候補が物理的に連続しているか（速度・加速度の確認）

External Evaluator テスト:
  - 既知の不安全軌跡を入力し、全候補がFailになることを確認
  - 既知の安全軌跡を入力し、Passになることを確認

Converter テスト:
  - 直線軌跡 → steer_angle ≈ 0 になることを確認
  - 一定曲率の軌跡 → 対応する steer_angle に変換されることを確認
```

---

## 10.13 MVP実装順序

```text
Step 1: BEV grid定義の文書化（最重要）
  → configs/bev_grid.yaml を作成し、全員が参照

Step 2: SensorPacket データ形式の確定
  → 型定義を先に書く

Step 3: CameraBEVEncoder の実装と可視化確認
  → LiDARと重ねて確認（道路が一致するか）

Step 4: LiDARBEVEncoder の実装と単体確認

Step 5: RadarBEVEncoder の実装（簡易版から）

Step 6: ModalityGateFuser の実装と3-modal融合確認

Step 7: TemporalBEVMemory の実装とwarpテスト

Step 8: BEV Information Heads（Static World から先に）

Step 9: Lane Topology Branch
  → Map候補 + ego motion + Temporal BEVを入力
  → 現在観測に整合した局所Lane Graphと自己位置を出力

Step 10: Dynamic World Branch + Agent Future

Step 11: VLA Planner（K=4から始める）

Step 12: External Evaluator（静的衝突チェックから先に）

Step 13: Trajectory Converter

Step 14: 統合テスト・Shadow Modeデータ収集へ
```

---

## 10.14 章のまとめ

```text
本章で設計した要素:
  1. インターフェース定義の重要性（座標系・タイムスタンプ・解像度）
  2. 必ず最初に決めるべき4つの定義
  3. リポジトリ構成
  4. 各モジュールのデータクラス定義
  5. 主要テンソル形状一覧
  6. よくある実装の失敗（8種類）
  7. 必須の単体テストとVisualizeチェック
  8. MVP実装順序（14ステップ）
```

次章では、実装ロードマップとShadow Mode検証を詳述する。
