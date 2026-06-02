# 第3章 Tri-modal BEV World Model

---

## 3.1 センサ同期とキャリブレーション

センサ融合の精度は、同期とキャリブレーションの精度に直結する。どれほど優れた融合アーキテクチャを作っても、入力のタイムスタンプがズレていたり、外部パラメータが間違っていれば精度は落ちる。

### 時刻同期

```text
推奨: IEEE 1588 PTP (Precision Time Protocol)
精度目標: 1ms以下
問題: CameraはGPU側でハードウェアタイムスタンプ、LiDARはコントローラ側で異なることが多い
対応: ソフトウェアタイムスタンプ補正 + ego motion補間
```

### Camera外部パラメータ

```text
camera_to_ego 変換行列 [4, 4]:
  - 単位: meter, radian
  - 推定方法: checker board キャリブレーション
  - 要求精度: 位置 < 1cm, 角度 < 0.1度
  - 注意: 車両振動による経時変化を定期再キャリブレーション
```

### LiDAR-Camera Extrinsic

```text
lidar_to_camera キャリブレーション:
  - 方法: テストターゲット or automatic target-less
  - 精度: 1cm以下（目安）
  - nuScenesキャリブレーション形式を参考にするとよい
```

### Radar Extrinsic

```text
radar_to_ego 変換行列:
  - Radarは通常バンパー埋め込みで固定
  - yaw方向のオフセットが特に重要（azimuth誤差の直接原因）
  - 車速補正（ego velocity subtraction）が必須
```

### キャリブレーション失効検知

```text
以下のアラートをモニタリングする:
- BEV上でのLiDAR点群とカメラセグメンテーションの不整合
- Radarターゲットとカメラ物体の位置ズレ
- 温度変化後の統計的なズレ量変化
→ 閾値超過でキャリブレーション要求フラグを立てる
```

---

## 3.2 Camera BEV Encoder

### 処理の流れ

```text
Multi-view images [B, N_cam, C, H_img, W_img]
    ↓
Backbone (Swin-T, ResNet50, EfficientDet等)
    ↓ 各カメラの特徴量 [B, N_cam, C_feat, H_feat, W_feat]
Neck (FPN)
    ↓ 多スケール特徴量 [B, N_cam, C_neck, H', W']
View Transformer (LSS/BEVFormer style)
    ↓ Camera BEV feature [B, C_bev, H_bev, W_bev]
```

### View Transformer の選択

| 方式 | 方法 | 長所 | 短所 |
|---|---|---|---|
| LSS (Lift-Splat-Shoot) | 深度予測 → 点群 → pillar scatter | 直感的、高速 | 深度精度に依存 |
| BEVFormer | Deformable Attention, BEV queryから2D参照 | 柔軟、時系列統合しやすい | 学習コスト高 |
| GKT | Frustum sampling + transformer | 省メモリ | 実装複雑 |
| BEVPoolv2 | pre-computation + scatter | 高速推論 | 事前計算が必要 |

本設計では、BEVFormer型またはLSS+Transformerのハイブリッドを基本とする。

### Backbone の選択

```text
軽量: EfficientNet-B2, MobileNetV3, GhostNet
標準: ResNet-50, ResNet-101
高精度: Swin-B, InternImage
推奨: ImageNetまたはNuImages pretrained → BEVで fine-tune
```

---

## 3.3 LiDAR BEV Encoder

### PointPillars ベースの処理

```text
LiDAR points [B, N_sweep, N_pts, D]
    ↓
Voxelization (voxel_size = [0.1m, 0.1m, 0.2m] 等)
    ↓ pillars [B, N_pillars, N_pts_per_pillar, D]
Pillar Feature Network (PFN)
    ↓ pillar features [B, N_pillars, C_pillar]
Scatter (pillars to BEV grid)
    ↓ LiDAR pseudo image [B, C_pillar, H_bev, W_bev]
2D Backbone (2nd conv stage)
    ↓ LiDAR BEV feature [B, C_lidar, H_bev, W_bev]
```

### Sparse Convolution の活用

点群は疎であり、密な2D畳み込みよりも疎な畳み込みが効率的なケースがある。

```text
推奨: spconv (SubMConv3d, SparseConv3d)
利用例: CenterPoint, VoxelNet, SECOND
BEVへの射影前にSparse 3D convを使う構成も可能
```

### Multi-sweep スタッキング

静止物体と動体の区別に、複数スキャンのスタッキングが有効である。

```text
Sweep stacking:
  N_sweeps = 3〜5
  各sweepにtime offset特徴を追加: [x, y, z, intensity, Δt]
  Δt = current_time - sweep_time
  これにより動体のモーションブラー様の差分が特徴として現れる
```

---

## 3.4 Radar BEV Encoder

### Radar の特性

```text
Radarが得意なこと:
  - 動体の相対速度 (Doppler)
  - 悪天候（雨・霧・砂嵐）での動作
  - 遠距離での金属物体の検出

Radarの弱点:
  - 空間解像度が低い
  - ghost target (多重反射・クラッタ)
  - 高さ情報なし（多くの場合）
  - 形状情報が乏しい
```

### Radar BEV エンコーダの実装

```text
方式A: Pillar approach（LiDAR同様）
  - radar_points を BEV 上のピラーに scatter
  - 速度情報 (vx, vy) を追加チャネルとして使う
  - 比較的高速で LiDAR と融合しやすい

方式B: Tensor approach（Range-Doppler をBEVに投影）
  - 一部のRadarはRange-Azimuth-Doppler tensorを出力
  - 直接BEVに変換する変換器を学習する
  - 情報量が多いが処理コスト高

本設計の推奨: 方式A（Pillar approach）
  入力特徴量: [x, y, vr_x, vr_y, rcs, t_offset]
```

### Ego speed 補正

RadarのドップラーはEgo速度を含む「relative velocity」として観測される。これをworld frame velocityに変換するには、Ego速度を引く必要がある。

```python
# Radar ego velocity補正（概略）
vr_world_x = radar_point.vr_x + ego_state.speed * cos(ego_yaw)
vr_world_y = radar_point.vr_y + ego_state.speed * sin(ego_yaw)
```

---

## 3.5 モダリティゲートによるTri-modal BEV融合

単純な加算や連結では、センサが欠損した場合に対応できない。  
モダリティゲートを持つことで、各センサの信頼度に応じた動的な重み付けが可能になる。

### モダリティゲートの構造

```text
Gate(m) = sigmoid(linear(quality_m))
  m ∈ {camera, lidar, radar}
  quality_m: センサ品質スコア（SNR, 点群密度, 画像露出等）

Fused BEV = Gate(camera) * F_cam + Gate(lidar) * F_lid + Gate(radar) * F_rad
```

### センサ品質スコア

```text
Camera quality:
  - exposure (露出): 過露出・暗闇を検知
  - blur score: ぼかし・雨滴検知
  - coverage: カメラビューが正常か

LiDAR quality:
  - point_density: 単位面積あたりの点数
  - range_coverage: 距離別の点群密度
  - blockage: 遮蔽物（汚れ、取り付け角ズレ）

Radar quality:
  - target_count: 検出ターゲット数（少なすぎる場合は疑う）
  - interference_flag: 他Radarからの干渉を検知
```

### センサ欠損時の動作

```text
LiDAR障害:
  - Gate(lidar) → 0 (自動閉鎖)
  - Camera BEVとRadar BEVで継続
  - 遠距離と形状精度が低下することを外部評価器に通知

カメラ一部障害:
  - 影響を受けるカメラのfeature maskを適用
  - BEV上の対応領域の不確実性を上昇させる

Radar欠損:
  - Gate(radar) → 0
  - 動体速度情報の信頼性が低下
  - 外部評価器の速度ベース評価を保守的に調整
```

---

## 3.6 時系列BEV Memory（Temporal BEV Memory）

単フレームのBEVだけでは、次の情報が取れない。

```text
取れないもの（単フレーム）:
  - 物体の動き（速度・加速度）
  - 静止しているように見えるが動いている物体
  - 消えた物体（一時的な遮蔽からの復帰）
  - レーン変更の途中状態
  - 信号の状態変化
```

時系列BEVを統合することで、これらを補完できる。

### Temporal Memory の実装選択

| 方式 | 方法 | 特徴 |
|---|---|---|
| ConvGRU | BEV特徴量にGRUを適用 | 実装簡単、効率的 |
| BEVFormer Temporal Attention | 過去フレームのBEVを自己注意で参照 | 柔軟、精度高 |
| Sparse Temporal Token | 重要なBEV tokenのみを保持し再利用 | 計算効率 |

---

### 3.6.1 標識・路面標識認識HeadとSpeed Constraint Token

速度制限の判断を安定化するため、Tri-modal BEVの上に標識認識専用Headを追加する。  
対象は「道路標識（交通標識）」「路面標示（路面標識）」の両方であり、認識結果はPlannerに直接使わず、**制約トークン（Speed Constraint Token）**として渡す。

### 認識対象

```text
対象カテゴリ:
  - 速度制限標識（例: 30, 40, 50, 60 km/h）
  - 速度制限解除標識
  - 学校区域・工事区間などの補助標識
  - 路面の速度表示（数字、ゾーン表示）
  - 一時停止・進入禁止など速度計画に影響する規制標識
```

### Head構成（推奨）

```text
入力:
  - Camera特徴（高解像度）
  - BEV fused特徴（位置整合）

出力:
  sign_boxes:          標識2D/3D候補
  sign_class:          標識クラス
  sign_speed_limit:    速度制限値 [m/s]
  sign_confidence:     信頼度
  road_mark_speed:     路面標示由来の速度制限値 [m/s]
  road_mark_confidence: 信頼度
```

### Token化とPlannerへの受け渡し

```text
Speed Constraint Token (T_speed_env):
  fields:
    source_priority: [map, sign, road_mark]
    legal_speed_limit_mps
    temporary_speed_limit_mps
    confidence
    ttl_frames

Plannerへの入力:
  CondFormerに T_speed_env を追加し、
  軌跡候補と速度プロファイル生成の両方を条件付けする。
```

### 研究・実装上の注意

```text
- 速度標識は見落としと誤検出が安全上クリティカルなため、
  単フレーム判定ではなく時系列安定化（Nフレーム投票、TTL）を使う。
- Map速度と標識速度が矛盾する場合は、保守側（低い速度）を優先し、
  外部評価器で最終確定する。
- 夜間・逆光・雨天で標識信頼度が下がるため、confidenceを必ず出力する。
```

### Ego Motion Warp

過去フレームのBEVを現在フレームに合わせるため、ego motionでwarpする必要がある。

```python
# BEV warp（概略）
# prev_bev: 前フレームのBEV (B, C, H, W)
# ego_delta: 前フレームから現フレームへのego位置変化 (dx, dy, dyaw)

# BEV gridのアフィン変換行列を作成
theta = ego_delta_to_affine(dx, dy, dyaw)  # (B, 2, 3)
warped_prev_bev = F.grid_sample(prev_bev, theta, mode='bilinear')
```

### 何フレーム保持するか

```text
L = 3〜5 フレーム が標準的
  - 少なすぎると（L=1）動体速度の推定精度が落ちる
  - 多すぎると（L>6）計算コストと古い情報の影響が増す
  - 10Hz動作で L=4 なら 400ms の時系列
  - 20Hz動作で L=4 なら 200ms の時系列
```

---

## 3.7 Planner向けBEVトークン構築

BEV featureは高解像度（200×200=40,000トークン）になりやすいが、Plannerに全部渡すと重すぎる。

### トークン削減の方針

```text
方法A: Sparse Pooling
  - 重要でないトークンを平均プーリングで削減
  - 重要度スコア（Motion Salienceや不確実性）で選別

方法B: Route-aware Sampling
  - ルート周辺のBEVトークンを高密度に保持
  - ルート外は粗くサンプリング

方法C: Hierarchical BEV
  - 近距離（0-30m）は高解像度
  - 中距離（30-80m）は中解像度
  - 遠距離（80m+）は低解像度

本設計: B + C の組み合わせを推奨
  - Planner入力トークン数: 2,000〜4,000 程度を目標
```

---

## 3.8 BEV情報Head群

BEVfeatureから、プランニングに必要な複数の情報を並列に推定する。

### 走行可能領域 Head

`bev_drivable` は走行可否の二値ではなく、**3クラスのセグメンテーション**として定義する。Tesla FSDの走行可能エリア表示と同じ考え方で、「物理的に進入できるが通常は走行すべきでない」領域を第3のクラスとして明示する。

例：縁石の手前は走行不可（急な段差）だが、縁石の向こうの歩道は縁石のない別の入口から進入でき、物理的には平坦である。この「歩道」部分が MARGINAL クラスにあたる。

```text
bev_drivable: [B, H, W]  ← 推論時のクラスラベル (argmax)
raw logits:  [B, 3, H, W] ← 学習・損失計算時の出力

クラス定義:
  0: NOT_DRIVABLE  = 走行不可能
       壁・高い段差・ガードレール・障害物など
       物理的に進入できない領域

  1: DRIVABLE      = 走行可能（通常走行域）
       車道・走行レーン・交差点内など
       通常走行で使うべき領域

  2: MARGINAL      = 走行可能だが走行すべきでない
       路肩・段差の先の歩道・駐車エリア隣接の空きスペースなど
       物理的には平坦・進入可能だが、通常は走行すべきでない領域
       緊急回避や特殊状況（障害物回避・緊急退避）では使用可

損失: cross-entropy loss (3-class, class weightあり)
  weight: NOT_DRIVABLE=1.0, DRIVABLE=1.0, MARGINAL=3.0 (稀なクラスを強調)

教師信号の生成:
  DRIVABLE: HD Map の車道領域
  NOT_DRIVABLE: HD Map の建物・路外 + LiDAR による急な段差（>15cm）検出
  MARGINAL: HD Map の路肩・歩道 + DRIVABLE/NOT_DRIVABLE の間の領域
           段差高さ5〜15cmのグレーゾーンも MARGINAL に含める
```

#### 外部評価器での扱い
```text
MARGINAL クラスの軌跡上の扱い:
  - NOT_DRIVABLE と同じく Hard Fail にはしない
  - ペナルティとして評価スコアを下げる（ソフトペナルティ）
  - DRIVABLE の候補が存在する場合は MARGINAL を通る候補を選ばない
  - 全 DRIVABLE 候補が衝突等でFail → MARGINAL 候補を選択（緊急回避）
```

### 占有Head

```text
bev_occupancy: [B, H, W]
  - 値: [0, 1] 連続値（占有確率）または二値
  - 静的物体の占有を表す
  - 動体は bev_agent_occ で別途扱う
```

### 車線 Head

```text
bev_lane: [B, H, W, C]
  - C=1: 車線存在確率
  - C=2: 左端/右端
  - C=3: 車線種別（実線/破線/境界）
  - 損失: segmentation loss
```

### エージェント占有 Head

```text
bev_agent_occ: [B, H, W]
  - 動的物体が占有するBEVセル
  - 歩行者、自転車、車両を含む
  - 静的なbev_occupancyとは分離する
```

### エージェント速度場 Head

```text
bev_agent_vel: [B, H, W, 2]
  - 各BEVセルでの動的物体の速度 (vx, vy)
  - 占有がない領域では意味を持たない（マスク処理）
  - RadarのDoppler速度を補完的に活用
```

### 不確実性 Head

```text
bev_uncertainty: [B, H, W]
  - モデルの予測不確実性
  - センサ欠損・遠距離・悪天候時に高くなるよう設計
  - Evidential Deep LearningまたはMC Dropoutで推定
  - 外部評価器の重要な入力
```

### ストップライン・横断歩道・信号機 Head

```text
bev_stopline: [B, H, W]
  - 停止線の存在確率
  - 交差点制御の重要な入力

bev_crosswalk: [B, H, W]
  - 横断歩道の存在確率
  - 歩行者対応の重要な入力

traffic_light_head:
  - traffic_light_boxes: 信号機の2D/3D候補
  - traffic_light_state: RED / YELLOW / GREEN / ARROW / UNKNOWN
  - traffic_light_confidence: 状態推定の信頼度
  - controlled_lane_ids: どのLaneNode/停止線に対応する信号か
  - time_since_seen: 一時遮蔽時のTTL管理
```

```text
設計上の注意:
  - 信号機は「検出」だけでなく、停止線・LaneNodeとの対応付けが重要
  - 矢印信号は進行方向ごとに有効/無効が変わるため、direction付きでToken化する
  - 逆光・大型車遮蔽・夜間の誤認識を考慮し、Temporal Memoryで状態を安定化する
  - 信号状態がUNKNOWNまたは低信頼度の場合、External Evaluatorは保守側に倒す
```

### Agent Detection Head（3D物体検出）

本設計のAgent Detectionは **2段階の分類ヘッド** を持つ。

- **メインクラス**（5クラス）: 行動計画・衝突回避に使用する
- **車種サブタイプ**（VEHICLEのみ）: 表示・UI用途に使用する

アノテーションコストの観点から、メインクラスは意図的に最小限に絞る。犬か猫かを区別する必要はなく、「動的障害物」として適切に回避できれば十分である。一方、車種の詳細分類は運転者・乗員への表示（UIパネルや可視化ツール等）で有用なため、VEHICLEに限り別途推定する。

```text
メインクラス（5クラス）← 行動計画に使用:
  0: VEHICLE        = 車両全般（乗用車・トラック・バス・二輪車を含む）
  1: PEDESTRIAN     = 歩行者（歩行・立位・しゃがみ込みを含む）
  2: CYCLIST        = 自転車・電動キックボード（速度・機動性がVEHICLEと異なる）
  3: ANIMAL         = 一般的動物（犬・鹿等。自転車と同程度の速度・大きさを想定）
  4: UNKNOWN        = 分類不能な障害物（工事資材・落下物・特殊物体等）

車種サブタイプ（5クラス）← VEHICLE 検出時のみ、表示用:
  0: CAR            = 乗用車・軽自動車・ミニバン・SUV
  1: TRUCK          = トラック・バン（大型・小型問わず）
  2: BUS            = バス・マイクロバス
  3: MOTORCYCLE     = 二輪車・原付
  4: SPECIAL        = 工事機械・農機具等の特殊車両

設計の意図:
  - Plannerはメインクラスのみを参照（5クラスに絞ることでアノテーションコスト削減）
  - UIパネル・ログビューアは vehicle_subtype を参照して車種アイコン等を表示
  - サブタイプの誤分類はPlannerに影響しない（表示上の問題のみ）
  - VEHICLE以外のクラスではサブタイプは null

アノテーション方針:
  - メインクラスのみで十分な場合はサブタイプを省略可（null扱い）
  - 分類が難しい物体はメインクラスを UNKNOWN に倒し、動いている場合は unknown_dynamic を付与する
```

#### 未知の動的物体への対応

上記5クラスに収まらない動的物体（例：路上を転がるゴミ箱、突然現れる作業員用の重機）は、クラス分類に頼らず **BEVの時間変化（Temporal BEV）** で対応する。

```text
未知動的物体の検出ロジック:
  Step 1: bev_agent_occ の時系列変化を監視
         現フレームと前L フレームのOccupancyの差分を計算

  Step 2: 動体判定
         同一BEVセルの占有が時系列で移動 → 動的物体と見なす
         速度場 bev_agent_vel の大きさが閾値（例: 0.5m/s）を超える

  Step 3: Detection Headで未分類でも、以下を推定:
         - 位置（BEV上のセル）
         - 速度ベクトル（bev_agent_vel から）
         - 占有サイズ（BEV占有セル数から概算）

  Step 4: 外部評価器へ「UNKNOWN_DYNAMIC」フラグ付きで渡す
         - Plannerは未分類障害物として保守的に扱う
         - bev_uncertainty を上昇させて保守的な評価を促す

設計の意図:
  クラス分類が正確でなくても、BEVの時間変化で「何かが動いている」
  ことは検出できる。Plannerに必要なのはその事実だけで十分である。
```

```text
Detection Head 出力:
  agent_states: List[AgentState]
    - bbox_bev:        (cx, cy, w, l, yaw)                           ← BEV座標系
    - velocity:        (vx, vy) m/s
    - heading:         float, rad
    - category:        "vehicle" / "pedestrian" / "cyclist" / "animal" / "unknown"
    - vehicle_subtype: "car" / "truck" / "bus" / "motorcycle" / "special" | null  ← VEHICLE時のみ、表示用
    - motion_state:    "parked" / "stationary" / "moving" / "accelerating" / "decelerating" / "turning"
    - existence_score: float [0, 1]
    - track_id:        str  ← フレーム間継続識別ID
    - occlusion_level: "none" / "partial" / "full"
    - unknown_dynamic: bool  ← BEV時系列から検出した未知動体フラグ
  agent_tokens: (N_agent, C_agent)  ← 後段 Future Predictor 用埋め込みベクトル

損失:
  メインクラス: CE loss（5クラス）
  サブタイプ  : CE loss（5クラス、category=="vehicle" のサンプルのみ）
  位置回帰   : L1/Smooth L1
  速度回帰   : L1
バックボーン: BEV feature をそのまま流用（追加Encoderなし）
クエリ数: 最大 200 agent（nuScenesベースライン準拠）
```

---

## 3.9 外部言語指示の取り込み

外部言語指示（ナビゲーション指示、ユーザーコマンド）は、BEVとは独立したチャネルから来る。

```text
入力: "次の交差点を右折してください"
    ↓
Text Encoder (CLIP or T5 based)
    ↓ T_inst: [B, N_tokens, D_lang]
CondFormer に入力
```

### 言語指示の種別

```text
レベル1: 識別子（最軽量）
  - 数値IDまたはone-hot
  - 例: TURN_RIGHT, CHANGE_LANE_LEFT, KEEP_LANE

レベル2: 構造化コマンド（DSL）
  - 設計した命令形式（Domain-Specific Language）
  - 例: {action: turn, direction: right, at: next_intersection}

レベル3: 軽量自然文
  - Short text encoder (CLIP style)
  - 例: "右折してください"

推奨: レベル2（DSL）を基本とし、レベル3は読み取り専用で補完
理由: 自然文の曖昧さを減らし、安全制御への影響を制限する
```

---

## 3.10 内部シーントークナイザー（Internal Scene Tokenizer）

シーンの状況を「言語的な圧縮表現」として持ち、Plannerへの条件付けに使う。

### なぜ内部言語トークンが必要か

```text
外部言語だけでは次の情報が取れない:
  - 現在の交差点の複雑さ
  - 近くにいる歩行者や自転車の意図
  - 前走車の急ブレーキの可能性
  - 自車のルート上のリスク

BEVの低次特徴量をそのまま渡すと:
  - Plannerに全BEVトークンを渡す必要がある
  - 計算コストが高い
  - 意味的な構造が失われる

内部シーントークンは:
  - シーンの要約を潜在空間で表す
  - Plannerに少数のトークンで渡せる
  - VLM teacher でシーン要約を蒸留できる
```

### シーントークナイザーの構造

```text
Input: BEV feature + BEV information heads outputs
    ↓
Scene Query Tokens (固定数: N_scene = 8〜16)
    ↓ Cross-Attention over BEV
Scene Token Features [B, N_scene, D_scene]
    ↓ (optional) Debug text decoder
T_scene_text (デバッグ用の文字列)
```

### VLM Teacherによる蒸留

DriveLMやLLaVAを使って、シーントークンの学習を補助する。

```text
Teacher: large VLM (DriveLM, LLaVA-v2)
  - Input: raw camera images
  - Output: scene description text + risk analysis

Student: Internal Scene Tokenizer
  - Target: produce similar "meaning" in latent space
  - Loss: feature distillation + optional text decode loss
```

---

## 3.11 CondFormer（条件付けTransformer）

BEV情報、外部言語指示（T_inst）、内部シーントークン（T_scene）を統合し、Plannerへの条件付けベクトルを構築する。

### CondFormerの構造

```text
Input:
  - BEV tokens (spatial)
  - T_inst (外部言語)
  - T_scene (内部シーン)
  ※ agent情報はここでは使わない。動き予測（agent_futures）をPlannerで直接参照する

CondFormer:
  Layer 1: Cross-attention over BEV
  Layer 2: Cross-attention over T_inst
  Layer 3: Cross-attention over T_scene
  Layer 4: Self-attention (全条件の統合)
  Layer 5 (optional): Instruction priority gate
    → 外部指示が強い場合は T_inst の重みを増幅
```

### 指示優先ゲート

```text
右折指示のような強い指示を確実に反映させるために、
外部言語指示の強度に応じて注意重みを増幅するゲートを追加する。

instruction_priority_gate = sigmoid(linear(T_inst_cls))
CondFormer_out = (1 + instruction_priority_gate) * language_attention_out
                + standard_attention_out
```

---

## 3.12 VLA Plannerへの接続

CondFormerの出力は、VLA PlannerのQ（query）の初期化、またはcross-attentionのkey/valueとして入力される。

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

## 3.13 Lane Topology World Model（詳細）

レーントポロジー（車線のネットワーク構造）は、計画判断の根拠になる最重要な静的情報の一つである。

### 3.13.1 なぜレーントポロジーが必要か

```text
BEVのセグメンテーションだけでは次が分からない:
  - この車線は先でどこへ続くか
  - 右折専用か直進か
  - 交差点でどの車線に入るべきか
  - ルート上の次の車線はどれか
  - Uターン可能か

Lane Topology がこれらを補完する。
```

### 3.13.2 テスラ的学習によるレーン誘導

Teslaの公開情報から学べる点として、HD Mapに依存しないレーンの「Attention型」学習がある。  
BEV特徴量からレーン境界の存在確率と幾何（曲線）を直接予測し、その先の接続グラフも予測する。

```text
レーン幾何の予測:
  - Bezier曲線または多項式でレーン中心線を表す
  - 各制御点の座標と存在確率を出力
  - 教師: HD Map or LiDARからの推定レーン境界

トポロジーの予測:
  - 各レーンノードが次のノードと繋がるかをClassificationで予測
  - Adjacency matrix as binary prediction task
```

### 3.13.3 Lane Topology Branch の設計

```text
Inputs:
  - Current BEV feature [B, C, H, W]
  - Temporal BEV memory [B, T_hist, C, H, W] (ego motion warp済み)
  - Map candidates [B, N_map, D_map] (HD/SD Mapが利用可能な場合)
  - Ego motion [B, T_hist, 3] (dx, dy, dyaw)
  - Ego state [speed, yaw_rate, accel, steer]
    ↓
Map Candidate Encoder + Ego Motion Encoder
    ↓
BEV-map cross-attention / temporal fusion
    ↓
Lane Query Tokens (N_lane queries)
    ↓ Deformable Cross-Attention over BEV
Lane Node Features [B, N_lane, D_lane]
    ↓ MLP heads
Lane Geometry Output: [B, N_lane, N_ctrl_pts, 2]  (Bezier control points in BEV)
Lane Existence Score: [B, N_lane]
    ↓ GNN (Graph Neural Network)
Lane Topology Edges [B, N_lane, N_lane]  (adj matrix: successor/predecessor/adjacent)
Ego Pose in Lane Graph:
  - current_lane_candidates
  - lateral_offset
  - heading_error
  - lane_graph_confidence
```

このBranchは、地図を直接の正解として貼り付けるのではなく、現在のBEV観測と時系列BEV memoryを主入力にして、地図候補を補助情報として照合する。
ego motionは過去BEVを現在座標へwarpするためだけでなく、短時間で観測が欠けたレーン接続を時系列的に保つためにも使う。

### 3.13.4 Lane Graph表現

```text
LaneNode:
  lane_id: str
  center_line: List[(x, y)]        # BEV座標（Ego座標系、単位m）
  curvature_profile: List[float]   # center_line の各点における曲率 κ [1/m]
                                   # 同じ長さ。正値=左カーブ, 負値=右カーブ
  max_curvature: float             # |κ| の最大値 [1/m]（速度制限チェック用）
  advisory_speed_mps: float        # 曲率ベース推奨速度 sqrt(a_lat_max/|κ|)
                                   # κ≒0 の場合は speed_limit を参照
  lane_type: str                   # "normal" / "turn" / "merge" / "split"
  direction: str                   # "forward" / "left_turn" / "right_turn" / "u_turn"
  speed_limit: float               # 規制速度 m/s, -1 if unknown
  traffic_light_controlled: bool
  existence_score: float           # モデルの信頼度

LaneEdge:
  src_id: str
  dst_id: str
  edge_type: str              # "successor" / "predecessor" / "left_adj" / "right_adj"
  weight: float               # 接続強度
```

#### 曲率プロファイルの計算方法

`curvature_profile` は `center_line` の各点から3点円弧法で計算する。

```python
import numpy as np

def compute_curvature_profile(center_line: list) -> list:
    """polylineの各点の曲率 κ [1/m] を3点円弧法で計算する"""
    pts = np.array(center_line)  # (N, 2)
    kappas = [0.0]               # 先頭点は前後がないので 0
    for i in range(1, len(pts) - 1):
        p0, p1, p2 = pts[i-1], pts[i], pts[i+1]
        # 3点を通る円の曲率
        d01 = np.linalg.norm(p1 - p0)
        d12 = np.linalg.norm(p2 - p1)
        d02 = np.linalg.norm(p2 - p0)
        area = abs(np.cross(p1 - p0, p2 - p0))  # 平行四辺形面積の半分
        if d01 * d12 * d02 < 1e-6:
            kappas.append(0.0)
        else:
            R = (d01 * d12 * d02) / (2.0 * area + 1e-9)  # 外接円半径
            # 符号: 左カーブ正（cross productの符号で判定）
            sign = np.sign(np.cross(p1 - p0, p2 - p1))
            kappas.append(float(sign / R))
    kappas.append(0.0)           # 末尾点
    return kappas

A_LAT_MAX = 2.0  # m/s^2（快適性上限の横加速度）

def advisory_speed(max_kappa: float, speed_limit: float) -> float:
    """曲率から推奨速度を計算する"""
    if abs(max_kappa) < 1e-4:  # 直線に近い
        return speed_limit
    v_adv = (A_LAT_MAX / abs(max_kappa)) ** 0.5
    return min(v_adv, speed_limit)
```

#### 曲率プロファイルの用途

```text
1. Planner への速度制約入力:
   - target_lane_sequence の先読みレーンの max_curvature から
     advisory_speed_mps を算出し、Planner の速度プロファイル生成に渡す
   - コーナー手前の早期減速を促す

2. External Evaluator の快適性チェック:
   - 軌跡の局所曲率がレーンの curvature_profile を大幅に超えていないか確認
   - |κ_traj - κ_lane| > 0.05 [1/m] の場合はソフトペナルティ

3. Converter のサニティチェック:
   - Converter が軌跡から計算した曲率と LaneNode.curvature_profile を照合
   - 大幅な乖離はセンサ誤認識 or 地図不整合の兆候として検出
```

### 3.13.5 Map-assisted Lane Topology 推定

HD MapまたはSD Mapが利用できる場合は、地図情報を周辺レーン候補として取り込み、オンライン推定と照合する。
ただし、本設計では地図を強いPriorとして固定せず、センサから観測されたBEV空間のレーン構造と整合する候補だけを局所レーントポロジーに採用する。

```text
Map candidates:
  - 粗い測位位置から周辺道路セグメントを取得
  - HD Mapなら車線幾何・接続候補を使う
  - SD Mapなら道路中心線・交差点構造・規制情報を候補として使う
  - 地図更新漏れ・工事・臨時規制で不正確なことがある

Observed BEV lane evidence:
  - カメラ/LiDAR/Radar統合BEVからレーン境界、停止線、走行可能域を推定
  - Temporal BEV memoryをego motionで現在座標へwarpして統合
  - 遮蔽や一時的な検出落ちを時系列情報で補う

推定方法:
  - Map候補とBEV観測をcross-attentionまたはmatching costで照合する
  - 整合する候補はレーンID・接続・規制情報の曖昧性解消に使う
  - 整合しない候補は弱める、または地図不整合として扱う
  - 地図なし: BEV観測 + Temporal memory + ego motionのみで推定する
  - 出力は現在観測に整合した局所Lane Graphと、そのGraph上の自車姿勢である
```

### 3.13.6 Route-aware Lane Selection

ナビゲーションのルートと、現在のレーントポロジーを照合して、どの車線に自車が向かうべきかを選択する。

```text
Route waypoints: (global lat/lon sequence)
    ↓ 粗い測位で周辺Map候補を検索
Route Segment Candidates: (road_segment_id_1, road_segment_id_2, ...)
    ↓ 局所Lane Topology Graphと照合
Route Lane Sequence Candidates: (lane_id_1, lane_id_2, lane_id_3, ...)
    ↓ Lane Topology Graph で最短路探索
target_lane_sequence: PlannerへのInput
```

### 3.13.7 左折・右折レーンの処理

交差点での車線選択は、特に注意を要する。

```text
右折:
  - 右折専用レーンが存在する場合はそちらへ
  - 待機位置（対向車を待つ位置）を学習データから学ぶ
  - 歩行者の横断歩道を確認するタイミング

左折（日本/右ハンドル）:
  - 対向車を待つ
  - 交差点センターへの進入位置
  - 横断歩道・信号の確認
```

### 3.13.8 出口・分岐・合流の処理

```text
High Exit:
  - 速度を落とし、出口車線へ寄る
  - 分岐直前では早めに方向を決定

分岐（Y字）:
  - どちらの方向に進むかをルートから決定
  - レーントポロジーで"split"ノードを認識

合流:
  - 加速度を調整して合流タイミングを計算
  - Dynamic Risk Mapで後続車の速度を確認
```

### 3.13.9 地図候補と観測が不整合な場合の不確実性処理

```text
Map候補とBEV観測から推定した局所Lane Graphの差分が大きい場合:
  - map_mismatch_score を計算
  - 閾値超過時: bev_uncertainty を上昇
  - 外部評価器: より保守的な軌跡を選択
  - ODD判定: 地図不整合フラグを立てる
  - Lane Topology Branch: 観測優先で局所Lane Graphを再推定
  - ログ: 地図不整合の発生を記録し、地図更新または再学習キューへ追加
```

### 3.13.10 出力インターフェース

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

### 3.13.11 軽量更新戦略

Lane Topologyは全フレームで再計算する必要はない。

```text
Slow update (1-5 Hz):
  - Lane Topology全体の再推定
  - Map候補とBEV観測の整合性チェック
  - 局所Lane Graph上の自車姿勢推定

Medium update (10Hz):
  - target_lane_sequenceの更新（ルート変化時）
  - map_mismatch_scoreの更新

Fast layer (20Hz+):
  - 最後のLane Topology推定結果をキャッシュして使用
  - Ego motionでwarpして使用
  - 局所Lane Graphに対する横方向オフセットとヨー角差を更新
```

---

## 3.14 センサ別BEV精度の限界と注意点

| センサ | BEV精度の限界 | 注意点 |
|---|---|---|
| Camera | 遠距離・夜間・逆光で落ちる | 日照条件による性能変化を事前に評価 |
| LiDAR | 雨・霧で点群が減少 | 悪天候での点群密度モニタリング |
| Radar | Ghostターゲット、低解像度 | Radar-only BEVは形状情報なし |

これらの限界を事前に理解した上で、センサ品質ゲートと不確実性推定を設計することが重要である。

---

## 3.15 実装上の重要な注意点（BEV座標系の統一）

BEV座標系の定義が全モジュールで統一されていないと、融合が正しく動かない。

```text
必ず共通定義すべき項目:
  - x: forward, y: left (あるいは x: right, y: forward — どちらかに統一)
  - BEV grid の origin: ego 中心か、rear axle か
  - BEV grid の resolution: meters/pixel (例: 0.5m/pixel で 200x200 → 100m x 100m)
  - yaw の正方向: counter-clockwise か clockwise か

実装でよくある失敗:
  - CameraとLiDARで別々の座標系を使い、BEV上でズレる
  - yawの符号が逆になり、車線が左右反転する
  - originがrear axleのmoduleとego centerのmoduleが混在する
```

**この座標系定義は、プロジェクトの最初に文書化し、全実装者が参照できる場所に置く。** 後から直すコストが非常に高い。

---

## 3.16 章のまとめ

```text
本章で設計した要素:
  1. センサ同期・キャリブレーション
  2. Camera BEV Encoder (view transformer)
  3. LiDAR BEV Encoder (PointPillars + sparse conv)
  4. Radar BEV Encoder (pillar approach + velocity)
  5. Modality-gated Tri-modal BEV Fusion
  6. Temporal BEV Memory (ego motion warp付き)
  7. Planner向けBEVトークン削減
  8. BEV Information Heads (9種類、うちAgent Detection Head含む)
  9. External Language Instruction の取り込み
 10. Internal Scene Tokenizer (VLM teacher)
 11. CondFormer (条件付けTransformer)
 12. Lane Topology World Model (詳細設計)
```

次章では、Dynamic Object World Model と未来リスク推定を詳述する。
