# 付録A フォワードパスの疑似コード

---

## A.1 全体フォワードパス

```python
"""
Tri-Modal BEV VLA Planner - Full Forward Pass (Pseudocode)
座標系: Ego-centric BEV, x=前方, y=左方, 単位=m
"""

import torch
import torch.nn as nn
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SensorPacket:
    """1サイクル分のセンサ入力"""
    camera_images: torch.Tensor       # (N_cam, 3, H, W), float32, [0,1]
    lidar_points: torch.Tensor        # (N_pts, 4), float32, [x,y,z,intensity]
    radar_points: torch.Tensor        # (N_rad, 5), float32, [x,y,z,v_r,intensity]
    timestamp_ns: int
    calibration: "CalibrationData"


@dataclass
class EgoState:
    """自車状態"""
    velocity_mps: float               # 前進速度 [m/s]
    yaw_rate_radps: float             # ヨーレート [rad/s]
    acceleration_mps2: float          # 前向き加速度 [m/s^2]
    timestamp_ns: int


@dataclass
class MapCandidates:
    """粗い測位とルートから検索された周辺Map候補。利用不能なら空でよい。"""
    tokens: Optional[torch.Tensor]     # (N_map, C_map)
    road_segment_ids: List[str]
    source: str                        # "hd_map" / "sd_map" / "none"
    confidence: float


class TriModalBEVVLAPlanner(nn.Module):
    def forward(
        self,
        sensor: SensorPacket,
        ego_state: EgoState,
        bev_memory: "TemporalBEVMemory",
        text_instruction: Optional[str],
        map_candidates: Optional[MapCandidates],
    ) -> "PlannerOutputs":
        
        # ============================================================
        # Step 1: センサ同期・BEV Encoding
        # ============================================================
        
        # 1a. Camera BEV Encoder (BEVFormer-style)
        # camera_features: (N_cam, C, H_f, W_f) -- backbone特徴
        camera_features = self.camera_backbone(sensor.camera_images)  # EfficientNet/ConvNext等
        
        # Cross-attention: カメラ特徴 → BEVグリッド
        bev_camera = self.cam_bev_encoder(
            camera_features=camera_features,
            calibration=sensor.calibration,
            bev_queries=self.bev_queries,    # (Bx, By, C_bev)
        )  # → (Bx, By, C_bev)
        
        # 1b. LiDAR BEV Encoder (Pillar-based)
        lidar_pillars = self.lidar_pillar_encoder(
            sensor.lidar_points,
            voxel_size=(0.2, 0.2, 4.0),
            point_cloud_range=(-51.2, -51.2, -3.0, 51.2, 51.2, 1.0),
        )  # → (Bx, By, C_lidar)
        bev_lidar = self.lidar_bev_encoder(lidar_pillars)  # → (Bx, By, C_bev)
        
        # 1c. Radar BEV Encoder
        bev_radar = self.radar_bev_encoder(
            sensor.radar_points,
            voxel_size=(0.4, 0.4),
        )  # → (Bx, By, C_radar)  速度チャンネルを含む
        
        # ============================================================
        # Step 2: Modality Gate Fusion
        # ============================================================
        
        # 各モダリティの健全性スコア計算
        cam_health = self.cam_health_monitor(camera_features)      # (N_cam,) ∈ [0,1]
        lidar_health = self.lidar_health_monitor(sensor.lidar_points)  # scalar ∈ [0,1]
        radar_health = self.radar_health_monitor(sensor.radar_points)  # scalar ∈ [0,1]
        
        # Gated fusion
        bev_fused = self.modality_gate(
            bev_camera, bev_lidar, bev_radar,
            cam_health, lidar_health, radar_health,
        )  # → (Bx, By, C_bev)
        
        # ============================================================
        # Step 3: Temporal BEV Memory
        # ============================================================
        
        # 過去BEVを現在座標系に変換（Ego Motion Compensation）
        bev_memory_aligned = bev_memory.get_aligned(
            ego_state=ego_state,
            n_frames=5,
        )  # → (T, Bx, By, C_bev)

        ego_motion_history = bev_memory.get_ego_motion_history(n_frames=5)
        # → (T, 3) [dx, dy, dyaw]
        
        # Temporal Fusion
        bev_temporal = self.temporal_fusion(
            bev_current=bev_fused,
            bev_history=bev_memory_aligned,
        )  # → (Bx, By, C_bev)
        
        # MemoryをUpdateする（次サイクル用）
        bev_memory.update(bev_temporal, timestamp_ns=sensor.timestamp_ns)
        
        # ============================================================
        # Step 4: BEV Information Heads
        # ============================================================
        
        bev_out = self.bev_heads(bev_temporal)
        # bev_out contains:
        #   .drivable_area    (Bx, By, 3)    per-class logits (0=NOT_DRIVABLE, 1=DRIVABLE, 2=MARGINAL)
        #   .drivable_label   (Bx, By)       argmax class label
        #   .static_occupancy (Bx, By)       logits
        #   .occ_flow         (Bx, By, 2)    [dx, dy]
        #   .lane_center      (Bx, By)       logits
        #   .stopline         (Bx, By)       logits
        #   .speed_limit      (Bx, By, S)    S=speed classes
        #   .dynamic_occ      (Bx, By)       logits (dynamic objects)
        #   .uncertainty      (Bx, By)       ∈ [0,1]

        # ============================================================
        # Step 5: Lane Topology Branch
        # ============================================================

        lane_graph = self.lane_topology_head(
            bev_current=bev_fused,
            bev_temporal=bev_temporal,
            bev_history=bev_memory_aligned,
            map_candidates=map_candidates,
            ego_motion_history=ego_motion_history,
            ego_state=ego_state,
        )
        # lane_graph contains:
        #   .lane_nodes / .lane_edges
        #     LaneNode には curvature_profile (per-point κ [1/m]) および
        #     max_curvature (最大|κ|) / advisory_speed_mps (曲率ベース推奨速度) が含まれる
        #   .target_lane_sequence
        #   .ego_pose_in_lane_graph
        #   .map_mismatch_score
        
        # ============================================================
        # Step 6: Dynamic Object Branch
        # ============================================================
        
        # Agent Detection + Tracking
        agent_tokens, agent_states = self.dynamic_head(bev_temporal)
        # agent_tokens: (N_agent, C_agent)
        # agent_states: List[AgentState] with bbox, velocity, class_id, vehicle_subtype, id
        # agent_states[i].track_id: 同一エージェントを跨ぐ一意ID
        
        # Agent Future Trajectory Prediction
        # 注: agent_history は外部から渡さない。
        #     agent_future_predictor がGRU hidden stateをtrack_idごとに内部保持する。
        #     外部インターフェースは現フレームのみ。（Trajectron++ / MOTR方式）
        agent_futures = self.agent_future_predictor(
            agent_tokens=agent_tokens,
            agent_ids=[s.track_id for s in agent_states],  # 内部状態の引き当てキー
            bev_context=bev_temporal,
            n_modes=6,
            horizon_s=5.0,
        )
        # 内部処理: h_prev = hidden_states[track_id] (なければ zeros)
        #           h_new  = GRU(agent_token, h_prev)
        #           hidden_states[track_id] = h_new  ← 次フレームに引き継ぐ
        #           h_new → BEV cross-attn → agent間 cross-attn → multi-mode decoder
        # agent_futures: (N_agent, 6, T_future, 2)  [x, y] per timestep
        
        # Dynamic Risk Map: 将来の動体占有をBEVに投影
        dynamic_risk_map = self.dynamic_risk_builder(
            agent_futures=agent_futures,
            horizon_s=5.0,
        )  # → (Bx, By, T_horizon)
        
        # ============================================================
        # Step 7: Language Processing
        # ============================================================
        
        # External instruction (route, command)
        if text_instruction is not None:
            T_ext = self.text_encoder(text_instruction)  # (L_ext, C_text)
        else:
            T_ext = self.default_instruction_token  # (1, C_text)
        
        # Internal scene description (from VLM teacher, or cached)
        T_scene = self.scene_tokenizer(
            camera_images=sensor.camera_images,
            bev_context=bev_temporal,
        )  # (L_scene, C_text) -- 蒸留済みコンパクト表現
        
        # ============================================================
        # Step 8: CondFormer (Language-BEV Conditioning)
        # ============================================================
        
        # BEVトークン + 現在観測に整合した局所レーントポロジー
        planner_bev_tokens = self.bev_to_planner_tokens(bev_temporal)
        lane_tokens = self.lane_topology_encoder(
            lane_graph=lane_graph,
            ego_pose_in_lane_graph=lane_graph.ego_pose_in_lane_graph,
        )
        
        # CondFormer: テキスト条件でBEVトークンを強調
        # ※ agent情報はここでは使わない。動き予測（agent_futures）をPlannerで直接参照する
        conditioned_tokens = self.cond_former(
            bev_tokens=planner_bev_tokens,
            lane_tokens=lane_tokens,
            text_ext=T_ext,
            text_scene=T_scene,
        )  # → (N_token, C_bev)
        
        # ============================================================
        # Step 9: K-query VLA Planner
        # ============================================================
        
        K = 8  # 候補数（発展時は DiffusionDrive で M=16〜32 サンプル、§6.10 参照）
        
        # K個のクエリを使ったTransformer Decoder
        # agent_futures: Agent Detection → Agent Future Predictor で得た将来軌跡候補
        #   Plannerはエージェントが「どこにいるか」ではなく「どこへ向かうか」を参照する
        # dynamic_risk_map: agent_futuresをBEVに投影した将来リスクマップ
        trajectory_candidates = self.vla_planner(
            queries=self.planner_queries,        # (K, C_query) -- 学習済みクエリ
            context=conditioned_tokens,          # (N_token, C_bev)  BEV+lane+text
            agent_futures=agent_futures,         # (N_agent, 6, T_future, 2)  将来軌跡候補
            dynamic_risk_map=dynamic_risk_map,   # (Bx, By, T_horizon)  将来リスクマップ
            ego_state=ego_state,
            horizon_s=5.0,
            dt_s=0.5,
        )
        # trajectory_candidates: (K, T, 3)
        #   [:, :, 0]: x [m], ego-centric前方
        #   [:, :, 1]: y [m], ego-centric左方
        #   [:, :, 2]: v [m/s]
        
        # ============================================================
        # Step 10: External Evaluator による安全選択
        # ============================================================
        
        selected_trajectory, safety_scores = self.external_evaluator(
            trajectories=trajectory_candidates,
            bev_static=bev_out,
            lane_topology=lane_graph,
            dynamic_risk_map=dynamic_risk_map,
            ego_state=ego_state,
            agent_futures=agent_futures,
        )
        # selected_trajectory: (T, 3)
        # safety_scores: (K,)
        
        # 全候補がFallbackの場合
        if self.external_evaluator.all_failed(safety_scores):
            selected_trajectory = self.fallback_trajectory(ego_state)
        
        # ============================================================
        # Step 11: 出力の構築
        # ============================================================
        
        return PlannerOutputs(
            trajectory_candidates=trajectory_candidates,
            selected_trajectory=selected_trajectory,
            safety_scores=safety_scores,
            bev_outputs=bev_out,
            lane_topology=lane_graph,
            agent_states=agent_states,
            agent_futures=agent_futures,
            dynamic_risk_map=dynamic_risk_map,
            fallback_active=self.external_evaluator.all_failed(safety_scores),
            bev_uncertainty=bev_out.uncertainty,
            debug={
                "cam_health": cam_health,
                "lidar_health": lidar_health,
                "radar_health": radar_health,
                "T_scene_text": self.scene_tokenizer.last_text if self.debug_mode else None,
            }
        )
```

---

## A.2 External Evaluator の疑似コード

```python
class ExternalEvaluator:
    """ニューラルネットワーク外に分離された安全評価器"""
    
    def __call__(
        self,
        trajectories: torch.Tensor,     # (K, T, 3)
        bev_static: BEVOutputs,
        dynamic_risk_map: torch.Tensor,  # (Bx, By, T_horizon)
        ego_state: EgoState,
        agent_futures: torch.Tensor,
    ):
        K = trajectories.shape[0]
        scores = torch.zeros(K)
        
        for k in range(K):
            traj = trajectories[k]  # (T, 3)
            
            # 1. 静的衝突チェック
            static_collision = self._check_static_collision(
                traj, bev_static.static_occupancy
            )
            
            # 2. 走行可能領域チェック（3クラス対応）
            # NOT_DRIVABLE(0): Hard Fail
            # MARGINAL(2): ソフトペナルティのみ（スコアダウン）
            off_drivable = self._check_drivable(
                traj, bev_static.drivable_label,  # [Bx, By] class 0/1/2
                hard_fail_class=0,   # NOT_DRIVABLE
                soft_penalty_class=2,  # MARGINAL
            )
            
            # 3. 動体衝突リスクチェック
            dynamic_collision = self._check_dynamic_collision(
                traj, dynamic_risk_map
            )
            
            # 4. キネマティクス制約チェック
            kinematic_ok = self._check_kinematics(traj, ego_state)
            
            # 5. 快適性チェック（軌跡曲率 vs レーン曲率プロファイル）
            comfort_ok = self._check_comfort(traj)
            
            # 6. 曲率ベース推奨速度チェック
            # 自車が走行中のレーンの advisory_speed_mps を大幅超過していないか確認
            adv_speed_ok = True
            if lane_graph is not None:
                current_lane_id = lane_graph.ego_pose_in_lane_graph.current_lane_candidates
                for ln in lane_graph.lane_nodes:
                    if ln.lane_id in current_lane_id:
                        # trajの速度が advisory_speed_mps の 1.2倍を超える場合はソフトペナルティ
                        max_traj_speed = float(traj[:, 2].max())
                        if max_traj_speed > ln.advisory_speed_mps * 1.2:
                            adv_speed_ok = False
                        break
            
            if static_collision or off_drivable or dynamic_collision:
                scores[k] = -1.0  # Fail
            elif not kinematic_ok or not comfort_ok:
                scores[k] = 0.5   # Marginal
            elif not adv_speed_ok:
                scores[k] = 0.8   # 曲率速度超過: ソフトペナルティ（排除はしない）
            else:
                # スコア = safety + comfort
                safety_score = self._safety_score(traj, bev_static, dynamic_risk_map)
                comfort_score = self._comfort_score(traj)
                scores[k] = 0.7 * safety_score + 0.3 * comfort_score
        
        # 最高スコアの候補を選択
        best_k = scores.argmax()
        return trajectories[best_k], scores
    
    def all_failed(self, scores: torch.Tensor) -> bool:
        return (scores < 0).all()
```

---

## A.3 Trajectory-to-Steering Converter の疑似コード

```python
class TrajectoryToSteeringConverter:
    """軌跡 → ステアリング角 の決定論的変換"""
    
    def __init__(self, wheel_base_m: float = 2.7, dt_s: float = 0.1):
        self.L = wheel_base_m
        self.dt = dt_s
    
    def convert(
        self,
        trajectory: torch.Tensor,  # (T, 3): [x, y, v]
        ego_state: EgoState,
    ) -> ControlCommand:
        
        # Step 1: 有効性チェック
        if not self._is_valid(trajectory):
            return ControlCommand(steer_rad=0.0, accel_mps2=-2.0, valid=False)
        
        v = ego_state.velocity_mps
        
        # Step 2: ルックアヘッド距離の計算
        look_ahead_m = max(2.0, min(v * 1.0, 10.0))
        
        # Step 3: ルックアヘッド点の取得
        target_point = self._get_lookahead_point(trajectory, look_ahead_m)
        
        # Step 4: 曲率の計算（Pure Pursuit）
        dx, dy = target_point[0], target_point[1]
        ld = (dx**2 + dy**2)**0.5  # 実際のルックアヘッド距離
        if ld < 1e-3:
            kappa = 0.0
        else:
            kappa = 2.0 * dy / (ld**2)  # [1/m]
        
        # Step 4b: LaneNode.curvature_profile との照合（サニティチェック）
        # lane_kappa: 自車位置に対応する LaneNode.curvature_profile の値
        # if abs(kappa - lane_kappa) > 0.15: bev_uncertainty を引き上げる
        
        # Step 5: 自転車モデルによるステアリング角変換
        steer_rad = torch.atan(torch.tensor(kappa * self.L))
        
        # Step 6: ステアリング角のクランプ
        steer_rad = steer_rad.clamp(-0.6, 0.6)  # ±34度
        
        # Step 7: 速度制御
        v_target = trajectory[1, 2]  # 次ステップの目標速度
        accel = self._speed_controller(v, v_target)
        
        return ControlCommand(
            steer_rad=float(steer_rad),
            accel_mps2=float(accel),
            valid=True,
        )
    
    def _get_lookahead_point(self, traj, look_ahead_m):
        """軌跡からルックアヘッド距離に最も近い点を返す"""
        for i in range(len(traj)):
            dist = (traj[i, 0]**2 + traj[i, 1]**2)**0.5
            if dist >= look_ahead_m:
                return traj[i, :2]
        return traj[-1, :2]
    
    def _speed_controller(self, v_current, v_target, k=1.0):
        """PD速度コントローラ"""
        error = v_target - v_current
        accel = k * error
        return max(-5.0, min(accel, 3.0))  # クランプ
```
