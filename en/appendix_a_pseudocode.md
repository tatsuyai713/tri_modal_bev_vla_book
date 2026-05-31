# Appendix A Forward-Pass Pseudocode

---

## A.1 Full Forward Pass

```python
"""
Tri-Modal BEV VLA Planner - Full Forward Pass (Pseudocode)
Coordinate system: Ego-centric BEV, x=forward, y=left, unit=m
"""

import torch
import torch.nn as nn
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SensorPacket:
    """Sensor inputs for one cycle"""
    camera_images: torch.Tensor       # (N_cam, 3, H, W), float32, [0,1]
    lidar_points: torch.Tensor        # (N_pts, 4), float32, [x,y,z,intensity]
    radar_points: torch.Tensor        # (N_rad, 5), float32, [x,y,z,v_r,intensity]
    timestamp_ns: int
    calibration: "CalibrationData"


@dataclass
class EgoState:
    """Ego vehicle state"""
    velocity_mps: float               # Forward speed [m/s]
    yaw_rate_radps: float             # Yaw rate [rad/s]
    acceleration_mps2: float          # Forward acceleration [m/s^2]
    timestamp_ns: int


@dataclass
class MapCandidates:
    """Nearby map candidates retrieved from coarse localization and route. May be empty if unavailable."""
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
        # Step 1: Sensor Synchronization & BEV Encoding
        # ============================================================
        
        # 1a. Camera BEV Encoder (BEVFormer-style)
        # camera_features: (N_cam, C, H_f, W_f) -- backbone features
        camera_features = self.camera_backbone(sensor.camera_images)  # EfficientNet/ConvNext, etc.
        
        # Cross-attention: camera features -> BEV grid
        bev_camera = self.cam_bev_encoder(
            camera_features=camera_features,
            calibration=sensor.calibration,
            bev_queries=self.bev_queries,    # (Bx, By, C_bev)
        )  # -> (Bx, By, C_bev)
        
        # 1b. LiDAR BEV Encoder (Pillar-based)
        lidar_pillars = self.lidar_pillar_encoder(
            sensor.lidar_points,
            voxel_size=(0.2, 0.2, 4.0),
            point_cloud_range=(-51.2, -51.2, -3.0, 51.2, 51.2, 1.0),
        )  # -> (Bx, By, C_lidar)
        bev_lidar = self.lidar_bev_encoder(lidar_pillars)  # -> (Bx, By, C_bev)
        
        # 1c. Radar BEV Encoder
        bev_radar = self.radar_bev_encoder(
            sensor.radar_points,
            voxel_size=(0.4, 0.4),
        )  # -> (Bx, By, C_radar)  includes velocity channels
        
        # ============================================================
        # Step 2: Modality Gate Fusion
        # ============================================================
        
        # Compute health score per modality
        cam_health = self.cam_health_monitor(camera_features)      # (N_cam,) in [0,1]
        lidar_health = self.lidar_health_monitor(sensor.lidar_points)  # scalar in [0,1]
        radar_health = self.radar_health_monitor(sensor.radar_points)  # scalar in [0,1]
        
        # Gated fusion
        bev_fused = self.modality_gate(
            bev_camera, bev_lidar, bev_radar,
            cam_health, lidar_health, radar_health,
        )  # -> (Bx, By, C_bev)
        
        # ============================================================
        # Step 3: Temporal BEV Memory
        # ============================================================
        
        # Transform past BEV to current coordinate frame (Ego Motion Compensation)
        bev_memory_aligned = bev_memory.get_aligned(
            ego_state=ego_state,
            n_frames=5,
        )  # -> (T, Bx, By, C_bev)

        ego_motion_history = bev_memory.get_ego_motion_history(n_frames=5)
        # -> (T, 3) [dx, dy, dyaw]
        
        # Temporal Fusion
        bev_temporal = self.temporal_fusion(
            bev_current=bev_fused,
            bev_history=bev_memory_aligned,
        )  # -> (Bx, By, C_bev)
        
        # Update Memory (for the next cycle)
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
        #   .uncertainty      (Bx, By)       in [0,1]

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
        #     LaneNode includes curvature_profile (per-point kappa [1/m]) and
        #     max_curvature (max |kappa|) / advisory_speed_mps (curvature-based recommended speed)
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
        # agent_states[i].track_id: unique ID that persists across the same agent
        
        # Agent Future Trajectory Prediction
        # Note: agent_history is NOT passed from outside.
        #       agent_future_predictor maintains GRU hidden states internally per track_id.
        #       The external interface only sees the current frame. (Trajectron++ / MOTR style)
        agent_futures = self.agent_future_predictor(
            agent_tokens=agent_tokens,
            agent_ids=[s.track_id for s in agent_states],  # key for looking up internal state
            bev_context=bev_temporal,
            n_modes=6,
            horizon_s=5.0,
        )
        # Internal processing: h_prev = hidden_states[track_id] (zeros if not found)
        #           h_new  = GRU(agent_token, h_prev)
        #           hidden_states[track_id] = h_new  <- carried over to the next frame
        #           h_new -> BEV cross-attn -> inter-agent cross-attn -> multi-mode decoder
        # agent_futures: (N_agent, 6, T_future, 2)  [x, y] per timestep
        
        # Dynamic Risk Map: project future dynamic occupancy onto BEV
        dynamic_risk_map = self.dynamic_risk_builder(
            agent_futures=agent_futures,
            horizon_s=5.0,
        )  # -> (Bx, By, T_horizon)
        
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
        )  # (L_scene, C_text) -- distilled compact representation
        
        # ============================================================
        # Step 8: CondFormer (Language-BEV Conditioning)
        # ============================================================
        
        # BEV tokens + local lane topology aligned to current observation
        planner_bev_tokens = self.bev_to_planner_tokens(bev_temporal)
        lane_tokens = self.lane_topology_encoder(
            lane_graph=lane_graph,
            ego_pose_in_lane_graph=lane_graph.ego_pose_in_lane_graph,
        )
        
        # CondFormer: highlight BEV tokens conditioned on text
        # Note: agent information is NOT used here. The Planner directly references
        #       the motion predictions (agent_futures).
        conditioned_tokens = self.cond_former(
            bev_tokens=planner_bev_tokens,
            lane_tokens=lane_tokens,
            text_ext=T_ext,
            text_scene=T_scene,
        )  # -> (N_token, C_bev)
        
        # ============================================================
        # Step 9: K-query VLA Planner
        # ============================================================
        
        K = 8  # number of candidates
        
        # Transformer Decoder with K queries
        # agent_futures: future trajectory candidates obtained from Agent Detection -> Agent Future Predictor
        #   The Planner references where agents are "heading to," not just "where they are"
        # dynamic_risk_map: future risk map projecting agent_futures onto BEV
        trajectory_candidates = self.vla_planner(
            queries=self.planner_queries,        # (K, C_query) -- learned queries
            context=conditioned_tokens,          # (N_token, C_bev)  BEV+lane+text
            agent_futures=agent_futures,         # (N_agent, 6, T_future, 2)  future trajectory candidates
            dynamic_risk_map=dynamic_risk_map,   # (Bx, By, T_horizon)  future risk map
            ego_state=ego_state,
            horizon_s=5.0,
            dt_s=0.5,
        )
        # trajectory_candidates: (K, T, 3)
        #   [:, :, 0]: x [m], ego-centric forward
        #   [:, :, 1]: y [m], ego-centric left
        #   [:, :, 2]: v [m/s]
        
        # ============================================================
        # Step 10: Safe Selection by External Evaluator
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
        
        # If all candidates fail
        if self.external_evaluator.all_failed(safety_scores):
            selected_trajectory = self.fallback_trajectory(ego_state)
        
        # ============================================================
        # Step 11: Build Outputs
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

## A.2 External Evaluator Pseudocode

```python
class ExternalEvaluator:
    """Safety evaluator separated from the neural network"""
    
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
            
            # 1. Static collision check
            static_collision = self._check_static_collision(
                traj, bev_static.static_occupancy
            )
            
            # 2. Drivable area check (3-class support)
            # NOT_DRIVABLE(0): Hard Fail
            # MARGINAL(2): Soft penalty only (score reduction)
            off_drivable = self._check_drivable(
                traj, bev_static.drivable_label,  # [Bx, By] class 0/1/2
                hard_fail_class=0,   # NOT_DRIVABLE
                soft_penalty_class=2,  # MARGINAL
            )
            
            # 3. Dynamic collision risk check
            dynamic_collision = self._check_dynamic_collision(
                traj, dynamic_risk_map
            )
            
            # 4. Kinematic constraint check
            kinematic_ok = self._check_kinematics(traj, ego_state)
            
            # 5. Comfort check (trajectory curvature vs. lane curvature profile)
            comfort_ok = self._check_comfort(traj)
            
            # 6. Curvature-based advisory speed check
            # Check whether ego speed far exceeds the advisory_speed_mps of the current lane
            adv_speed_ok = True
            if lane_graph is not None:
                current_lane_id = lane_graph.ego_pose_in_lane_graph.current_lane_candidates
                for ln in lane_graph.lane_nodes:
                    if ln.lane_id in current_lane_id:
                        # Soft penalty if traj speed exceeds advisory_speed_mps by more than 1.2x
                        max_traj_speed = float(traj[:, 2].max())
                        if max_traj_speed > ln.advisory_speed_mps * 1.2:
                            adv_speed_ok = False
                        break
            
            if static_collision or off_drivable or dynamic_collision:
                scores[k] = -1.0  # Fail
            elif not kinematic_ok or not comfort_ok:
                scores[k] = 0.5   # Marginal
            elif not adv_speed_ok:
                scores[k] = 0.8   # Curvature speed exceeded: soft penalty (not disqualified)
            else:
                # Score = safety + comfort
                safety_score = self._safety_score(traj, bev_static, dynamic_risk_map)
                comfort_score = self._comfort_score(traj)
                scores[k] = 0.7 * safety_score + 0.3 * comfort_score
        
        # Select the highest-score candidate
        best_k = scores.argmax()
        return trajectories[best_k], scores
    
    def all_failed(self, scores: torch.Tensor) -> bool:
        return (scores < 0).all()
```

---

## A.3 Trajectory-to-Steering Converter Pseudocode

```python
class TrajectoryToSteeringConverter:
    """Deterministic conversion: trajectory -> steering angle"""
    
    def __init__(self, wheel_base_m: float = 2.7, dt_s: float = 0.1):
        self.L = wheel_base_m
        self.dt = dt_s
    
    def convert(
        self,
        trajectory: torch.Tensor,  # (T, 3): [x, y, v]
        ego_state: EgoState,
    ) -> ControlCommand:
        
        # Step 1: Validity check
        if not self._is_valid(trajectory):
            return ControlCommand(steer_rad=0.0, accel_mps2=-2.0, valid=False)
        
        v = ego_state.velocity_mps
        
        # Step 2: Calculate lookahead distance
        look_ahead_m = max(2.0, min(v * 1.0, 10.0))
        
        # Step 3: Get lookahead point
        target_point = self._get_lookahead_point(trajectory, look_ahead_m)
        
        # Step 4: Compute curvature (Pure Pursuit)
        dx, dy = target_point[0], target_point[1]
        ld = (dx**2 + dy**2)**0.5  # actual lookahead distance
        if ld < 1e-3:
            kappa = 0.0
        else:
            kappa = 2.0 * dy / (ld**2)  # [1/m]
        
        # Step 4b: Sanity check against LaneNode.curvature_profile
        # lane_kappa: value of LaneNode.curvature_profile at the ego position
        # if abs(kappa - lane_kappa) > 0.15: raise bev_uncertainty
        
        # Step 5: Convert to steering angle via bicycle model
        steer_rad = torch.atan(torch.tensor(kappa * self.L))
        
        # Step 6: Clamp steering angle
        steer_rad = steer_rad.clamp(-0.6, 0.6)  # +/-34 degrees
        
        # Step 7: Speed control
        v_target = trajectory[1, 2]  # target speed at the next step
        accel = self._speed_controller(v, v_target)
        
        return ControlCommand(
            steer_rad=float(steer_rad),
            accel_mps2=float(accel),
            valid=True,
        )
    
    def _get_lookahead_point(self, traj, look_ahead_m):
        """Return the trajectory point closest to the lookahead distance"""
        for i in range(len(traj)):
            dist = (traj[i, 0]**2 + traj[i, 1]**2)**0.5
            if dist >= look_ahead_m:
                return traj[i, :2]
        return traj[-1, :2]
    
    def _speed_controller(self, v_current, v_target, k=1.0):
        """PD speed controller"""
        error = v_target - v_current
        accel = k * error
        return max(-5.0, min(accel, 3.0))  # clamp
```
