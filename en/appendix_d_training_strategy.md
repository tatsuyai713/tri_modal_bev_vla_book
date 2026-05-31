# Appendix D Training Strategy and OSS Weight Reuse

---

## D.1 Overall Training Policy

The Tri-Modal BEV VLA Planner has a complex modular architecture, making it virtually impossible to train all modules simultaneously from scratch. Follow these principles:

```text
Principle 1: Warm Start First
  - Start from existing publicly available OSS weights
  - Training from scratch is limited to LiDAR/Camera heads only

Principle 2: Stage-wise Training
  - Separate training into stages per module
  - Freeze weights from the previous stage and update only the current stage
  - Joint Fine-tuning is done last

Principle 3: Data Quality > Data Volume
  - Training on a small amount of high-quality data is better than training on
    a large amount of low-quality data
  - Enforce the filtering pipeline rigorously (Chapter 12)

Principle 4: Carefully Design Loss Functions
  - When multiple losses conflict, spend time tuning the weight balance
  - Continuously monitor whether any loss is stuck at zero

Principle 5: Observe Learning Curves Before Advancing to the Next Stage
  - Move to the next stage after val loss stops improving
  - Apply early stopping when overfitting begins
```

---

## D.2 Six-Stage Training Plan

| Stage | Target Modules | Data | Frozen Range |
|---|---|---|---|
| Stage 1 | BEV Backbone (Camera + LiDAR) | nuScenes Detection | None (new training) |
| Stage 2 | BEV Information Heads | nuScenes + in-house data | Backbone frozen |
| Stage 3 | K-query Planner | Human trajectory data | Stage 1+2 frozen |
| Stage 4 | External Language Conditioning | Language + trajectory data | Stage 1+2 frozen, only CondFormer updated |
| Stage 5 | Internal Scene Tokenizer | VLM teacher data | Stage 1+2+3 frozen |
| Stage 6 | Joint Fine-tuning | All data mixed | All modules trainable (low LR) |

---

## D.3 Stage 1: BEV Backbone Training

```text
Objective:
  - Use BEVFormer / BEVFusion weights as a warm start
  - Adapt to in-house sensor configuration (number of cameras, lens, LiDAR type)

Warm Start Sources:
  BEVFusion (MIT version):
    - Integrated BEV Encoder for Camera + LiDAR
    - Trained on nuScenes
    - Download: https://github.com/mit-han-lab/bevfusion (checkpoint in README)

  BEVFormer (v2):
    - Camera-only BEV Encoder
    - Trained on nuScenes
    - Download: https://github.com/fundamentalvision/BEVFormer

Post-Warm-Start Adjustments:
  - Changing number of cameras (6->8, etc.): Re-initialize input layer of Camera BEV Encoder
  - Changing LiDAR type (velodyne->ouster, etc.): Adjust voxel size of Pillar Encoder
  - Changing BEV grid range: Re-train positional encoding

Training Configuration:
  Optimizer: AdamW (lr=1e-4, weight_decay=1e-2)
  Scheduler: CosineAnnealingLR
  Batch size: 4-8 (depends on GPU memory)
  Epochs: 24 (nuScenes standard)
  Augmentation:
    - Camera: random brightness/contrast, horizontal flip (nuScenes left-right flip)
    - LiDAR: random rotation, random scale
```

---

## D.4 Stage 2: BEV Information Heads Training

```text
Prerequisite: Load Stage 1 weights, freeze Backbone

Targets:
  - drivable_head
  - static_occupancy_head
  - occ_flow_head
  - lane_center_head
  - stopline_head
  - speed_limit_head
  - dynamic_occ_head
  - uncertainty_head

Loss Function:
  L_stage2 = 
    lambda_driv * BCE(pred_drivable, gt_drivable) +
    lambda_occ  * BCE(pred_occ, gt_occ) +
    lambda_flow * L1(pred_flow, gt_flow) +
    lambda_lane * BCE(pred_lane, gt_lane) +
    lambda_stop * BCE(pred_stop, gt_stop) +
    lambda_spd  * CE(pred_speed_limit, gt_speed_limit) +
    lambda_dyn  * BCE(pred_dyn_occ, gt_dyn_occ)

Recommended Weights:
  lambda_driv = 1.0
  lambda_occ  = 2.0  (slightly larger to handle class imbalance)
  lambda_flow = 0.5
  lambda_lane = 1.0
  lambda_stop = 2.0  (larger for the rare class)
  lambda_spd  = 1.0
  lambda_dyn  = 2.0

Automatic Teacher Signal Generation:
  gt_drivable: Project 3-class labels onto BEV from HD Map + LiDAR height change detection
    - DRIVABLE(1): HD Map roadway area
    - NOT_DRIVABLE(0): HD Map off-road + LiDAR height change >15cm
    - MARGINAL(2): HD Map shoulder/sidewalk + height change 5-15cm gray zone
  gt_occ: Static occupancy from LiDAR point cloud (ground removed)
  gt_flow: Computed from LiDAR tracking trajectories of moving objects
  gt_lane: Project HD Map lane centers onto BEV
  gt_stop: Project HD Map stop lines onto BEV
  gt_speed_limit: HD Map speed limits
  gt_dyn_occ: Project dynamic object 3D bboxes onto BEV

Training Configuration:
  Optimizer: AdamW (lr=3e-4)
  Epochs: 12
  Data: nuScenes + in-house data mix (in-house ratio 50%)
```

---

## D.5 Stage 3: K-query Planner Training

```text
Prerequisite: Load Stage 1+2 weights, freeze them

Targets:
  - CondFormer (language-free version)
  - K-query VLA Planner Decoder

Inputs:
  - BEV tokens (Stage 1+2 output, no gradient)
  - Lane topology tokens
  - planner_queries (K queries, learnable)

Outputs:
  - (K, T, 3) trajectory candidates

Loss Function:
  L_planner = L_traj + L_speed + L_comfort + L_safety_reg

  L_traj (trajectory target loss):
    - MHP Loss (Multi-Head Prediction Loss)
    - Flow gradients only from the candidate closest to the human trajectory (Winner-takes-all)
    L_traj = min_k L2(traj_k, traj_human)

  L_speed (speed profile loss):
    L_speed = L1(v_pred, v_human)
    - L1 difference from the human speed profile

  L_comfort (comfort regularization):
    L_comfort = lambda * (mean(|a_x|) + mean(|a_y|))
    - Suppress excessive acceleration and steering

  L_safety_reg (safety regularization):
    L_safety_reg = mean(max(0, margin - dist_to_occ))
    - Penalty when distance to static occupancy is below the margin

Training Configuration:
  Optimizer: AdamW (lr=1e-3 for queries, 3e-4 for others)
  Epochs: 24
  K = 8
  T = 10 (5 seconds, 0.5-second intervals)
```

---

## D.6 Stage 4: External Language Conditioning (CondFormer) Training

```text
Prerequisite: Freeze Stage 1+2 weights, also load Stage 3 Planner weights

Targets:
  - Text Encoder
  - CondFormer (language-conditioned part)
  - Instruction Priority Gate

Data:
  - Driving logs + DSL labels (right turn / left turn / lane change / stop / go straight)
  - Annotate each driving segment with DSL annotations

Training:
  - Train to output trajectories that match the intent of the DSL input
  - Loss based on agreement with the trajectory selected by the existing Planner (Stage 3)

  L_lang = L2(traj_lang_conditioned, traj_stage3_best)

Notes:
  - Verify that trajectory quality does not degrade without language input (default instruction)
  - Confirm that metrics improve with language input compared to without
```

---

## D.7 Stage 5: Internal Scene Tokenizer Training

```text
Targets:
  - Scene Tokenizer
  - Distillation from VLM teacher (Knowledge Distillation)

Method:
  - Use text descriptions T_scene_teacher generated by a VLM (LLaVA, etc.) from camera images as teacher
  - Train Scene Tokenizer output text to approach T_scene_teacher

Loss:
  L_scene = Cross_Entropy(T_scene_pred, T_scene_teacher)

Alternatively, feature distillation without text conversion:
  L_scene = MSE(scene_token_pred, VLM_encoder(T_scene_teacher))

Data:
  - Camera images + VLM-generated text (see automatic annotation in Chapter 12)
  - Scale: recommend 100,000+ scenes

Notes:
  - Stage 5 is optional. Skip in MVP and add later.
  - Low-quality VLM teacher becomes noise
```

---

## D.8 Stage 6: Joint Fine-tuning

```text
Final stage: all modules trained simultaneously at a low learning rate.

Freezing Strategy:
  - BEV Backbone: low LR (1e-5)
  - BEV Heads: low LR (3e-5)
  - Planner: standard LR (1e-4)
  - CondFormer: standard LR (1e-4)
  - Scene Tokenizer: low LR (3e-5)

Loss:
  L_joint = L_stage2 + L_stage3 + L_stage4 + L_stage5 (weighted sum of each loss)

Notes:
  - Joint Fine-tuning assumes each module has already converged
  - If one module trains disproportionately, the whole system may collapse
  - Monitor val_loss across all modules

Training Configuration:
  Optimizer: AdamW (different lr for different modules)
  Epochs: 6-12
  Use early stopping aggressively
```

---

## D.9 OSS Weight Reuse Guide (Per Module)

### BEVFusion -> BEV Encoder (Camera + LiDAR)

```python
import torch
from mmdet3d.models import build_model

# Load BEVFusion official weights
checkpoint = torch.load("bevfusion-det.pth", map_location="cpu")
model = build_model(cfg.model)
model.load_state_dict(checkpoint["state_dict"], strict=False)
# strict=False allows loading even when camera count, etc. differ
# Mismatched keys are ignored (re-initialized from scratch)
```

### UniAD -> Agent Future Predictor (Motion Transformer)

```python
# Extract only the motion_head part of UniAD for warm start
checkpoint = torch.load("uniad-stage1.pth", map_location="cpu")
state_dict = {
    k.replace("motion_head.", ""): v
    for k, v in checkpoint["state_dict"].items()
    if k.startswith("motion_head.")
}
model.agent_future_predictor.load_state_dict(state_dict, strict=False)
```

### DriveLM -> VLM Teacher / Language Conditioning

```python
# Use the QFormer-based Language Encoder part of DriveLM
checkpoint = torch.load("drivelm-qa.pth", map_location="cpu")
# Extract language encoder (BLIP2/QFormer-based) weights
lang_state_dict = {
    k.replace("language_model.", ""): v
    for k, v in checkpoint.items()
    if k.startswith("language_model.")
}
model.text_encoder.load_state_dict(lang_state_dict, strict=False)
```

---

## D.10 Freeze/Train Table by Module

| Module | Stage1 | Stage2 | Stage3 | Stage4 | Stage5 | Stage6 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| Camera Backbone | Train | Frozen | Frozen | Frozen | Frozen | Train(low LR) |
| LiDAR Pillar Encoder | Train | Frozen | Frozen | Frozen | Frozen | Train(low LR) |
| BEV Encoder (Fusion) | Train | Frozen | Frozen | Frozen | Frozen | Train(low LR) |
| Temporal BEV Memory | Train | Frozen | Frozen | Frozen | Frozen | Train(low LR) |
| BEV Heads | - | Train | Frozen | Frozen | Frozen | Train(low LR) |
| Dynamic Object Head | - | Train | Frozen | Frozen | Frozen | Train(low LR) |
| Agent Future Predictor | - | Train | Frozen | Frozen | Frozen | Train(low LR) |
| Lane Topology Encoder | - | Train | Frozen | Frozen | Frozen | Train(low LR) |
| Text Encoder | - | - | - | Train | Frozen | Train(low LR) |
| CondFormer | - | - | Train(simple) | Train | Frozen | Train |
| Planner Queries (K) | - | - | Train | Frozen | Frozen | Train |
| VLA Planner Decoder | - | - | Train | Frozen | Frozen | Train |
| Scene Tokenizer | - | - | - | - | Train | Train(low LR) |

---

## D.11 Heuristic Label Generation for Parked/Static/Dynamic

Training Agent Futures requires distinguishing between moving objects, stopped vehicles, and parked vehicles.

```python
def classify_agent_motion(agent_track: list) -> str:
    """
    agent_track: [AgentState, ...] (time series)
    returns: "dynamic" | "stopped" | "parked"
    """
    velocities = [s.velocity for s in agent_track]
    
    # Maximum speed
    v_max = max(abs(v) for v in velocities)
    
    # Fraction of time stopped
    stopped_ratio = sum(1 for v in velocities if abs(v) < 0.3) / len(velocities)
    
    if v_max > 1.0:
        # Some movement occurred
        if stopped_ratio > 0.8:
            return "stopped"  # Mostly stopped but moved (waiting at signal, etc.)
        else:
            return "dynamic"
    else:
        # Barely moved
        return "parked"
```

---

## D.12 Building Agent Relevance Teacher Signal

Teacher signal for weighting agents the Planner should attend to.

```python
def compute_agent_relevance(ego_trajectory, agent_state) -> float:
    """
    Compute relevance from ego trajectory and agent future position
    """
    # Distance: higher relevance for closer agents
    dist = distance(ego_trajectory[0], agent_state.position)
    dist_score = max(0.0, 1.0 - dist / 30.0)
    
    # Intersection with ego trajectory: higher relevance for intersecting paths
    intersection_score = check_path_intersection(
        ego_trajectory, agent_state.predicted_path
    )
    
    # Speed: higher relevance for moving agents
    v_score = min(1.0, agent_state.velocity / 5.0)
    
    relevance = 0.4 * dist_score + 0.4 * intersection_score + 0.2 * v_score
    return relevance
```

---

## D.13 Low-Cost Training Configuration (R9700 + 32GB RAM)

Practical training configuration when available GPU is limited.

```text
Hardware example: AMD Ryzen 9 7900X + RTX 4090 24GB (or RTX 3090 24GB)

Countermeasures:
  1. Reduce BEV grid resolution
     - 512x512 -> 256x256 or 128x128
     - Performance drops but sufficient for functional verification

  2. Reduce batch size
     - Batch size = 1 or 2
     - Use Gradient Accumulation (8-16 steps) to achieve effective batch size

  3. FP16 Mixed Precision Training
     Use torch.cuda.amp.autocast() + GradScaler
     - Reduces VRAM usage by roughly half

  4. Train modules separately
     - Train only Stage 1, then only Stage 2, one stage at a time

  5. Lightweight Backbone
     - MobileNetV3 or EfficientNet-B0 instead of ResNet-50
     - BEVFormer-tiny (publicly available lightweight version)

  6. Reduce K (number of candidates)
     - K=8 -> K=4

Example command:
  python train.py \
    --stage 1 \
    --bev_resolution 256 \
    --batch_size 2 \
    --grad_accum 8 \
    --fp16 \
    --backbone efficientnet_b0
```

---

## D.14 Warm Start Pattern Comparison

| Strategy | Initial Accuracy | Training Time | Risk | Recommended |
|---|---|---|---|---|
| From scratch | Low | Very long | High | x |
| BEVFusion warm start only | Medium | Medium | Medium | triangle |
| BEVFusion + UniAD warm start | High | Short | Low | best |
| Full fine-tuning of UniAD | High | Medium | Medium | good |

**Recommendation**: Use BEVFusion (MIT version) for the BEV Encoder and UniAD's motion_head for Agent Futures.

---

## D.15 Recommended Hybrid Route

```text
Step 1: Load BEVFusion (MIT) weights -> use for Camera + LiDAR BEV Encoder
Step 2: Load UniAD weights -> use for Agent Future Predictor, BEV Heads
Step 3: Stage 2: Fine-tune BEV Heads with nuScenes + in-house data
Step 4: Stage 3: Train Planner with human trajectory data
Step 5: Stage 4: Train CondFormer with language data
Step 6: Stage 5: Train Scene Tokenizer with VLM teacher data (optional)
Step 7: Stage 6: Joint Fine-tuning (low LR)
```

---

## D.16 Notes on Using Published Checkpoints

```text
Notes when using public checkpoints:

1. Verify the license (see Appendix C)
   - Models trained on nuScenes may be affected by CC BY-NC-SA 4.0

2. Model architecture version mismatch
   - The architecture of the code that created the checkpoint may differ from yours
   - Load with strict=False and always check loaded / not-loaded keys

3. Differences in input format
   - Different number of cameras, resolution, normalization method
   - Different LiDAR point cloud format (x,y,z,intensity vs x,y,z,ring, etc.)

4. Coordinate system differences
   - nuScenes: right-hand system, x=forward, z=up
   - Verify that it matches your in-house sensor configuration

5. Output range differences
   - Whether values are logit vs post-sigmoid vs post-softmax
   - Different BEV resolution and range

Example verification code:
  checkpoint = torch.load("checkpoint.pth")
  model_keys = set(model.state_dict().keys())
  ckpt_keys = set(checkpoint["state_dict"].keys())
  
  missing = model_keys - ckpt_keys
  unexpected = ckpt_keys - model_keys
  
  print(f"Missing keys ({len(missing)}): {list(missing)[:5]}")
  print(f"Unexpected keys ({len(unexpected)}): {list(unexpected)[:5]}")
```

---

## D.17 Appendix D Summary

```text
Elements designed in this appendix:
  1. Overall training policy (5 principles)
  2. Six-stage training plan (Backbone->Heads->Planner->Language->Scene->Joint)
  3. Details of each stage (loss functions, training settings, warm start sources)
  4. OSS weight reuse guide (BEVFusion/UniAD/DriveLM)
  5. Freeze/Train table (all modules x all stages)
  6. Heuristic label generation for Parked/Static/Dynamic
  7. Building agent relevance teacher signal
  8. Low-cost training configuration (RTX 4090 24GB)
  9. Warm start pattern comparison
  10. Recommended hybrid route
  11. Notes on using checkpoints
```
