# Appendix C Reference OSS Repositories and Papers

---

## C.1 BEV Perception & Fusion

### BEVFormer
- **Use**: Camera BEV Encoder (core of this design)
- **URL**: https://github.com/fundamentalvision/BEVFormer
- **Paper**: "BEVFormer: Learning Bird's-Eye-View Representation from Multi-Camera Images via Spatiotemporal Transformers" (Li et al., ECCV 2022)
- **License**: Apache 2.0
- **How used in this design**: Primary backbone for Camera-to-BEV conversion. Pre-trained weights are publicly available for warm starting.

### BEVFusion (MIT)
- **Use**: Camera + LiDAR Fusion (recommended integration method in this design)
- **URL**: https://github.com/mit-han-lab/bevfusion
- **Paper**: "BEVFusion: A Simple and Robust LiDAR-Camera Fusion Framework" (Liang et al., NeurIPS 2022)
- **License**: MIT
- **How used in this design**: Integrated fusion of Camera BEV and LiDAR BEV. TensorRT-optimized implementation available.

### BEVFusion (NuScenes)
- **Use**: Alternative BEVFusion implementation (for nuScenes)
- **URL**: https://github.com/ADLab-AutoDrive/BEVFusion
- **Paper**: "BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation" (Liu et al., ICRA 2023)
- **License**: Apache 2.0

---

## C.2 Occupancy & Segmentation

### FusionOcc / SurroundOcc
- **Use**: BEV Occupancy prediction (reference for bev_occupancy head in this design)
- **SurroundOcc URL**: https://github.com/weiyithu/SurroundOcc
- **Paper**: "SurroundOcc: Multi-Camera 3D Occupancy Prediction for Autonomous Driving" (Wei et al., ICCV 2023)
- **License**: Apache 2.0

### OpenOccupancy
- **Use**: Occupancy evaluation benchmark
- **URL**: https://github.com/JeffWang987/OpenOccupancy

---

## C.3 End-to-End Planning

### UniAD
- **Use**: Unified Autonomous Driving framework (reference for the design philosophy of this design)
- **URL**: https://github.com/OpenDriveLab/UniAD
- **Paper**: "Planning-oriented Autonomous Driving" (Hu et al., CVPR 2023 Best Paper)
- **License**: Apache 2.0
- **How used in this design**: Reference for integrated design of BEV heads, agent prediction, and Planner. Warm-start weights are available.

### DriveLM
- **Use**: Language-conditioned planning (reference for the VLA Planner in this design)
- **URL**: https://github.com/OpenDriveLab/DriveLM
- **Paper**: "DriveLM: Driving with Graph Visual Question Answering" (Sima et al., ECCV 2024)
- **License**: Apache 2.0
- **How used in this design**: Method for generating VLM teacher signals for T_scene. The nuScenes DriveLM data can be used.

### DiffusionDrive
- **Use**: Diffusion-based planning (reference as an alternative approach to K-candidates)
- **URL**: https://github.com/hustvl/DiffusionDrive
- **Paper**: "DiffusionDrive: Truncated Diffusion Model for End-to-End Autonomous Driving" (Liao et al., NeurIPS 2024)
- **License**: Apache 2.0

### SparseDrive
- **Use**: Sparse BEV planning (reference for lightweight design in this design)
- **URL**: https://github.com/swc-17/SparseDrive
- **Paper**: "SparseDrive: End-to-End Autonomous Driving via Sparse Scene Representation" (Sun et al., 2024)
- **License**: Apache 2.0

### BridgeAD
- **Use**: Integration of natural language and VLA planning
- **URL**: https://github.com/crossmodalgroup/bridgead
- **Paper**: "BridgeAD: Bridging the Gap between End-to-End Autonomous Driving and Language" (2024)

---

## C.4 Trajectory Prediction

### HiVT / MTR / Wayformer
- **Use**: Reference for Agent Future Trajectory Prediction
- **HiVT URL**: https://github.com/ZikangZhou/HiVT
- **MTR URL**: https://github.com/sshaoshuai/MTR
- **Paper (MTR)**: "Motion Transformer with Global Intention Localization and Local Movement Refinement" (Shi et al., NeurIPS 2023)

---

## C.5 Benchmarks & Datasets

### nuScenes
- **Use**: Detection, Tracking, Prediction, Planning evaluation
- **URL**: https://www.nuscenes.org/
- **devkit URL**: https://github.com/nutonomy/nuscenes-devkit
- **License**: CC BY-NC-SA 4.0

### nuPlan
- **Use**: Closed-loop Planning evaluation
- **URL**: https://nuplan.org/
- **devkit URL**: https://github.com/motional/nuplan-devkit
- **License**: CC BY-NC-SA 4.0

### NAVSIM
- **Use**: Non-reactive simulation evaluation
- **URL**: https://github.com/autonomousvision/navsim
- **License**: Apache 2.0

### Bench2Drive
- **Use**: CARLA-based Closed-loop evaluation
- **URL**: https://github.com/Thinklab-SJTU/Bench2Drive
- **License**: MIT

### Waymo Open Dataset
- **URL**: https://waymo.com/open/
- **Features**: 1,950 segments, LiDAR + Camera, high-quality 3D bbox labels

### Argoverse 2
- **URL**: https://www.argoverse.org/av2.html
- **Features**: Strong for trajectory prediction and scene understanding

---

## C.6 World Models & Surveys

### World Model Survey
- **Paper**: "A Survey on World Models for Autonomous Driving" (2024)
- **URL**: https://arxiv.org/abs/2402.xxxxx (verify)

### TR-World / DreamDrive, etc.
- **Use**: Reference for Generative World Models
- Not directly adopted in this design, but promising for future extensions

---

## C.7 Simulators

### CARLA
- **Use**: Closed-loop Simulation (Phase 3)
- **URL**: https://github.com/carla-simulator/carla
- **License**: MIT (code) + proprietary license (assets)

### SUMO
- **Use**: Traffic flow simulation (can be integrated with CARLA)
- **URL**: https://sumo.dlr.de
- **License**: EPL 2.0

### nuPlan Reactive Sim
- **URL**: https://github.com/motional/nuplan-devkit
- **Use**: Log-based reactive simulation

---

## C.8 Lightweight & Optimization

### Fast-BEV
- **Use**: Real-time Camera BEV inference
- **URL**: https://github.com/Sense-GVT/Fast-BEV
- **Paper**: "Fast-BEV: A Fast and Strong Bird's-Eye View Perception Baseline" (2022)

### BEVDet
- **Use**: Camera-only BEV detection
- **URL**: https://github.com/HuangJunJie2017/BEVDet
- **License**: Apache 2.0

---

## C.9 VLM & Language Models

### LLaVA
- **Use**: Candidate for VLM Teacher (internal scene description generation)
- **URL**: https://github.com/haotian-liu/LLaVA
- **Paper**: "Visual Instruction Tuning" (Liu et al., NeurIPS 2023)
- **License**: Apache 2.0

### InternVL / Qwen-VL
- **Use**: Potential lightweight VLMs applicable to on-vehicle internal distillation
- **InternVL URL**: https://github.com/OpenGVLab/InternVL
- **Qwen-VL URL**: https://github.com/QwenLM/Qwen-VL

---

## C.10 Key Referenced Papers (Chronological)

| Year | Paper Title | Conference | Notes |
|---|---|---|---|
| 2022 | BEVFormer | ECCV | Camera BEV foundation |
| 2022 | BEVFusion (MIT) | NeurIPS | Camera+LiDAR fusion |
| 2023 | UniAD | CVPR Best Paper | Unified E2E planning |
| 2023 | SurroundOcc | ICCV | BEV Occupancy |
| 2023 | MTR | NeurIPS | Trajectory prediction |
| 2023 | SparseDrive | arXiv | Sparse BEV planning |
| 2024 | DriveLM | ECCV | Language+planning |
| 2024 | DiffusionDrive | NeurIPS | Diffusion-based planning |
| 2024 | NAVSIM | NeurIPS | New evaluation benchmark |
| 2024 | Bench2Drive | NeurIPS | Closed-loop evaluation |

---

## C.11 Notes on OSS Licenses

```text
Licenses requiring confirmation for commercial use:
  CC BY-NC-SA 4.0:
    - nuScenes, nuPlan
    - Non-commercial only. Contact Motional/nuScenes separately for commercial use.

  MIT / Apache 2.0:
    - BEVFusion (MIT), UniAD, DriveLM, SparseDrive, BridgeAD
    - Commercial use allowed (copyright notice required)

Commercial deployment of models trained on academic datasets:
  - The dataset license may affect the model
  - Consult with lawyers or your IP department
```
