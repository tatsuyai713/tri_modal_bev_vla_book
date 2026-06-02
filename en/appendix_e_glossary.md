# Appendix E Glossary and Abbreviations

---

## E.1 Architecture-Related

| Abbreviation / Term | Full Name | Description |
|---|---|---|
| BEV | Bird's-Eye View | 2D map representation from a top-down perspective |
| VLA | Vision-Language-Action | Planning model integrating vision, language, and action |
| Tri-Modal | - | Fusion of three modalities: Camera + LiDAR + Radar |
| E2E | End-to-End | Approach that learns end-to-end from sensor input to control output |
| Structured E2E | - | Positioning of this design. E2E with explicit intermediate representations (BEV) |
| CondFormer | Conditioning Transformer | Transformer module that applies text conditioning to BEV tokens |
| MHP Loss | Multi-Head Prediction Loss | Loss that flows gradients only to the best candidate among K candidates |
| QAT | Quantization-Aware Training | Training method that simulates quantization during training |
| PTQ | Post-Training Quantization | Method to quantize an already-trained model |
| KD | Knowledge Distillation | Method to transfer knowledge from a large model to a smaller model |

---

## E.2 Sensor & Perception-Related

| Abbreviation / Term | Full Name | Description |
|---|---|---|
| LiDAR | Light Detection And Ranging | 3D distance sensor using laser light |
| Radar | Radio Detection And Ranging | Speed and distance sensor using radio waves |
| IMU | Inertial Measurement Unit | Sensor that measures acceleration and angular velocity |
| RTK-GNSS | Real-Time Kinematic GNSS | High-precision GPS positioning with centimeter-level accuracy |
| Pillar | - | BEV feature representation that compresses LiDAR point clouds along the Z axis |
| Voxel | Volume element | One cell in a 3D grid |
| ROI | Region of Interest | The region to focus attention on |
| FPN | Feature Pyramid Network | Multi-scale feature extraction network |
| BEV Encoder | - | Network that converts sensor data into BEV space |
| Modality Gate | - | Mechanism that adjusts fusion weights according to the reliability of each sensor |
| Occ Flow | Occupancy Flow | Temporal change (velocity field) of the occupancy grid |
| DRIVABLE | - | bev_drivable class 1. Normal driving area (roadway, driving lane) |
| NOT_DRIVABLE | - | bev_drivable class 0. Area that is physically impassable (walls, large height changes, etc.) |
| MARGINAL | - | bev_drivable class 2. Area that is physically passable but should not normally be driven on (shoulder, sidewalk beyond a curb, etc.). Equivalent to Tesla FSD's same concept. |

---

## E.3 Planning & Control-Related

| Abbreviation / Term | Full Name | Description |
|---|---|---|
| Planner | - | Network module that generates trajectories |
| Converter | - | Deterministic module that converts trajectories to steering angle |
| Evaluator | - | External module that evaluates the safety of K candidate trajectories and selects one |
| K candidates | K candidate trajectories | K trajectories generated in parallel by the Planner |
| K-query VLA Planner | - | Transformer Decoder-based trajectory generation Planner with K learnable queries. MVP implementation trained with MHP Loss |
| DiffusionDrive | - | Trajectory generation via conditional diffusion model (NVIDIA, NeurIPS 2024). Adopted as the evolution of K-query Planner |
| Fallback | - | Safe alternative action used when normal planning fails |
| TTC | Time-to-Collision | Estimated time until collision |
| THW | Time Headway | Time gap to the vehicle ahead |
| lookahead | Lookahead | The look-ahead point that the Converter focuses on in the trajectory |
| Pure Pursuit | - | Control algorithm that computes curvature toward the lookahead point |
| ADE | Average Displacement Error | Average positional prediction error over all timesteps |
| FDE | Final Displacement Error | Positional prediction error at the final timestep |
| MR | Miss Rate | Fraction of samples where all candidates deviate beyond a threshold |
| minADE@K | - | ADE of the best candidate among K candidates |

---

## E.4 Safety & Regulation-Related

| Abbreviation / Term | Full Name | Description |
|---|---|---|
| ODD | Operational Design Domain | The operational domain in which the autonomous driving system guarantees operation |
| ISO 26262 | - | International standard for automotive functional safety |
| SOTIF | Safety Of The Intended Functionality | ISO 21448. Safety of the intended functionality |
| UNECE R157 | - | UN regulation on advanced driver assistance systems |
| ASIL | Automotive Safety Integrity Level | Safety level in ISO 26262 (A through D) |
| SIL | Safety Integrity Level | Safety level in IEC 61508 |
| HARA | Hazard Analysis and Risk Assessment | Hazard analysis and risk assessment |
| FMEA | Failure Mode and Effects Analysis | Failure mode and effects analysis |
| FTA | Fault Tree Analysis | Method of analyzing causes of undesired top events in a tree structure |
| Safety Case | - | Collection of arguments and evidence showing that a system is safe |

---

## E.5 Training & Evaluation-Related

| Abbreviation / Term | Full Name | Description |
|---|---|---|
| nuScenes | - | Autonomous driving dataset published by Nuro/Motional |
| nuPlan | - | Dataset and simulator for planning evaluation |
| NAVSIM | Non-Reactive Autonomous Vehicle SIMulation | Non-reactive simulation evaluation |
| NDS | nuScenes Detection Score | 3D object detection evaluation metric for nuScenes |
| mAP | mean Average Precision | Mean average precision for object detection |
| IoU | Intersection over Union | Degree of overlap between prediction and ground truth |
| mIoU | mean IoU | IoU averaged over all classes |
| Open-loop | - | Evaluation that replays recorded data. The ego vehicle's actions do not affect the environment |
| Closed-loop | - | Simulation evaluation where other vehicles react to the system's actions |
| Shadow Mode | - | Collection and evaluation phase where the system runs in a real vehicle but a human maintains control |
| OTA | Over-The-Air | Updates via wireless network |
| MVP | Minimum Viable Product | Minimum viable product. The first working version with the minimum necessary functions. In this book, refers to the first model configuration that can pass Shadow Mode validation. |

---

## E.6 Network Architecture-Related

| Abbreviation / Use | Full Name | Description |
|---|---|---|
| Transformer | - | Sequence model using Self/Cross Attention |
| Cross-Attention | - | Attention between queries and a separate set of keys and values |
| Deformable Attention | - | Attention that learns sampling points (used in BEVFormer, etc.) |
| QFormer | Query Transformer | Transformer that compresses VLM features with a fixed number of queries |
| FP16 | Float16 | 16-bit floating point number. Used to improve inference speed. |
| INT8 | Integer 8-bit | 8-bit integer. Quantization format for fast inference. |
| TensorRT | - | NVIDIA's GPU inference optimization library |
| ONNX | Open Neural Network Exchange | Intermediate format for model portability |

---

## E.7 Coordinate Systems and Units

| Term | Definition |
|---|---|
| Ego coordinate system | Vehicle-centered coordinate system. x=forward, y=left, z=up (right-hand system) |
| BEV grid | BEV space in the ego coordinate system. Unit is meters. |
| Steering angle | In radians. +left turn, -right turn |
| Acceleration | m/s². +forward acceleration, -deceleration (braking) |
| Timestamp | In nanoseconds |
| LiDAR point cloud | x,y,z [m] + intensity [0-1]. In ego coordinate system. |
| Radar point cloud | x,y,z [m] + radial_velocity [m/s] + intensity. In ego coordinate system. |

---

## E.8 Terms Uniquely Defined in This Book

| Term | Definition | First Appeared |
|---|---|---|
| Structured E2E | E2E architecture that explicitly retains intermediate representations (BEV) | Chapter 1 |
| T_ext | External text instruction token (route/command) | Chapter 5 |
| T_scene | Internal scene description token (VLM distillation) | Chapter 5 |
| Modality Gate | Fusion weight adapter based on sensor health | Chapter 3 |
| Human Trajectory Teacher | Method of using human driver trajectories as training teacher signals | Chapter 6 |
| Lane-Center-Free | Design that directly uses human trajectories as teacher signals without following lane centers | Chapter 6 |
| Trajectory-to-Steering Converter | Deterministic module that converts trajectories to steering angle | Chapter 7 |
| Dynamic Risk Map | Map projecting future dynamic occupancy onto BEV with a time axis | Chapter 4 |
| External Evaluator | Safety evaluator separated from the neural network | Chapter 2 |
| Fallback Trajectory | Rule-based trajectory used when all candidates are unsafe | Chapter 2 |
| Motion Salience Gate | Mechanism that controls Planner attention according to the importance of dynamic scenes | Chapter 4 |
| Shadow ADE | Trajectory error vs. actual driving in Shadow Mode | Chapter 11, Chapter 13 |
| curvature_profile | List of curvatures corresponding to each point of a LaneNode's center_line. Used for forward/backward speed limits and the Evaluator's comfort check. | Chapter 3, Chapter 5, Appendix A |
| max_curvature | Maximum absolute value of curvature [1/m] within a LaneNode. Used in computing curvature speed v = sqrt(a_lat_max/kappa). | Chapter 3, Appendix B |
| advisory_speed_mps | Curvature-based recommended speed. sqrt(a_lat_max / max_curvature) and no greater than speed_limit. | Chapter 3, Chapter 5, Appendix A |
