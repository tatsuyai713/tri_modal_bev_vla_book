# 付録C 参考OSSリポジトリ・論文一覧

---

## C.1 BEV知覚・フュージョン

### BEVFormer
- **用途**: Camera BEV Encoder (本設計の中核)
- **URL**: https://github.com/fundamentalvision/BEVFormer
- **論文**: "BEVFormer: Learning Bird's-Eye-View Representation from Multi-Camera Images via Spatiotemporal Transformers" (Li et al., ECCV 2022)
- **ライセンス**: Apache 2.0
- **本設計での使い方**: Camera-to-BEV変換の主力バックボーン。warm start用の学習済み重みが公開されている。

### BEVFusion (MIT)
- **用途**: Camera + LiDAR Fusion (本設計の推奨統合手法)
- **URL**: https://github.com/mit-han-lab/bevfusion
- **論文**: "BEVFusion: A Simple and Robust LiDAR-Camera Fusion Framework" (Liang et al., NeurIPS 2022)
- **ライセンス**: MIT
- **本設計での使い方**: Camera BEVとLiDAR BEVの統合フュージョン。TensorRT最適化済み実装あり。

### BEVFusion (NuScenes)
- **用途**: BEVFusion の別実装（nuScenes向け）
- **URL**: https://github.com/ADLab-AutoDrive/BEVFusion
- **論文**: "BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation" (Liu et al., ICRA 2023)
- **ライセンス**: Apache 2.0

---

## C.2 占有・セグメンテーション

### FusionOcc / SurroundOcc
- **用途**: BEV Occupancy 予測 (本設計のbev_occupancy headの参考)
- **SurroundOcc URL**: https://github.com/weiyithu/SurroundOcc
- **論文**: "SurroundOcc: Multi-Camera 3D Occupancy Prediction for Autonomous Driving" (Wei et al., ICCV 2023)
- **ライセンス**: Apache 2.0

### OpenOccupancy
- **用途**: Occupancyの評価ベンチマーク
- **URL**: https://github.com/JeffWang987/OpenOccupancy

---

## C.3 エンドツーエンド計画

### UniAD
- **用途**: Unified Autonomous Driving framework (本設計の設計哲学の参考)
- **URL**: https://github.com/OpenDriveLab/UniAD
- **論文**: "Planning-oriented Autonomous Driving" (Hu et al., CVPR 2023 Best Paper)
- **ライセンス**: Apache 2.0
- **本設計での使い方**: BEV heads, agent予測, Plannerの統合設計の参考。warm start用重みが利用可能。

### DriveLM
- **用途**: Language-conditioned 計画 (本設計のVLA Plannerの参考)
- **URL**: https://github.com/OpenDriveLab/DriveLM
- **論文**: "DriveLM: Driving with Graph Visual Question Answering" (Sima et al., ECCV 2024)
- **ライセンス**: Apache 2.0
- **本設計での使い方**: T_scene の VLM 教師信号の生成手法。nuScenesのDriveLMデータが使える。

### DiffusionDrive
- **用途**: Diffusion-based 計画 (K-candidateの代替手法として参考)
- **URL**: https://github.com/hustvl/DiffusionDrive
- **論文**: "DiffusionDrive: Truncated Diffusion Model for End-to-End Autonomous Driving" (Liao et al., NeurIPS 2024)
- **ライセンス**: Apache 2.0

### SparseDrive
- **用途**: Sparse BEV 計画 (本設計の軽量化の参考)
- **URL**: https://github.com/swc-17/SparseDrive
- **論文**: "SparseDrive: End-to-End Autonomous Driving via Sparse Scene Representation" (Sun et al., 2024)
- **ライセンス**: Apache 2.0

### BridgeAD
- **用途**: 自然言語とVLA計画の統合
- **URL**: https://github.com/crossmodalgroup/bridgead
- **論文**: "BridgeAD: Bridging the Gap between End-to-End Autonomous Driving and Language" (2024)

---

## C.4 軌跡予測

### HiVT / MTR / Wayformer
- **用途**: Agent Future Trajectory Prediction の参考
- **HiVT URL**: https://github.com/ZikangZhou/HiVT
- **MTR URL**: https://github.com/sshaoshuai/MTR
- **論文（MTR）**: "Motion Transformer with Global Intention Localization and Local Movement Refinement" (Shi et al., NeurIPS 2023)

---

## C.5 ベンチマーク・データセット

### nuScenes
- **用途**: Detection, Tracking, Prediction, Planning 評価
- **URL**: https://www.nuscenes.org/
- **devkit URL**: https://github.com/nutonomy/nuscenes-devkit
- **ライセンス**: CC BY-NC-SA 4.0

### nuPlan
- **用途**: Closed-loop Planning 評価
- **URL**: https://nuplan.org/
- **devkit URL**: https://github.com/motional/nuplan-devkit
- **ライセンス**: CC BY-NC-SA 4.0

### NAVSIM
- **用途**: Non-reactive simulation 評価
- **URL**: https://github.com/autonomousvision/navsim
- **ライセンス**: Apache 2.0

### Bench2Drive
- **用途**: CARLA ベースの Closed-loop 評価
- **URL**: https://github.com/Thinklab-SJTU/Bench2Drive
- **ライセンス**: MIT

### Waymo Open Dataset
- **URL**: https://waymo.com/open/
- **特徴**: 1,950セグメント, LiDAR + Camera, 高品質3D bboxラベル

### Argoverse 2
- **URL**: https://www.argoverse.org/av2.html
- **特徴**: 軌跡予測とシーン理解に強い

---

## C.6 世界モデル・サーベイ

### World Model Survey
- **論文**: "A Survey on World Models for Autonomous Driving" (2024)
- **URL**: https://arxiv.org/abs/2402.xxxxx (要確認)

### TR-World / DreamDrive 等
- **用途**: Generative World Modelの参考
- 本設計では直接採用していないが、将来の拡張として有望

---

## C.7 シミュレータ

### CARLA
- **用途**: Closed-loop Simulation (Phase 3)
- **URL**: https://github.com/carla-simulator/carla
- **ライセンス**: MIT (コード) + 独自ライセンス (アセット)

### SUMO
- **用途**: 交通流シミュレーション (CARLAと連携可能)
- **URL**: https://sumo.dlr.de
- **ライセンス**: EPL 2.0

### nuPlan Reactive Sim
- **URL**: https://github.com/motional/nuplan-devkit
- **用途**: ログベースのリアクティブシミュレーション

---

## C.8 軽量化・最適化

### Fast-BEV
- **用途**: リアルタイム Camera BEV 推論
- **URL**: https://github.com/Sense-GVT/Fast-BEV
- **論文**: "Fast-BEV: A Fast and Strong Bird's-Eye View Perception Baseline" (2022)

### BEVDet
- **用途**: Camera-only BEV 検出
- **URL**: https://github.com/HuangJunJie2017/BEVDet
- **ライセンス**: Apache 2.0

---

## C.9 VLM・言語モデル関連

### LLaVA
- **用途**: VLM Teacher（内部シーン説明生成）の候補
- **URL**: https://github.com/haotian-liu/LLaVA
- **論文**: "Visual Instruction Tuning" (Liu et al., NeurIPS 2023)
- **ライセンス**: Apache 2.0

### InternVL / Qwen-VL
- **用途**: 軽量VLMとして車上での内部蒸留に適用可能性あり
- **InternVL URL**: https://github.com/OpenGVLab/InternVL
- **Qwen-VL URL**: https://github.com/QwenLM/Qwen-VL

---

## C.10 参考にした主要論文（時系列）

| 年 | 論文タイトル | 会議 | 備考 |
|---|---|---|---|
| 2022 | BEVFormer | ECCV | Camera BEV基盤 |
| 2022 | BEVFusion (MIT) | NeurIPS | Camera+LiDAR融合 |
| 2023 | UniAD | CVPR Best Paper | 統合型E2E計画 |
| 2023 | SurroundOcc | ICCV | BEV Occupancy |
| 2023 | MTR | NeurIPS | 軌跡予測 |
| 2023 | SparseDrive | arXiv | 疎なBEV計画 |
| 2024 | DriveLM | ECCV | Language+計画 |
| 2024 | DiffusionDrive | NeurIPS | Diffusion計画 |
| 2024 | NAVSIM | NeurIPS | 新評価ベンチマーク |
| 2024 | Bench2Drive | NeurIPS | Closed-loop評価 |

---

## C.11 OSSライセンスの注意点

```text
商用利用時の確認が必要なライセンス:
  CC BY-NC-SA 4.0:
    - nuScenes, nuPlan
    - 非商用のみ。商用利用は別途Motional/nuscenesに連絡が必要

  MIT / Apache 2.0:
    - BEVFusion (MIT), UniAD, DriveLM, SparseDrive, BridgeAD
    - 商用利用可（著作権表示が必要）

学術データセットを使って学習したモデルの商用展開:
  - データセットのライセンスがモデルに影響する場合がある
  - 弁護士や知財部門への確認を推奨
```
