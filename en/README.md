# Tri-Modal BEV VLA Planner Design Book

## Structured End-to-End Architecture for Real-Vehicle Autonomous Driving

**Version: 20.0**  
**Date: 2026-05-31**  
**Languages: Japanese primary, English edition in `en/`**

---

## About This Book

This design book describes an autonomous driving architecture that fuses Vision, LiDAR, and Radar into a shared BEV (Bird's Eye View) representation, conditions planning with external language commands and internal scene tokens, outputs multiple candidate trajectories, and selects a safe trajectory through an external evaluator.

The central idea is **Structured End-to-End**: keep the learning flexibility of modern E2E systems while preserving explicit intermediate representations such as Static World, Dynamic World, Lane Topology, Agent Futures, Dynamic Risk Maps, candidate trajectories, and safety logs.

---

## Language Editions

| Edition | Path | Description |
|---|---|---|
| Japanese / 日本語版 | `tri_modal_bev_vla_book/` | Full source edition and primary working documents |
| English / 英語版 | `tri_modal_bev_vla_book/en/` | English edition with the same chapter/file structure |

---

## Chapter Structure

| No. | File | English title | Japanese title |
|---|---|---|---|
| 0 | `00_preface.md` | Preface | まえがき |
| 1 | `01_design_philosophy.md` | Design Philosophy: From ADAS to Structured End-to-End | 設計思想: ADASからStructured End-to-Endへ |
| 2 | `02_system_requirements.md` | System Requirements and Overall Architecture | システム要件と全体アーキテクチャ |
| 3 | `03_tri_modal_bev.md` | Tri-modal BEV World Model | Tri-modal BEV World Model |
| 4 | `04_dynamic_world.md` | Dynamic Object World Model and Future Risk Estimation | Dynamic Object World Modelと未来リスク推定 |
| 5 | `05_language_vla_planner.md` | Language Conditioning and VLA Planner | Language ConditioningとVLA Planner |
| 6 | `06_human_trajectory.md` | Human Trajectory Teacher and Lane-Center-Free Planning | 人間軌跡教師とレーンセンター非依存Planning |
| 7 | `07_steering_conversion.md` | Converting Selected Trajectories to Target Steering | 選択軌跡から目標舵角への変換 |
| 8 | `08_realtime_design.md` | Lightweight and Real-Time Design for Vehicle Deployment | 実車適用のための軽量化とリアルタイム設計 |
| 9 | `09_product_safety.md` | Productization, Safety, Regulation, ODD, and Logging | 製品化・安全性・法規・ODD・ログ |
| 10 | `10_implementation.md` | Implementation Modules and Interface Design | 実装モジュールとインターフェース設計 |
| 11 | `11_roadmap_shadow.md` | Implementation Roadmap and Shadow Mode Validation | 実装ロードマップとShadow Mode検証 |
| 12 | `12_data_collection.md` | Data Collection, Fleet Learning, and Annotation Pipeline | データ収集・フリート学習・アノテーションパイプライン |
| 13 | `13_evaluation_metrics.md` | Evaluation Metrics, Benchmarks, and Scenario Design | 評価指標・ベンチマーク・シナリオ設計 |
| 14 | `14_hardware_platform.md` | Hardware Platforms, Quantization, and OTA Deployment | ハードウェアプラットフォーム・量子化・OTA展開 |
| A | `appendix_a_pseudocode.md` | Forward-Pass Pseudocode | Forward疑似コード |
| B | `appendix_b_output_format.md` | Output Format Specification | 出力フォーマット定義 |
| C | `appendix_c_references.md` | OSS Repositories and Papers | 参考OSS・論文一覧 |
| D | `appendix_d_training_strategy.md` | Training Strategy and OSS Weight Reuse | 学習戦略とOSS公開重みの活用 |
| E | `appendix_e_glossary.md` | Glossary and Abbreviations | 用語集・略語一覧 |

---

## Core Design Principles

| Area | Description |
|---|---|
| Sensor fusion | Fuse Vision, LiDAR, and Radar in a shared BEV space |
| World model | Separate Static World, Dynamic World, Lane Topology, and Agent Futures |
| Dynamic world | Keep detection/state estimation separate from multi-modal future prediction |
| Unknown objects | Use `unknown` class plus `unknown_dynamic` flag for conservative handling |
| Planner | Generate K candidate trajectories and select through an external evaluator |
| Control | Convert selected trajectory to steering deterministically |
| Teacher signal | Learn from human driving trajectories, not only lane centerlines |
| Productization | Keep explicit intermediate outputs, ODD monitoring, safety logs, and rollback gates |

---

## Latest Updates

| Version | Date | Changes |
|---|---|---|
| v19 | 2026-05-30 | Split the design book into chapter-level Markdown files. Added Chapters 12-14 and expanded implementation, data, evaluation, and hardware content. |
| v20 | 2026-05-31 | Consolidated Chapter 4 Sections 4.3 and 4.6 responsibilities, updated unknown-object handling, fixed Dynamic Risk Map descriptions, refreshed README as bilingual, and added the English edition folder. |

---

## Target Readers

- Autonomous driving system architects and research engineers
- Developers working on productization of in-vehicle AI/ML
- Engineers integrating perception, prediction, planning, and control
- System architects integrating recognition, prediction, planning, and control
- Safety, regulation, ODD, validation, and logging engineers
