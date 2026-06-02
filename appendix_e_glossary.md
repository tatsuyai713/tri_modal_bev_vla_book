# 付録E 用語集・略語一覧

---

## E.1 アーキテクチャ関連

| 略語 / 用語 | 正式名称 | 説明 |
|---|---|---|
| BEV | Bird's-Eye View | 俯瞰視点の2次元マップ表現 |
| VLA | Vision-Language-Action | 視覚・言語・行動を統合した計画モデル |
| Tri-Modal | - | Camera + LiDAR + Radar の3モダリティ融合 |
| E2E | End-to-End | センサ入力から制御出力までを一貫して学習する手法 |
| Structured E2E | - | 本設計の位置付け。中間表現（BEV）を明示的に持つE2E |
| CondFormer | Conditioning Transformer | テキスト条件をBEVトークンに適用するTransformerモジュール |
| MHP Loss | Multi-Head Prediction Loss | K候補のうち最良候補にのみ勾配を流す損失 |
| QAT | Quantization-Aware Training | 量子化をシミュレートしながら学習する手法 |
| PTQ | Post-Training Quantization | 学習済みモデルを量子化する手法 |
| KD | Knowledge Distillation | 大きなモデルの知識を小さなモデルに転移する手法 |

---

## E.2 センサ・知覚関連

| 略語 / 用語 | 正式名称 | 説明 |
|---|---|---|
| LiDAR | Light Detection And Ranging | レーザー光を使った3D距離センサ |
| Radar | Radio Detection And Ranging | 電波を使った速度・距離センサ |
| IMU | Inertial Measurement Unit | 加速度・角速度を計測するセンサ |
| RTK-GNSS | Real-Time Kinematic GNSS | cm精度の高精度GPS測位 |
| Pillar | - | LiDAR点群をZ方向に圧縮したBEV特徴表現 |
| Voxel | Volume element | 3次元グリッドの1セル |
| ROI | Region of Interest | 注意を向ける領域 |
| FPN | Feature Pyramid Network | マルチスケール特徴抽出ネットワーク |
| BEV Encoder | - | センサデータをBEV空間に変換するネットワーク |
| Modality Gate | - | 各センサの信頼度に応じて融合重みを調整する機構 |
| Occ Flow | Occupancy Flow | 占有グリッドの時間変化（速度場） |
| DRIVABLE | - | bev_drivable クラス1。通常走行域（車道・走行レーン） |
| NOT_DRIVABLE | - | bev_drivable クラス0。物理的に進入不可能な領域（壁・高い段差等） |
| MARGINAL | - | bev_drivable クラス2。物理的には進入可能だが通常は走行すべきでない領域（路肩・段差先の歩道等）。Tesla FSD の同概念に相当 |

---

## E.3 計画・制御関連

| 略語 / 用語 | 正式名称 | 説明 |
|---|---|---|
| Planner | - | 軌跡を生成するネットワークモジュール |
| Converter | - | 軌跡をステアリング角に変換する決定論的モジュール |
| Evaluator | - | K候補軌跡の安全性を評価して選択する外部モジュール |
| K candidates | K候補軌跡 | Plannerが並列生成するK本の軌跡 |
| K-query VLA Planner | - | K個の学習可能クエリを持つTransformer Decoder型の軌跡生成Planner。MHP Lossで学習するMVP実装 |
| DiffusionDrive | - | 条件付き拡散モデルによる軌跡生成手法（NVIDIA, NeurIPS 2024）。K-query Plannerの発展形として採用 |
| Fallback | - | 通常計画が失敗した際の安全な代替行動 |
| TTC | Time-to-Collision | 衝突までの推定時間 |
| THW | Time Headway | 前走車との時間距離 |
| lookahead | ルックアヘッド | Converterが軌跡上で注目する先読み点 |
| Pure Pursuit | - | ルックアヘッド点への曲率を計算する制御アルゴリズム |
| ADE | Average Displacement Error | 全時刻の予測位置誤差の平均 |
| FDE | Final Displacement Error | 最終時刻の予測位置誤差 |
| MR | Miss Rate | 全候補が閾値以上外れたサンプルの割合 |
| minADE@K | - | K候補のうち最良候補のADE |

---

## E.4 安全・規制関連

| 略語 / 用語 | 正式名称 | 説明 |
|---|---|---|
| ODD | Operational Design Domain | 自動運転システムが動作を保証する運用領域 |
| ISO 26262 | - | 自動車の機能安全国際規格 |
| SOTIF | Safety Of The Intended Functionality | ISO 21448。意図された機能の安全性 |
| UNECE R157 | - | 高度な運転支援システムに関するUN規制 |
| ASIL | Automotive Safety Integrity Level | ISO 26262 の安全水準（A〜D） |
| SIL | Safety Integrity Level | IEC 61508 の安全水準 |
| HARA | Hazard Analysis and Risk Assessment | ハザード分析とリスク評価 |
| FMEA | Failure Mode and Effects Analysis | 故障モードと影響分析 |
| FTA | Fault Tree Analysis | 望ましくない上位事象から原因を木構造で分析する手法 |
| Safety Case | - | システムが安全であることを示す論拠・証拠の集合 |

---

## E.5 学習・評価関連

| 略語 / 用語 | 正式名称 | 説明 |
|---|---|---|
| nuScenes | - | Nuro/Motionalが公開した自動運転データセット |
| nuPlan | - | 計画評価用データセット・シミュレータ |
| NAVSIM | Non-Reactive Autonomous Vehicle SIMulation | 非リアクティブシミュレーション評価 |
| NDS | nuScenes Detection Score | nuScenesの3D物体検出評価指標 |
| mAP | mean Average Precision | 物体検出の平均精度 |
| IoU | Intersection over Union | 予測と正解の重なり度合い |
| mIoU | mean IoU | 全クラス平均のIoU |
| Open-loop | - | 記録データを再生する評価。自車の行動が環境に影響しない |
| Closed-loop | - | 他車両がシステムの行動に反応するシミュレーション評価 |
| Shadow Mode | - | 実車でシステムを動かすが制御は人間が行う収集・評価フェーズ |
| OTA | Over-The-Air | 無線ネットワーク経由の更新 |
| MVP | Minimum Viable Product | 最小実用製品。必要最低限の機能を備えた最初の動作可能バージョン。本書では Shadow Mode 検証を通過できる最初のモデル構成を指す |

---

## E.6 ネットワークアーキテクチャ関連

| 略語 / 用途 | 正式名称 | 説明 |
|---|---|---|
| Transformer | - | Self/Cross Attentionを使ったシーケンスモデル |
| Cross-Attention | - | クエリと別のキー・バリューセット間のAttention |
| Deformable Attention | - | サンプリング点を学習するAttention（BEVFormer等） |
| QFormer | Query Transformer | 固定数のクエリでVLM特徴を圧縮するTransformer |
| FP16 | Float16 | 16ビット浮動小数点数。推論速度向上に使う |
| INT8 | Integer 8-bit | 8ビット整数。高速推論のための量子化形式 |
| TensorRT | - | NVIDIAのGPU推論最適化ライブラリ |
| ONNX | Open Neural Network Exchange | モデルの可搬性のための中間フォーマット |

---

## E.7 座標系と単位

| 用語 | 定義 |
|---|---|
| Ego座標系 | 自車中心の座標系。x=前方、y=左方、z=上方（右手系） |
| BEVグリッド | Ego座標系でのBEV空間。単位はメートル |
| ステアリング角 | ラジアン単位。+左旋回、-右旋回 |
| 加速度 | m/s²。+前向き加速、-減速（ブレーキ） |
| Timestamp | nanosecond（ナノ秒）単位 |
| LiDAR点群 | x,y,z [m] + intensity [0-1]。Ego座標系 |
| Radar点群 | x,y,z [m] + radial_velocity [m/s] + intensity。Ego座標系 |

---

## E.8 本書で独自定義した用語

| 用語 | 定義 | 初出章 |
|---|---|---|
| Structured E2E | 中間表現（BEV）を明示的に保持するE2Eアーキテクチャ | 第1章 |
| T_ext | 外部テキスト指示トークン（経路・コマンド） | 第5章 |
| T_scene | 内部シーン記述トークン（VLM蒸留） | 第5章 |
| Modality Gate | センサ健全性に基づく融合重みアダプタ | 第3章 |
| Human Trajectory Teacher | 人間ドライバーの走行軌跡を学習の教師に使う手法 | 第6章 |
| Lane-Center-Free | 車線中心を追跡せず人間軌跡を直接教師にする設計 | 第6章 |
| Trajectory-to-Steering Converter | 軌跡をステアリング角に変換する決定論的モジュール | 第7章 |
| Dynamic Risk Map | 動体の将来占有をBEVに時間軸付きで投影したマップ | 第4章 |
| External Evaluator | ニューラルネットワーク外に分離された安全評価器 | 第2章 |
| Fallback Trajectory | 全候補が不安全な場合に使うルールベース軌跡 | 第2章 |
| Motion Salience Gate | 動的シーンの重要度に応じてPlannerの注意を制御する機構 | 第4章 |
| Shadow ADE | Shadow Modeにおける実走行との軌跡誤差 | 第11章・第13章 |
| curvature_profile | LaneNode の center_line 各点に対応する曲率リスト。前後方向の速度制限およびEvaluatorの快適性チェックに利用 | 第3章・第5章・附録A |
| max_curvature | LaneNode 内の曲率絶対値の最大値 [1/m]。曲率速度 v = √(a_lat_max/κ) の計算に使用 | 第3章・附録B |
| advisory_speed_mps | 曲率ベースの推奨速度。sqrt(a_lat_max / max_curvature) かつ speed_limit 以下 | 第3章・第5章・附録A |
