# 付録D 学習戦略とOSS重みの活用

---

## D.1 学習の全体方針

Tri-Modal BEV VLA Plannerはモジュール構成が複雑なため、スクラッチから全モジュールを同時に学習することはほぼ不可能である。以下の原則に従う。

```text
原則1: Warm Start First（ウォームスタートを最初に行う）
  - 既存の公開済みOSS重みから出発する
  - スクラッチからの学習はLiDAR/Camera headに限定する

原則2: ステージ分離学習（Stage-wise Training）
  - モジュールごとにステージを分けて学習する
  - 前ステージの重みを凍結し、次ステージのみ更新する
  - 合同Fine-tuning（Joint Fine-tuning）は最後に行う

原則3: データ品質 > データ量
  - 低品質データで大量学習するより、高品質データで少量学習する方が良い
  - フィルタリングパイプライン（第12章）を徹底する

原則4: 損失関数を慎重に設計する
  - 複数の損失が競合する場合、重みバランスのチューニングに時間を使う
  - 損失が0に張り付いていないか常に監視する

原則5: 学習曲線を観察してから次のステージに進む
  - val loss が改善しなくなってから次へ
  - オーバーフィットが始まったら early stopping
```

---

## D.2 7ステージ学習計画

| ステージ | 対象モジュール | データ | 凍結範囲 |
|---|---|---|---|
| Stage 1 | BEV Backbone (Camera + LiDAR) | nuScenes Detection | なし（新規学習） |
| Stage 2 | BEV Information Heads | nuScenes + 自社データ | Backbone を凍結 |
| Stage 3 | K-query Planner | 人間軌跡データ | Stage 1+2 を凍結 |
| Stage 4 | External Language Conditioning | 言語+軌跡データ | Stage 1+2 を凍結、CondFormer のみ更新 |
| Stage 5 | Internal Scene Tokenizer | VLM教師データ | Stage 1+2+3 を凍結 |
| Stage 6 | Closed-loop / Offline RL Speed Policy Fine-tuning | シミュレータ + 厳選実車ログ | World/Perceptionを凍結、Planner速度分岐のみ更新 |
| Stage 7 | Joint Fine-tuning | 全データ混合 | 全モジュール学習可（低LR） |

---

## D.3 Stage 1: BEV Backbone の学習

```text
目的:
  - BEVFormer / BEVFusion の重みをwarm startとして使う
  - 自社センサ構成（カメラ台数・レンズ・LiDAR種別）に合わせて調整

Warm Start 元:
  BEVFusion (MIT版):
    - Camera + LiDAR の統合BEV Encoder
    - nuScenes で学習済み
    - ダウンロード: https://github.com/mit-han-lab/bevfusion (READMEのcheckpoint)

  BEVFormer (v2):
    - Camera-only BEV Encoder
    - nuScenes で学習済み
    - ダウンロード: https://github.com/fundamentalvision/BEVFormer

Warm Start後の調整:
  - カメラ台数の変更（6→8台等）: Camera BEV Encoderのinput layerを再初期化
  - LiDAR種別の変更（velodyne→ouster等）: Pillar Encoderのvoxel sizeを調整
  - BEVグリッド範囲の変更: positional encodingを再学習

学習設定:
  Optimizer: AdamW (lr=1e-4, weight_decay=1e-2)
  Scheduler: CosineAnnealingLR
  Batch size: 4〜8 (GPU memoryに依存)
  Epochs: 24 (nuScenes標準)
  Augmentation:
    - Camera: random brightness/contrast, horizontal flip (nuScenes左右反転)
    - LiDAR: random rotation, random scale
```

---

## D.4 Stage 2: BEV Information Heads の学習

```text
前提: Stage 1の重みをロード、Backboneを凍結

対象:
  - drivable_head
  - static_occupancy_head
  - occ_flow_head
  - lane_center_head
  - stopline_head
  - speed_limit_head
  - dynamic_occ_head
  - uncertainty_head

損失関数:
  L_stage2 = 
    λ_driv * BCE(pred_drivable, gt_drivable) +
    λ_occ  * BCE(pred_occ, gt_occ) +
    λ_flow * L1(pred_flow, gt_flow) +
    λ_lane * BCE(pred_lane, gt_lane) +
    λ_stop * BCE(pred_stop, gt_stop) +
    λ_spd  * CE(pred_speed_limit, gt_speed_limit) +
    λ_dyn  * BCE(pred_dyn_occ, gt_dyn_occ)

重みの目安:
  λ_driv = 1.0
  λ_occ  = 2.0  (不均衡対策でやや大きめ)
  λ_flow = 0.5
  λ_lane = 1.0
  λ_stop = 2.0  (稀なクラスなので大きめ)
  λ_spd  = 1.0
  λ_dyn  = 2.0

教師信号の自動生成:
  gt_drivable: HD Map + LiDAR段差検出から3クラスラベルをBEVに投影
    - DRIVABLE(1): HD Map 車道領域
    - NOT_DRIVABLE(0): HD Map 路外 + LiDAR 段差 >15cm
    - MARGINAL(2): HD Map 路肩・歩道 + 段差 5〜15cm のグレーゾーン
  gt_occ: LiDAR点群からの静的占有（Ground除去済み）
  gt_flow: 動体のLiDAR追跡軌跡から計算
  gt_lane: HD Map の車線中心をBEVに投影
  gt_stop: HD Map の停止線をBEVに投影
  gt_speed_limit: HD Map の規制速度
  gt_dyn_occ: 動体の3D bbox をBEVに投影

学習設定:
  Optimizer: AdamW (lr=3e-4)
  Epochs: 12
  データ: nuScenes + 自社データ混合（自社データ比率50%）
```

---

## D.5 Stage 3: K-query Planner の学習

```text
前提: Stage 1+2 の重みをロード、凍結

対象:
  - CondFormer（言語なしのバージョン）
  - K-query VLA Planner Decoder

入力:
  - BEV tokens (Stage 1+2 出力、勾配なし)
  - Lane topology tokens
  - planner_queries (K個、学習可能)

出力:
  - (K, T, 3) 軌跡候補

損失関数:
  L_planner = L_traj + L_speed + L_comfort + L_safety_reg

  L_traj（軌跡ターゲット損失）:
    - MHP Loss (Multi-Head Prediction Loss)
    - 最も人間軌跡に近い候補のみで勾配を流す（Winner-takes-all）
    L_traj = min_k L2(traj_k, traj_human)

  L_speed（速度プロファイル損失）:
    L_speed = L1(v_pred, v_human)
    - 人間の速度プロファイルとのL1差

  L_comfort（快適性正則化）:
    L_comfort = λ * (mean(|a_x|) + mean(|a_y|))
    - 過度な加速・ハンドル操作を抑制

  L_safety_reg（安全正則化）:
    L_safety_reg = mean(max(0, margin - dist_to_occ))
    - 静的占有までの距離がマージン以下のペナルティ

学習設定:
  Optimizer: AdamW (lr=1e-3 for queries, 3e-4 for others)
  Epochs: 24
  K = 8
  T = 10 (5秒, 0.5秒間隔)

発展オプション: DiffusionDrive への移行（§6.10 参照）
  K-query Planner の安定稼働を確認した後、以下の場合に DiffusionDrive を検討する。
  - K 固定候補では稀少シナリオのカバレッジが不足する
  - 停止プロファイル・IDM・先読みカーブ速度の精度をさらに向上させたい
  - DiffusionDrive Phase A/B/C の学習手順は §6.10 を参照のこと
```

---

## D.6 Stage 4: 外部言語条件付け（CondFormer）の学習

```text
前提: Stage 1+2 の重みを凍結、Stage 3 の Planner 重みも読み込む

対象:
  - Text Encoder
  - CondFormer（言語条件付き部分）
  - Instruction Priority Gate

データ:
  - 走行ログ + DSLラベル（右折・左折・車線変更・停止・直進）
  - 各走行セグメントに対してDSLアノテーションを付与

学習:
  - DSLを入力し、その意図に合った軌跡を出力するよう学習
  - 既存Planner（Stage 3）の選択軌跡との一致度で損失

  L_lang = L2(traj_lang_conditioned, traj_stage3_best)

注意:
  - 言語入力なし（デフォルト指示）でも軌跡品質が落ちないことを確認する
  - 言語入力ありの場合と比較してメトリクスが改善していることを確認する
```

---

## D.7 Stage 5: 内部シーントークナイザの学習

```text
対象:
  - Scene Tokenizer
  - VLM教師との蒸留（Knowledge Distillation）

方法:
  - VLM（LLaVA等）がカメラ画像から生成したテキスト説明 T_scene_teacher を教師にする
  - Scene Tokenizer の出力テキストが T_scene_teacher に近づくよう学習

損失:
  L_scene = Cross_Entropy(T_scene_pred, T_scene_teacher)

または、テキスト変換なしの特徴蒸留:
  L_scene = MSE(scene_token_pred, VLM_encoder(T_scene_teacher))

データ:
  - カメラ画像 + VLM生成テキスト（12章の自動アノテーション参照）
  - 規模: 10万シーン以上を推奨

注意:
  - Stage 5 はオプション。MVPでは省略し、後で追加する
  - VLM教師の品質が低いとノイズになる
```

---

## D.8 Stage 6: Closed-loop / Offline RL Speed Policy Fine-tuning

```text
目的:
  人間模倣だけでは獲得しづらい「予見的減速→一定旋回→漸次再加速」を、
  閉ループシミュレーションとoffline/constrained RLで獲得する。

位置づけ:
  最新研究の主流は、実車online RLではなく、
  大規模ILを基盤にして閉ループ評価・offline RL・制約付き目的関数を組み合わせる方式である。

更新対象:
  - Plannerの速度分岐（v/a_x/phase head）
  - 必要に応じてCondFormerの速度制約attentionのみ

固定対象:
  - BEV encoder / world heads / evaluator

報酬設計例:
  r = w_progress * progress
    - w_jerk * |jerk|
    - w_lat * ReLU(v^2*κ - a_y_max)
    - w_yaw * ReLU(|v*yaw_rate| - a_y_max)
    - w_stop * |stop_position_error|
    - w_gap * |actual_stop_gap - desired_stop_gap|
    - w_signal * signal_violation
    - w_rule * overspeed_violation
    - w_risk * collision_or_high_risk

実装上の注意:
  - 速度違反ペナルティは高重みを維持する
  - 横Gは v^2*κ と v*yaw_rate の両方で評価する
  - 縦ジャーク・横ジャークの制約を報酬と外部評価器の両方に入れる
  - 停止線停止精度、停止時車間、信号遵守を報酬と外部評価器の両方に入れる
  - Rule-based最終速度評価と矛盾しない行動のみを高報酬化する
  - 実車適用前にシナリオ回帰テスト（カーブ、合流、停止線）を必須化する
```

---

## D.9 Stage 7: Joint Fine-tuning

```text
全モジュールを低学習率で同時に学習する最終ステージ。

凍結戦略:
  - BEV Backbone: 低LR（1e-5）
  - BEV Heads: 低LR（3e-5）
  - Planner: 標準LR（1e-4）
  - CondFormer: 標準LR（1e-4）
  - Scene Tokenizer: 低LR（3e-5）

損失:
  L_joint = L_stage2 + L_stage3 + L_stage4 + L_stage5 + L_stage6 (各損失の重み付き和)

注意:
  - Joint Fine-tuningは各モジュールが既に収束していることが前提
  - 一つのモジュールだけが突出して学習すると全体が崩れる可能性がある
  - val_lossを全モジュールで監視する

学習設定:
  Optimizer: AdamW (異なるモジュールに異なるlrを設定)
  Epochs: 6〜12
  Early stopping を積極的に使う
```

---

## D.10 OSS重みの活用ガイド（モジュール別）

### BEVFusion → BEV Encoder (Camera + LiDAR)

```python
import torch
from mmdet3d.models import build_model

# BEVFusion 公式重みのロード
checkpoint = torch.load("bevfusion-det.pth", map_location="cpu")
model = build_model(cfg.model)
model.load_state_dict(checkpoint["state_dict"], strict=False)
# strict=False で、カメラ台数等が違う場合でも読み込める
# 不一致のキーは無視される（新規初期化される）
```

### UniAD → Agent Future Predictor (Motion Transformer)

```python
# UniAD の motion_head 部分のみを取り出してwarm start
checkpoint = torch.load("uniad-stage1.pth", map_location="cpu")
state_dict = {
    k.replace("motion_head.", ""): v
    for k, v in checkpoint["state_dict"].items()
    if k.startswith("motion_head.")
}
model.agent_future_predictor.load_state_dict(state_dict, strict=False)
```

### DriveLM → VLM Teacher / Language Conditioning

```python
# DriveLMのQFormerベースのLanguage Encoder部分を活用
checkpoint = torch.load("drivelm-qa.pth", map_location="cpu")
# language encoder (BLIP2/QFormer系) の重みを抽出
lang_state_dict = {
    k.replace("language_model.", ""): v
    for k, v in checkpoint.items()
    if k.startswith("language_model.")
}
model.text_encoder.load_state_dict(lang_state_dict, strict=False)
```

---

## D.11 各モジュールのFreeze/Train テーブル

| モジュール | Stage1 | Stage2 | Stage3 | Stage4 | Stage5 | Stage6 | Stage7 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Camera Backbone | Train | Frozen | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| LiDAR Pillar Encoder | Train | Frozen | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| BEV Encoder (Fusion) | Train | Frozen | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| Temporal BEV Memory | Train | Frozen | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| BEV Heads | - | Train | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| Dynamic Object Head | - | Train | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| Agent Future Predictor | - | Train | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| Lane Topology Encoder | - | Train | Frozen | Frozen | Frozen | Frozen | Train(低LR) |
| Text Encoder | - | - | - | Train | Frozen | Frozen | Train(低LR) |
| CondFormer | - | - | Train(簡易) | Train | Frozen | Train(速度制約のみ) | Train |
| Planner Queries (K個) | - | - | Train | Frozen | Frozen | Train(速度寄り) | Train |
| VLA Planner Decoder | - | - | Train | Frozen | Frozen | Train(速度分岐のみ) | Train |
| Scene Tokenizer | - | - | - | - | Train | Frozen | Train(低LR) |

---

## D.12 Parked/Static/Dynamic のヒューリスティックラベル生成

Agent Futureの学習には、動体・停車車両・駐車車両の区別が必要である。

```python
def classify_agent_motion(agent_track: list) -> str:
    """
    agent_track: [AgentState, ...] (時系列)
    returns: "dynamic" | "stopped" | "parked"
    """
    velocities = [s.velocity for s in agent_track]
    
    # 最大速度
    v_max = max(abs(v) for v in velocities)
    
    # 停止していた期間の割合
    stopped_ratio = sum(1 for v in velocities if abs(v) < 0.3) / len(velocities)
    
    if v_max > 1.0:
        # 何らかの動きがあった
        if stopped_ratio > 0.8:
            return "stopped"  # ほぼ止まっているが動いた（信号待ち等）
        else:
            return "dynamic"
    else:
        # ほとんど動いていない
        return "parked"
```

---

## D.13 Agent Relevance 教師信号の構築

Plannerが注意を向けるべきエージェントを重み付けするための教師信号。

```python
def compute_agent_relevance(ego_trajectory, agent_state) -> float:
    """
    自車軌跡とエージェントの将来位置から関連度を計算
    """
    # 距離: 近いほど関連度高
    dist = distance(ego_trajectory[0], agent_state.position)
    dist_score = max(0.0, 1.0 - dist / 30.0)
    
    # 自車軌跡との交差: 軌跡が交差するほど関連度高
    intersection_score = check_path_intersection(
        ego_trajectory, agent_state.predicted_path
    )
    
    # 速度: 動いているほど関連度高
    v_score = min(1.0, agent_state.velocity / 5.0)
    
    relevance = 0.4 * dist_score + 0.4 * intersection_score + 0.2 * v_score
    return relevance
```

---

## D.14 低コスト構成での学習（R9700 + 32GB RAM）

手元のGPUが限られている場合の実用的な学習構成。

```text
ハードウェア例: AMD Ryzen 9 7900X + RTX 4090 24GB (または RTX 3090 24GB)

対応方針:
  1. BEVグリッド解像度を下げる
     - 512x512 → 256x256 または 128x128
     - 性能は落ちるが動作確認には十分

  2. バッチサイズを下げる
     - Batch size = 1 or 2
     - Gradient Accumulation (8〜16 steps) で実質的なバッチサイズを確保

  3. FP16 Mixed Precision Training
     torch.cuda.amp.autocast() + GradScaler を使う
     - VRAMを半分程度に削減

  4. モジュール分割学習
     - Stage 1のみ → Stage 2のみ、というように一度に一つのステージのみ学習

  5. 軽量Backbone
     - ResNet-50 の代わりに MobileNetV3 や EfficientNet-B0
     - BEVFormer-tiny (公開されている軽量版)

  6. K (候補数) を減らす
     - K=8 → K=4

実行例:
  python train.py \
    --stage 1 \
    --bev_resolution 256 \
    --batch_size 2 \
    --grad_accum 8 \
    --fp16 \
    --backbone efficientnet_b0
```

---

## D.15 Warm Start パターンの比較

| 戦略 | 初期精度 | 学習時間 | リスク | 推奨 |
|---|---|---|---|---|
| スクラッチ | 低 | 非常に長い | 高い | ✗ |
| BEVFusion warm start のみ | 中 | 中 | 中 | △ |
| BEVFusion + UniAD warm start | 高 | 短い | 低い | ◎ |
| UniAD 全体のfine-tuning | 高 | 中 | 中 | ○ |

**推奨**: BEVFusion (MIT版) を BEV Encoder に、UniAD の motion_head を Agent Future に使う。

---

## D.16 推奨ハイブリッドルート

```text
Step 1: BEVFusion (MIT) の重みをロード → Camera + LiDAR BEV Encoder に使用
Step 2: UniAD の重みをロード → Agent Future Predictor, BEV Heads に使用  
Step 3: Stage 2: BEV Heads を nuScenes + 自社データで Fine-tune
Step 4: Stage 3: Planner を人間軌跡データで学習
Step 5: Stage 4: CondFormer を言語データで学習
Step 6: Stage 5: Scene Tokenizer を VLM教師データで学習（オプション）
Step 7: Stage 6: Closed-loop / Offline RL Speed Policy Fine-tuning
Step 8: Stage 7: Joint Fine-tuning（低LR）
```

---

## D.17 Checkpoint 利用時の注意点

```text
公開checkpointを使う際の注意:

1. ライセンスの確認（付録C参照）
   - nuScenesで学習したモデルは CC BY-NC-SA 4.0 の影響を受ける可能性がある

2. モデルアーキテクチャのバージョン不一致
   - checkpoint を作ったコードと自分のコードのアーキテクチャが違う場合がある
   - strict=False でロードし、ロードできたキー / できなかったキーを必ず確認する

3. 入力形式の違い
   - カメラ台数、解像度、正規化方法が違う場合
   - LiDAR の点群フォーマット（x,y,z,intensity vs x,y,z,ring等）

4. 座標系の違い
   - nuScenes: 右手系、x=前方、z=上方
   - 自社センサ構成と同じか確認する

5. 出力レンジの違い
   - logit vs sigmoid vs softmax 後の値か
   - BEV の解像度・範囲が違う場合

確認コード例:
  checkpoint = torch.load("checkpoint.pth")
  model_keys = set(model.state_dict().keys())
  ckpt_keys = set(checkpoint["state_dict"].keys())
  
  missing = model_keys - ckpt_keys
  unexpected = ckpt_keys - model_keys
  
  print(f"Missing keys ({len(missing)}): {list(missing)[:5]}")
  print(f"Unexpected keys ({len(unexpected)}): {list(unexpected)[:5]}")
```

---

## D.18 付録Dのまとめ

```text
本付録で設計した要素:
  1. 学習の全体方針（5原則）
  2. 7ステージ学習計画（Backbone→Heads→Planner→Language→Scene→RL→Joint）
  3. 各ステージの詳細（損失関数・学習設定・warm start元）
  4. OSS重みの活用ガイド（BEVFusion/UniAD/DriveLM）
  5. Freeze/Train テーブル（全モジュール×全ステージ）
  6. Parked/Static/Dynamic ヒューリスティックラベル生成
  7. Agent Relevance 教師信号の構築
  8. 低コスト構成での学習（RTX 4090 24GB）
  9. Warm Start パターン比較
  10. 推奨ハイブリッドルート
  11. Checkpoint 利用時の注意点
```
