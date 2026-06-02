# 第6章 人間軌跡教師とレーンセンター非依存Planning

---

## 6.1 レーン中心線追従の構造と限界

従来のADAS/自動運転では、計画の「正解」をレーン中心線（Lane Center）からの偏差として定義することが多い。

```text
従来の目標:
  lateral_error = 現在の横位置 - レーン中心
  heading_error = 現在の方位 - レーン方位

ペナルティ: lateral_error + heading_error を最小化
```

このアプローチは以下の場面で問題になる。

---

## 6.2 レーン中心線が失敗する場面

### 場面1: 駐車車両

```text
問題: 左路肩に駐車車両がいる
レーン中心線は駐車車両を通り抜けるため、
「正解」が障害物の中を通ることになる

人間: 少し右にオフセットして通過
レーン中心追従: 駐車車両に向かって進み、補正ルールが必要になる
```

### 場面2: 大型トラックとの並走

```text
問題: 隣の車線に大型トラックが走行中
人間: 自然に少しトラックから離れる（心理的マージン）
レーン中心追従: トラックの存在を無視してレーン中央を維持
→ 乗員は不安を感じ、実際にリスクも上がる
```

### 場面3: カーブ

```text
問題: きついカーブ
人間: カーブの外側から内側へ、Apexを使う自然なライン
（いわゆる「アウト・イン・アウト」、一般道では穏やかに）
レーン中心追従: カーブを均等な横位置で通過
→ 不自然な横Gが発生することがある
```

### 場面4: 交差点での停止位置

```text
問題: 交差点で右折待ちをする際の停止位置
人間: 交差点センターの少し手前、対向車が通れる位置
レーン中心追従: レーン中心のまま交差点中央へ進んでしまう
```

### 場面5: 白線が消えた区間

```text
問題: 工事区間、擦れた白線
レーン中心が計算できないため、制御が不安定になる

人間: 道路端・路肩・前走車を見て自然に判断
```

---

## 6.3 人間軌跡を教師として使う考え方

Comma.ai/openpilotのE2E Lateral Planningから学ぶ重要な発想は、**レーン中心線ではなく、人間が実際に走った走行ログの未来軌跡を教師にする**ことである。

```text
Human Trajectory:
  - 走行ログから、時刻tから数秒先までの自車位置を取得
  - これをegoフレームに変換する
  - この軌跡をPlannerの目標にする
```

### なぜ強いのか

```text
1. 暗黙知が含まれる
   - 駐車車両回避の横オフセット
   - 大型車からの距離
   - カーブのナチュラルライン
   - 交差点での停止位置
   - これらは人間が自然に行う判断であり、ルールで書くのは難しい

2. スケールしやすい
   - 大量の走行ログを低コストで収集できる
   - 専用ラベリングなしで教師データが作れる

3. 車両非依存
   - 軌跡はメートル単位、ego基準
   - 車種が変わっても同じ軌跡データが使える
```

---

## 6.4 毎サイクル、egoフレーム基準の軌跡

本設計では、Plannerは毎処理サイクルで現在のegoフレーム基準の目標軌跡を出す。

```text
目標軌跡:
  - 現在時刻 t = 0 を基準
  - T ステップ先まで (例: 3秒先, T=10ステップ, 0.3秒間隔)
  - 各点: (x, y, yaw, v)

注意:
  - 前フレームの軌跡を引き継がない（毎サイクル再計算）
  - これにより急な場面変化に素早く対応できる
  - 前フレームとの不連続は Converterが滑らかに処理する
```

---

## 6.5 Converter vs Controller の責任分離

人間軌跡を教師にしたPlannerと、実際の車両への制御指令は、異なるモジュールが担う。

```text
Planner (本章の対象):
  - 毎サイクル、目標軌跡 [T, D] を出す
  - 車両モデルを知らない
  - 車種非依存

Converter (第7章の対象):
  - 目標軌跡 + Ego state → 目標曲率（舵角）
  - 決定論的な変換
  - 車両パラメータ（ホイールベース、ステアリングレシオ等）を知る
```

この分離により：
- Plannerは学習可能で、車両に依存しない
- Converterはテスト可能で、透明性が高い

---

## 6.6 レーンをContextとして使う、Targetとして使わない

本設計の重要な設計判断：

```text
従来: lane_center = TARGET (偏差を最小化)
本設計: lane_center = CONTEXT (参考情報としてCondFormerへ)

Plannerは:
  - Lane Topologyを「どこへ向かうか」の情報として使う
  - ただし、レーン中心への追従を直接最適化しない
  - 人間軌跡との差を最小化する
```

### なぜこれが良いのか

```text
レーン中心をContextにすることで:
  - レーン内でのオフセット（駐車車両回避等）が自然に学べる
  - 白線がない場面でもクラッシュしない（教師が人間なので）
  - レーン変更時の自然な軌跡が学べる
  - レーン情報の精度が下がっても、人間教師が補完する
```

---

## 6.7 人間軌跡ラベルの構築方法

走行ログから教師軌跡を作る方法。

```text
Step 1: ログから自車の未来軌跡を取得
  - GPS + IMU + dead reckoning で精密な軌跡
  - t = 0 から T=10 ステップ先まで
  - グローバル座標で取得

Step 2: 現在のegoフレームに変換
  - 現在の ego position/yaw を基準に変換
  - ego_transform(global_traj, ego_pose_t) → local_traj

Step 3: 速度プロファイルを追加
  - 各点での実際の車速を付加
  - v_t = ||(pos_{t+1} - pos_t)|| / Δt

Step 4: 品質フィルタリング
  - 急ハンドル・急ブレーキのログを除外 or 低ウェイト
  - センサ障害・ドライバー介入のシーンを除外
  - 夜間・悪天候の品質が低いシーンを調整
```

---

## 6.8 データ品質管理

```text
除外すべきシーン:
  - ドライバー介入（ハンドル持ち替え, 緊急ブレーキ）
  - センサ欠損（LiDARなし, GPS遮断）
  - 異常な横加速度 (|a_y| > 5 m/s^2)
  - 異常な縦加速度 (|a_x| > 8 m/s^2)
  - 交通違反のあるシーン（速度超過等）

低ウェイトにすべきシーン:
  - 降雨 / 低視程
  - 夜間 (LiDAR等で補完)
  - 工事区間

高ウェイトにすべきシーン:
  - 頻度の低いシナリオ（歩行者横断、割り込み等）
  - 自然な車線変更
  - 駐車車両回避
```

---

## 6.9 K-candidate + MHP Lossによる多モード学習

一つのシーンに対して、人間軌跡は一通りしかない（ログは1本）。しかし、Plannerは複数候補を出す。

```text
Multiple Hypothesis Prediction (MHP) Loss:
  - K候補のうち、ground truthに最も近い候補 k* を選ぶ
  - その候補 k* に対してのみ軌跡誤差損失を与える
  - これにより、すべての候補が「正解の1つ」を学ぶ競争をする

L_traj = min_k Σ_t || traj_k_t - gt_t ||_2^2
       = Σ_t || traj_{k*}_t - gt_t ||_2^2
```

### なぜ多モードが必要か

```text
同じシーンでも複数の「正しい」行動がある:
  - 右に避けるか左に避けるか
  - 加速して先に進むか減速して待つか
  - 先行車の直後について行くかマージンを大きく取るか

K候補を持てば、これらを全て学習できる。
外部評価器が安全側の候補を選ぶ。
```

---

## 6.10 速度計画と縦方向教師

人間軌跡には速度プロファイルが含まれる。これを縦方向計画の教師にする。

```text
縦方向の教師信号:
  - gt_v: 各タイムステップでの実際の車速
  - gt_ax: 縦加速度

Plannerの縦方向損失:
  L_speed = Σ_t || v_pred_t - gt_v_t ||_2^2
  L_accel = Σ_t || a_pred_t - gt_ax_t ||_2^2
```

### 信号対応・停止線対応の特別扱い

```text
信号停止・一時停止のシーンでは:
  - gt_v が急激に0に落ちる
  - L_speed の重みを通常の2倍にする
  - 停止線までの距離をCondFormerへの入力として追加する
  - 信号状態（RED/YELLOW/GREEN/ARROW/UNKNOWN）を速度計画の条件として追加する
```

### 停止線停止・車間停止の精度

```text
停止精度は、人間模倣だけではばらつきやすい。
必要な挙動:
  - 赤信号・一時停止では停止線手前の許容範囲に止まる
  - 前走車停止時は毎回ほぼ同じ時間/距離マージンで止まる
  - 停止直前に不要な再加速やカックンブレーキを出さない

このため、ILで停止位置の分布を学び、closed-loop/offline RLで停止誤差を最小化する。
```

```text
追加する教師・状態量:
  - stopline_distance_m
  - traffic_light_state
  - lead_vehicle_distance_m
  - lead_vehicle_speed_mps
  - desired_stopping_gap_m
  - final_stop_position_error_m
```

### 軌跡生成と同時に目標速度プロファイルを生成

```text
Plannerは横方向軌跡だけでなく、同じ時刻グリッドで縦方向プロファイルも同時出力する。

出力（各時刻t）:
  - v_target_t      : 目標速度
  - a_x_target_t    : 目標加速度（加速/減速）
  - phase_t         : ACCEL / CRUISE / DECEL

これにより、Converter側で
  - 目標舵角（横）
  - 目標速度（縦）
  - 目標加速度（縦）
を同じ候補から一貫して計算できる。
```

### カーブ進入・旋回中・脱出の速度挙動

```text
望ましい挙動:
  1) カーブ手前で予見的に減速
  2) 旋回中はほぼ一定速度を維持
  3) 脱出時に段階的に再加速

この挙動は「人間模倣だけ」では再現が不安定になりやすい。
理由:
  - データ分布がドライバー癖に依存
  - 先読みの最適性（横加速度・快適性・時間効率）が暗黙化される
```

### IL + 閉ループ最適化

```text
近年の自動運転研究では、
「人間ログの模倣学習だけ」で完結させず、次を組み合わせる流れが主流である。

  1. 大規模ログによるIL（behavior cloning / trajectory imitation）
  2. 閉ループシミュレーションでの評価・改善
  3. offline RL / constrained RL / planning-oriented loss による速度・快適性補正
  4. 最終段のrule-based evaluatorによる安全・法規制約

ここでRLは、実車で自由に探索するonline RLではなく、
シミュレータと記録済みログを使うoffline/closed-loop fine-tuningとして扱う。
```

```text
学習方針:
  Phase A: 大規模IL（人間軌跡・速度）
    - 一般交通での自然な意思決定、車線選択、停止位置を学習

  Phase B: 閉ループシミュレーション評価
    - imitation lossだけでは見えない compounding error を検出
    - カーブ、合流、停止線、信号機、前走車追従での閉ループ挙動を評価

  Phase C: offline RL / constrained RL（速度・快適性最適化）
    - カーブ進入/脱出など先読み速度制御を最適化
    - 代表報酬:
        r = w_progress * 進捗
          - w_jerk * |jerk|
          - w_lat * ReLU(v^2*κ - a_y_max)
          - w_lat_meas * ReLU(|v*yaw_rate| - a_y_max)
          - w_stop * |stop_position_error|
          - w_gap * |actual_stop_gap - desired_stop_gap|
          - w_rule * 速度制限違反
          - w_risk * 衝突・高リスク接近

  Phase D: IL + RL objective の混合再学習
    - 人間らしさを維持しつつ、曲率先読み速度と閉ループ安定性を改善
```

```text
実運用のポイント:
  - RLはシミュレータ主体で事前学習し、実車ログ/offline datasetで微調整する
  - 実車での探索行動は行わない
  - 速度違反ペナルティは高重みで与え、規制速度超過を抑制する
  - 信号違反、停止線超過、停止時車間不足は高重みでペナルティ化する
  - 外部評価器のrule-based速度チェックを最終安全弁として残す
```

### Phase 1 MVP：K-query VLA Planner（§5.5 の設計をそのまま適用）

```text
K-query Transformer Decoder は実装の容易さと計算コストの低さから
ロードマップ Phase 1〜2（MVP〜閉ループ評価）に適する。

  構成:
    - K=8〜24 本の学習可能クエリ
    - Transformer Decoder (4〜6 layers)
    - MHP Loss: L_traj = min_k L2(traj_k, traj_human)
    - 速度プロファイルヘッド: (v_target, a_x, phase) per timestep

  利点:
    - Forward 1 pass で K 候補を生成、デバッグが容易
    - 標準的な Transformer 実装で GPU メモリが少ない
    - MHP Loss が直感的で収束しやすい
    - §6.11 の縦横出力形式はそのまま使用可能

  適用シナリオ:
    ロードマップ Phase 1〜3（Offline Open-loop〜Shadow Mode）での検証に最適。
    BEV 知覚・Evaluator・Converter が安定した後、DiffusionDrive への移行を検討する。
```

### Phase 2 発展：DiffusionDrive（拡散モデル軌跡生成）＋外部評価器

```text
K-query Planner の安定稼働確認後、以下の場面で DiffusionDrive への移行を検討する。

  移行を検討する条件:
    - K 固定候補では稀少シナリオのカバレッジが不足する
    - 停止精度・IDM 追従・先読みカーブ速度をさらに向上させたい
    - 候補の連続的多様性と Evaluator の組み合わせで安全選択の質を上げたい

  DiffusionDrive の利点（K-query との比較）:
    - (x, y, v, a_x, phase) を denoising 1 pass で縦横一体生成 → §6.11 保証
    - 条件入力（stopline/TL/lead_veh/curvature）に対するdenoising lossだけで
      §6.12 の多フェーズ挙動が暗黙的に学習される（報酬設計不要）
    - 連続分布サンプル M=16〜32 → Evaluator が常に選択可能な候補を保証
    - NVIDIA ECCV 2024 で NVIDIA Drive 上のリアルタイム性実証済み
```

```text
本設計における DiffusionDrive 採用時の構成:

  採用手法：DiffusionDrive（条件付き拡散モデル）＋外部評価器によるサンプル選択

  §6.12 挙動の学習方法（条件付け入力として組み込む）：
    拡散モデルは以下を条件入力（conditioning）として受け取る：

      stopline_distance_m     → 停止線接近時の速度ゼロへの収束（§6.12 停止プロファイル）
      traffic_light_state     → 赤/黄での予見的減速開始
      lead_vehicle_distance_m → IDM類似の車間追従挙動（§6.12 IDM）
      lead_vehicle_speed_mps  → 相対速度に応じた加減速
      curvature_profile[t]    → 前方カーブに連動した先読み速度制御（§6.12 予見的カーブ）
      desired_speed_mps       → 制限速度・巡航速度の上限

    これらの条件入力に対して人間ドライバーのログが自然なS字プロファイルや
    IDM類似追従・先読み減速を示しているため、denoising lossのみで
    §6.12 の複雑な多フェーズ挙動が暗黙的に学習される。

  推論フロー：
    1. 条件入力（BEV特徴量, Lane Topology, 上記速度条件）を与えてM個の軌跡をサンプル
       （例: M=16〜32、RTX/Drive系GPUで100ms以内に完了）
    2. 外部評価器が全Mサンプルを §6.11 の縦横一体スコアで評価・フィルタ
    3. 最高スコアの軌跡 k* をConverterへ送出
```

```text
学習パイプライン（3フェーズ）：

  Phase A: 大規模IL（拡散モデルのdenoisingプレトレーニング）
    - 損失: L_diffusion = E[ ||eps - eps_theta(traj_noisy, t_noise, cond)||^2 ]
    - condは全条件入力（速度条件 + BEV特徴量 + Lane Topology）
    - 人間軌跡の速度プロファイルを (x, y, v, a_x, phase) として正規化して学習
    - §6.12 の多フェーズ挙動がここで暗黙的に習得される

  Phase B: 閉ループシミュレーション評価
    - WaymaxまたはCARLAで閉ループ挙動を計測
    - 分布シフト（compounding error）の検出と再学習箇所の特定

  Phase C: CMDP制約付きファインチューニング（必要な場合）
    - Lagrangian CMDPでサンプリング分布を絞り込む
    - 外部評価器の制約フラグ C_1..C_4 がそのままコスト信号となる
```

---
## 6.11 縦横同時最適化の原理と構造的利点（Joint Lateral-Longitudinal Optimization）

### 縦横結合の根本問題

横方向（ステアリング）と縦方向（速度/加速度）を独立に計画するアプローチは、一見シンプルに見えるが、本質的な物理的結合がある。

```text
縦横が独立でない理由:

【横加速度制約】
  a_y = v^2 * κ  ≤  A_Y_MAX

  → 曲率 κ が決まれば、安全に走れる速度の上限が決まる
  → 速度 v が決まれば、許容できる曲率 κ の上限が決まる
  → 2つは独立でなく、互いの設計に影響する

  例: κ = 0.05 (1/m), A_Y_MAX = 3.0 m/s^2
    → v_max = sqrt(3.0 / 0.05) = 7.75 m/s (約28 km/h)
    → 50km/h で曲率 0.05 のカーブには入れない

【停止線接近の例】
  - 右折レーン進入（横方向変化）と赤信号停止（縦方向減速）が同時に必要
  - 分離型: 横計画が「レーン変更を急ぎすぎ」 → 縦計画が「停止が間に合わない」
  - 統合型: 「緩くレーン変更しつつ早めに減速」という解を一度に選べる
```

### 従来の非結合型計画の失敗モード

```text
非結合型アプローチ（逐次計画）:
  Step 1: 横方向の軌跡 (x_t, y_t) を計画（速度考慮なし）
  Step 2: その軌跡の曲率 κ_t を後から計算
  Step 3: κ_t に基づいて速度プロファイル v_t を後付け計算

問題点:
  1. 【不整合】Step 1 で「急カーブを通る軌跡」を選んでも、
     Step 3 でその速度への移行が遅れる
     → カーブに高速で入り、横Gが A_Y_MAX を超える

  2. 【候補選択の失敗】K候補のうち「少し外側を通る緩やかなカーブ」を
     選べば高速を維持できるのに、Step 1 は速度を知らないため選べない

  3. 【快適性のグローバル喪失】横ジャーク と 縦ジャーク の同時ピークを
     避けるトレードオフは、分離計画では表現できない

  4. 【予見的減速のズレ】先読みカーブ前の減速開始タイミングが
     横計画と縦計画で整合しないことがある
```

### 縦横同時最適化の数理

Plannerは横・縦を **同一の時刻グリッドで同時出力** する。

```text
Plannerの出力（K候補 × T時刻ステップ）:
  trajectory[k][t] = {
    x_t:        前方距離 (m)           ← 横方向
    y_t:        横方向オフセット (m)   ← 横方向
    v_target_t: 目標速度 (m/s)         ← 縦方向
    a_x_t:      目標縦加速度 (m/s^2)   ← 縦方向
    phase_t:    ACCEL / CRUISE / DECEL ← 縦方向状態
  }

物理制約は両方向を含んでいるため、
候補 k* の選択は必ず 5次元の一体評価でなければならない。
```

```text
External Evaluatorにおける縦横一体評価:

  Fail 判定（安全制約）:
    - v_t^2 * |κ_t| > A_Y_MAX             (横G超過)
    - |a_x_t| > A_X_MAX                   (縦加速度超過)
    - |Δa_x_t / Δt| > JERK_X_MAX          (縦ジャーク超過)
    - |Δ(v_t^2 * κ_t) / Δt| > JERK_Y_MAX (横ジャーク超過)

  Score 付け（Pass した候補のランキング）:
    comfort_score = - w1*E[|a_y|] - w2*E[|a_x|]
                    - w3*E[|jerk_x|] - w4*E[|jerk_y|]
    progress_score = total_distance / total_time
    total = w_c * comfort_score + w_p * progress_score

横方向だけ、または縦方向だけが優れた候補は落第になりうる。
両者がバランスよく設計されて初めて高スコアを得る。
```

### 縦横同時学習の構造的利点

```text
1. 曲率連動速度学習（Curvature-Aware Speed Learning）
   - 人間はカーブを「見越して」速度を落とす（予見的制御）
   - これは縦・横が脳内で一体計算されているため
   - 分離型: カーブ検知→速度計算 のシーケンスが遅れる
   - 統合型: 「カーブがあるから今から減速する候補」をまとめて学習

2. 快適性の全体最適化
   - 報酬 r = - w_jerk_x * |jerk_x| - w_jerk_y * |jerk_y| で学習
   - 縦横ジャークが同時ピークになる状況（急ハンドル + 急ブレーキ）を回避

3. 候補多様性の活用
   - 候補A: 少し遠回りだが緩カーブ → 高速維持 → 時間効率が良い
   - 候補B: 短絡ルートだが急カーブ → 要減速 → 遅い
   - 速度を含む5次元評価によってのみ正しく比較できる

4. 実装のシンプルさ
   - 1回の Forward Pass で横・縦の全情報が揃う
   - 独立した速度プランナーモジュールが不要
   - 横・縦のインターフェース不整合バグが発生しない

```

---

## 6.12 速度プロファイルの詳細設計

### 速度・加速度・ジャークの3階層制約

人間が快適と感じる運転の制約は、3つの時間微分レベルで存在する。

```text
第1階層: 速度 v (m/s)
  - 法規速度制限以下
  - 曲率許容値: v ≤ sqrt(A_Y_MAX / |κ|)
  - 前走車追従: v ≤ IDM 制約速度

第2階層: 加速度 a = dv/dt (m/s^2)
  - 縦加速度: |a_x| ≤ A_X_MAX (例: 快適走行 2.0, 緊急時 5.0 m/s^2)
  - 横加速度: |a_y| = v^2 |κ| ≤ A_Y_MAX (例: 快適 2.5 m/s^2)

第3階層: ジャーク j = da/dt (m/s^3)
  - 縦ジャーク: |j_x| = |Δa_x / Δt| ≤ JERK_X_MAX
  - 横ジャーク: |j_y| = |Δ(v^2 κ) / Δt| ≤ JERK_Y_MAX
  - ジャーク制限により「カックンブレーキ」「急ハンドル」を防ぐ

ISO 2631（乗り心地・乗り物酔い基準）の参考値:
  縦ジャーク < 2.0 m/s^3: 快適
  縦ジャーク < 5.0 m/s^3: 許容
  縦ジャーク > 5.0 m/s^3: 不快・改善必要

  横ジャーク < 1.5 m/s^3: 快適（乗り物酔いリスク低）
  横ジャーク > 2.0 m/s^3: 改善必要
```

### キネマティック停止プロファイル（停止線・赤信号）

現在速度 v0 から距離 d_stop 先でゼロ速度に達する、ジャーク制限付き停止プロファイル。

```text
目標:
  - v(d_stop) = 0 （停止線に精度よく停止）
  - 過不足なく停止（停止誤差 < STOP_POS_TOL）
  - ジャーク制限を満たす（S字プロファイル）

【一定減速モデル（最も単純）】
  a_decel = v0^2 / (2 * d_stop)
  v(s) = sqrt(v0^2 - 2 * a_decel * s)

  問題: d_stop が短い場合に a_decel > A_X_MAX となりジャーク制限に違反

【S字（3フェーズ）停止プロファイル】
  Phase 1（ジャーク増大期）: a が 0 → -A_X_MAX まで線形変化, 期間 t1 = A_X_MAX / J_MAX
  Phase 2（一定減速期）:      a = -A_X_MAX を維持
  Phase 3（ジャーク緩和期）:  a が -A_X_MAX → 0 まで線形変化, v → 0

  最小制動距離（快適停止可能な最短距離）:
    d_min = v0^2 / (2 * A_X_MAX) + A_X_MAX * t1^2 / 3  (近似)

  判定フロー:
    if d_stop >= d_min:
      S字プロファイル（快適）
    elif d_stop >= d_emergency_min:
      一定最大減速（乗客不快だが安全、乗客に通知推奨）
    else:
      AEB（自動緊急ブレーキ）トリガー条件 → AEBモジュールへ移管
```

```python
def compute_stop_profile(v0, d_stop, a_max, j_max, dt=0.1):
    """
    ジャーク制限つき停止プロファイルの生成
    Returns: [(v_t, a_t, phase_t)] for t in [0, T]
    """
    t1 = a_max / j_max                    # ジャーク増大/緩和の期間
    v1 = v0 - 0.5 * j_max * t1 ** 2       # Phase 1 後の速度
    d1 = v0 * t1 - j_max * t1 ** 3 / 6    # Phase 1 の移動距離
    d3 = v1 ** 2 / (2 * a_max)            # Phase 3 の移動距離（一定減速近似）
    d2 = d_stop - d1 - d3                 # Phase 2 の移動距離

    if d2 < 0:  # 距離が短い: 単純一定減速にフォールバック
        a_decel = min(v0 ** 2 / (2.0 * max(d_stop, 0.1)), a_max * 2.0)
        return constant_decel_profile(v0, d_stop, a_decel, dt)

    # 3フェーズ S字プロファイル生成
    profile = []
    v, a = v0, 0.0
    for phase, d_phase in [('DECEL_RAMP', d1), ('DECEL', d2), ('DECEL_EASE', d3)]:
        pos = 0.0
        while pos < d_phase and v > 0:
            if phase == 'DECEL_RAMP':
                a = max(a - j_max * dt, -a_max)
            elif phase == 'DECEL_EASE':
                a = min(a + j_max * dt, 0.0)
            v = max(v + a * dt, 0.0)
            pos += v * dt
            profile.append((round(v, 4), round(a, 4), 'DECEL'))
    return profile
```

### IDM（Intelligent Driver Model）による前走車追従速度制限

```text
IDM（Treiber et al. 2000）の適応追従モデル:

  desired_gap(v, v_lead) = s0 + v * T
                           + v * (v - v_lead) / (2 * sqrt(a_comf * b_comf))

  パラメータ:
    s0    : 最小静止車間距離 (例: 2.0 m)
    T     : 時間ヘッドウェイ (例: 1.5〜2.0 s)
    a_comf: 快適加速度 (例: 1.5 m/s^2)
    b_comf: 快適減速度 (例: 2.0 m/s^2)
    v     : 自車速度
    v_lead: 先行車速度 (前走車停止時 v_lead=0)

  追従加速度:
    a_idm = a_comf * [1 - (v / v_free)^4 - (desired_gap / actual_gap)^2]

    v_free  : 自由走行目標速度（法規速度以下に設定）
    actual_gap: BEV Dynamic Head から得た前走車距離

  特性:
    actual_gap >> desired_gap → 自由走行 (v → v_free)
    actual_gap ≈ desired_gap → 定車間追従
    actual_gap < desired_gap → 急制動 (a_idm 急激に負)
    v_lead = 0 かつ actual_gap < desired_gap → 自車も停止
```

```text
本設計における IDM の使い方:
  方式A（Converterでのクリッピング）:
    acc_idm = compute_idm(v, v_lead, actual_gap)
    target_accel = min(planner_accel, acc_idm)
    target_speed = min(planner_speed, v + acc_idm * lookahead_time)

  方式B（Plannerへの状態量入力）:
    CondFormer への入力特徴量に以下を追加:
      - lead_gap_m          (BEVから)
      - lead_speed_mps      (BEVから)
      - relative_speed_mps  (= v - v_lead)
    → Planner が IDM 類似の追従挙動を自然に模倣学習する

  推奨: 方式A と 方式B を併用する。
  Plannerが適切な追従を学習しつつ、Converterが最終安全弁として機能する。
```

### 予見的カーブ速度制御（Anticipatory Curve Speed Control）

前方の曲率ピークを先読みして、適切なタイミングで減速を始める。

```text
問題の設定:
  - 現在位置は直線 (κ ≈ 0)
  - 50m 先にカーブ頂点 κ_peak = 0.05 (1/m)
  - A_Y_MAX = 3.0 m/s^2 → 許容速度 v_curve = sqrt(3.0/0.05) ≈ 7.75 m/s
  - 現在速度 v0 = 13.9 m/s (50 km/h)

先読み減速の計算:
  v_target = sqrt(A_Y_MAX / κ_peak) = 7.75 m/s
  d_available = 50 m  (カーブ頂点までの距離)
  d_required  = (v0^2 - v_target^2) / (2 * A_X_MAX)
              = (193.2 - 60.1) / (2 * 2.0) = 33.3 m

  d_available (50m) > d_required (33.3m) → 緩やかな減速で余裕あり:
    a_x = -(v0^2 - v_target^2) / (2 * d_available)
        = -133.1 / 100 = -1.33 m/s^2  (快適減速範囲内)

  もし d_available < d_required なら:
    最大減速でも間に合わない → 経路変更 or 減速後にカーブ再挑戦
    → External Evaluator がこの候補を Fail

Plannerが学習する挙動:
  - 人間ドライバーは「先のカーブを見越して」速度を下げる
  - この予見的制御は IL で学習し、RL で精度を高める
  - BEV の Lane Topology から先のカーブ曲率プロファイルが取得可能
  - 学習が不十分でもExternal Evaluatorが補正する（フェイルセーフ）
```

### S字（ジャーク制限）加減速プロファイル

```text
加速時（発進・合流・追い越し）:
  Phase 1: j = +J_MAX で a を 0 → +A_X_MAX まで増大（期間 t1 = A_X_MAX/J_MAX）
  Phase 2: a = +A_X_MAX を維持（v が上昇）
  Phase 3: j = -J_MAX で a を +A_X_MAX → 0 まで減少

減速時（停止・前走車減速）:
  Phase 1: j = -J_MAX で a を 0 → -A_X_MAX まで増大（ブレーキ踏み込み）
  Phase 2: a = -A_X_MAX を維持（v が下降）
  Phase 3: j = +J_MAX で a を -A_X_MAX → 0 まで戻す（クリープ停止）

典型的なパラメータ（乗用車, 快適走行クラス）:
  A_X_MAX    = 2.0 m/s^2
  A_Y_MAX    = 2.5 m/s^2
  JERK_X_MAX = 2.0 m/s^3
  JERK_Y_MAX = 1.5 m/s^3

ロボタクシー/高快適クラス（乗客保護重視）:
  A_X_MAX    = 1.5 m/s^2
  A_Y_MAX    = 2.0 m/s^2
  JERK_X_MAX = 1.0 m/s^3
  JERK_Y_MAX = 1.0 m/s^3

緊急回避クラス（安全最優先）:
  A_X_MAX    = 8.0 m/s^2  (AEB領域)
  A_Y_MAX    = 5.0 m/s^2  (タイヤ摩擦限界付近)
  JERK_X_MAX = 無制限
  JERK_Y_MAX = 無制限
```

### 速度フェーズ管理（ACCEL / CRUISE / DECEL）

```text
フェーズ遷移の条件（優先順位順）:

  【DECEL 優先（最上位）】
    - traffic_light_state == RED かつ stopline_distance < d_brake_start(v)
    - traffic_light_state == YELLOW かつ 停止可能距離以内
    - stopline_distance_m < d_brake_start(v)  (停止線近接)
    - actual_gap < desired_gap(v)              (車間不足)
    - v > v_limit_curve = sqrt(A_Y_MAX / κ_next_peak)  (先読みカーブ速度超過)
    - v > legal_speed_limit                    (速度制限超過)

  【CRUISE（中位）】
    - |v - target_speed| < 0.5 m/s            (目標速度近傍)
    - DECEL 条件に非該当
    - actual_gap ≈ desired_gap

  【ACCEL（低位）】
    - v < target_speed - 0.5 m/s
    - 前走車との車間が desired_gap * 1.5 以上
    - 速度制限・横G制限内

DECEL フラグが立ったら前走車が加速しても即 DECEL を維持する。
ACCEL への復帰は DECEL 条件が全て解消されてから一定時間後（ヒステリシス）。
```

---
## 6.13 データ収集の規模感

```text
Phase 1 (プロトタイプ):
  - 数万〜数十万km
  - 代表的なODD内のシーン
  - 品質重視

Phase 2 (製品化):
  - 数百万km
  - テールケース（稀なシナリオ）を補強
  - フリート収集

Phase 3 (フリート学習):
  - 数千万km以上
  - Shadow Modeで大量収集
  - 継続的な改善ループ
```

---

## 6.14 Structured E2Eとしての位置づけ

```text
End-to-End (pure):
  sensors → NN → steering angle
  長所: シンプル
  短所: 中間表現なし、製品検証難しい

Structured E2E (本設計):
  sensors → BEV world → trajectory candidates → evaluator → converter
  長所: 中間表現あり、製品検証可能、車両非依存
  短所: 設計が複雑、モジュール間インターフェースの管理が必要
```

本設計は「End-to-Midとも呼べる」設計で、人間軌跡を学ぶ能力と製品として必要な検証性を両立させる。

---

## 6.15 章のまとめ

```text
本章で設計した要素:
  1. レーン中心線追従の限界（5シナリオ）
  2. 人間軌跡教師のアプローチ
  3. 毎サイクル・egoフレーム基準の目標軌跡
  4. Converter との責任分離
  5. レーンをContext（非Target）として使う設計
  6. 人間軌跡ラベルの構築方法
  7. データ品質管理
  8. K-candidate + MHP Loss
  9. 速度計画・縦方向教師（信号/停止精度/IL+RL）
  10. 停止線・車間停止の精度と RL による改善
  11. 軌跡生成アーキテクチャ：K-query VLA Planner（MVP実装）→ DiffusionDrive（拡散
      モデル、製品発展）。§6.12 挙動（停止・IDM・カーブ速度）を条件入力で学習（6.10）
  12. 縦横同時最適化の原理・利点・研究動向（6.11）
  13. 速度プロファイルの詳細設計（S字・IDM・先読みカーブ）（6.12）
  14. データ収集の規模感
  15. Structured E2Eとしての位置づけ
```

次章では、選択された軌跡から実際の舵角指令へ変換するTrajectory-to-Steering Converterを詳述する。
