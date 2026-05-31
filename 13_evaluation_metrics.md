# 第13章 評価指標・ベンチマーク設計

---

## 13.1 評価設計の重要性

何を測るかを正確に定義することは、開発の方向性を決める。誤った指標を最適化すると、実際の性能は上がらない。

```text
よくある失敗:
  - Open-loop の ADE を下げることに集中 → Closed-loop で衝突率が改善しない
  - nuScenes のベンチマーク 1位 → 実車で普通に走れない
  - 平均値を改善 → 稀なケースが悪化している
  - 指標が良くなる → 実用性能は変わっていない（指標選択の誤り）

対策:
  - 指標の意味を理解した上で選ぶ
  - 複数の視点（指標）で評価する
  - シナリオ別に分解して分析する
  - 最終的には人間評価を組み込む
```

---

## 13.2 知覚系（BEV Head）の評価指標

### Occupancy / Drivable Area

$$\text{IoU} = \frac{TP}{TP + FP + FN}$$

| 指標 | 対象 | 意味 |
|---|---|---|
| mIoU (drivable 3-class) | BEV drivable label（3クラス） | 走行可否クラスの全体精度 |
| IoU (DRIVABLE) | bev_drivable class=1 | 通常走行域の一致度 |
| IoU (MARGINAL) | bev_drivable class=2 | 走行可だが推奨外領域の一致度 |
| IoU (occupancy) | BEV static occupancy | 静的障害物の一致度 |
| IoU (lane) | BEV lane center | 車線の一致度 |
| mIoU | 全クラス平均 | 全体的な BEV 品質 |

### 3D Object Detection (Agent Detection)

$$\text{mAP} = \frac{1}{|C|} \sum_{c} AP_c$$

| 指標 | 意味 |
|---|---|
| mAP | クラス別 Average Precision の平均 |
| NDS (nuScenes Detection Score) | mAP + 速度推定・向き推定・サイズ推定を統合 |
| ATE (Average Translation Error) | 位置誤差 [m] |
| ASE (Average Scale Error) | サイズ誤差（1 - IoU_3D） |
| AOE (Average Orientation Error) | 向き誤差 [rad] |
| AVE (Average Velocity Error) | 速度誤差 [m/s] |
| AAE (Average Attribute Error) | 属性誤差（stopped/moving等） |

---

## 13.3 予測系（Agent Future）の評価指標

### 軌跡予測

$$\text{ADE} = \frac{1}{T} \sum_{t=1}^{T} \| \hat{p}_t - p_t \|_2$$

$$\text{FDE} = \| \hat{p}_T - p_T \|_2$$

$$\text{minADE}_k = \mathbb{E} \left[ \min_{i \in [1,k]} \text{ADE}^{(i)} \right]$$

$$\text{minFDE}_k = \mathbb{E} \left[ \min_{i \in [1,k]} \text{FDE}^{(i)} \right]$$

$$\text{MR} = \frac{\text{予測がすべて閾値より外れたサンプル数}}{\text{全サンプル数}}$$

| 指標 | 説明 | 一般的な閾値 |
|---|---|---|
| ADE | 全時刻の平均距離誤差 | < 1.0 m |
| FDE | 最終時刻の距離誤差 | < 2.0 m |
| minADE@6 | 6候補のうち最良のADE | < 0.7 m |
| minFDE@6 | 6候補のうち最良のFDE | < 1.5 m |
| MR | 全候補が閾値以上外れた率 | < 10% |

---

## 13.4 計画系（Planner）の評価指標

### 軌跡品質

| 指標 | 計算方法 | 意味 |
|---|---|---|
| L2 error (1s/2s/3s) | 予測軌跡 vs 人間軌跡 のL2 | 人間走行への近さ |
| Collision Rate | 軌跡が障害物と重なる割合 | 安全性 |
| Off-route Rate (Hard) | NOT_DRIVABLE(0) 領域に入った割合 | 走行不可領域への侵入 |
| Off-route Rate (Soft) | MARGINAL(2) 領域に入った割合 | 走行推奨外領域への侵入 |
| Comfort (\|a_x\|) | 縦加速度の平均・最大 | 乗り心地 |
| Comfort (\|a_y\|) | 横加速度の平均・最大 | 乗り心地 |
| Comfort (jerk) | 加加速度 | 乗り心地 |

### 安全指標

| 指標 | 意味 |
|---|---|
| TTC (Time-to-Collision) | 衝突までの時間、閾値以下の割合 |
| THW (Time Headway) | 前走車との時間距離 |
| Fallback Rate | Fallbackが発動した割合 |

---

## 13.5 ベンチマークと本設計の対応

### nuScenes

```text
URL: https://www.nuscenes.org/
データ: 1,000シーン, 23 object categories, 6 cameras, LiDAR, Radar
評価: Detection NDS, Tracking AMOTA, Prediction ADE/FDE, Planning
本設計との対応:
  - BEV Head: nuScenes Detection NDS で評価可
  - Agent Future: nuScenes Prediction で評価可
  - Planner: IDM/Human走行を比較
  - 限界: シーン多様性は量産データより少ない
```

### nuPlan

```text
URL: https://github.com/motional/nuplan-devkit
データ: 1,500時間, Boston/Singapore/Pittsburgh/Las Vegas
評価: Reactive closed-loop simulation
指標:
  - Ego-Progress: 経路進捗
  - Ego-Expert: 人間走行への近さ
  - Comfort: 加速度
  - Collision: 衝突率
  - TTC: 時間距離
本設計との対応:
  - Phase 3のClosed-loop Simulationの参考に使える
  - PDM-Closed等の参考実装がある
```

### NAVSIM

```text
URL: https://github.com/autonomousvision/navsim
特徴: Non-reactive simulation（他車両は実軌跡に従う）
指標: PDMS (Planning Distribution Matching Score)
  - NC (No-collision): 衝突なし
  - DAC (Drivable Area Compliance): 走行可能領域遵守
    ※ NAVSIM の DAC は DRIVABLE(1) 領域遵守。MARGINAL(2) 侵入はペナルティ軽減で評価する
  - EP (Ego Progress): 進捗
  - TTC: 時間距離
  - Comfort: 快適性
本設計との対応:
  - Phase 1 の評価に適している
  - Non-reactive なのでClosed-loop より簡易
```

### Bench2Drive

```text
URL: https://github.com/Thinklab-SJTU/Bench2Drive
特徴: CARLA ベースの closed-loop, 44シナリオ
指標: Driving Score (RC + IS)
  - RC (Route Completion): 経路完走率
  - IS (Infraction Score): 違反ペナルティ
本設計との対応:
  - Phase 3 の Closed-loop 評価に使える
  - 実車に近い評価が可能
```

---

## 13.6 Open-loop vs Closed-loop の評価差

```text
Open-loop 評価の問題点:
  - 自車が何をしても他車は記録された軌跡を走り続ける
  - 自車が少しずれると、急に道路外にいる状態でLogが続く
  - 実際の危険な場面での挙動が評価できない

例:
  停車中の前走車を避けるため右に出た → その後の他車反応は元Logのまま
  実際にはその動作が他車に影響するが Open-loop では評価できない

Closed-loop 評価:
  - 他車がシステムの行動に反応する
  - より現実に近い
  - ただしシミュレータの現実性（Sim-to-Real gap）が問題

推奨:
  両方で評価する。Open-loopで基礎性能、Closed-loopで総合的な挙動を確認。
  Open-loop の数値だけで「良い」とは言えない。
```

---

## 13.7 Shadow Mode 専用の評価指標

Shadow Modeでは実走行との比較が可能になるため、より直接的な指標が使える。

| 指標 | 計算方法 | 意味 |
|---|---|---|
| Shadow ADE | システム軌跡 vs 実際の走行 のADE | 全体的な一致 |
| Shadow FDE | 終端位置の差 | 方向性の一致 |
| Suspicious Rate | Suspicious差分の割合 | 要確認シーンの割合 |
| Critical Rate | Critical差分の割合 | 即対応が必要な割合 |
| Fallback Rate | Fallback発動率 | 安全回路の過剰反応度 |
| Speed Profile Match | 速度プロファイルの相関 | 加速・減速パターンの一致 |
| Evaluator Reject Rate | Evaluatorが候補を全Rejectした率 | 評価器の過剰拒否 |

---

## 13.8 言語条件付きの評価

Language conditioning を持つ本システム特有の評価。

```text
言語指示への追従率:
  - DSL入力「右折」→ 出力軌跡が右折しているか
  - DSL入力「車線変更（右）」→ 出力軌跡が右車線変更しているか
  - DSL入力「停止」→ 出力軌跡が停止方向か

評価方法:
  - 各DSLコマンドに対して50+シナリオを用意
  - 軌跡が意図した方向に向かっているかをルールで判定
  - 言語を変えた時の軌跡変化量も確認

拒否率の評価:
  - 物理的に不可能な指示（狭路で「右車線変更」等）を与えた時
  - 候補がすべてFallbackになっているか
  - 不可能な指示を無視して安全な軌跡を出力しているか
```

---

## 13.9 シナリオカバレッジとシナリオ別スコア

全体平均だけでなく、シナリオ別に評価する。

```text
シナリオ分類:
  基本シナリオ:
    - 高速道路直進
    - 市街地直進
    - 右折
    - 左折
    - 車線変更（右/左）
    
  高難度シナリオ:
    - 駐車車両回避
    - 歩行者横断
    - 自転車横断
    - 不規則な交差点
    - 信号待ち再発進
    
  テールケース:
    - 工事区間
    - 白線消失
    - 霧・夜間
    - センサ欠損
    - 渋滞停止発進

評価の報告方法:
  - シナリオ別のmetrics tableで報告
  - 全体平均だけでは見えない弱点を明示する
  - 改善前後でシナリオ別の変化を確認する
```

---

## 13.10 回帰テストのシナリオセット設計

```text
シナリオセットの構成:
  - 100〜200シナリオ
  - シナリオ分類がバランスよく含まれる
  - 過去に問題があったシーンは必ず含める

シナリオ形式:
  A. Simulation シナリオ
    - CARLA等でのスクリプトシーン
    - 完全な再現性がある
    
  B. Log Replay シナリオ
    - 実走行ログの特定セグメント
    - 実際のセンサデータなので現実に近い

更新方針:
  - 新しい問題が発見されたらシナリオセットに追加
  - ただし増えすぎるとテスト時間が長くなる
  - 四半期に一度、古いシナリオを整理・統合する
```

---

## 13.11 人間評価（Human Evaluation）の組み込み

指標だけでは捉えられない品質を、人間評価で補完する。

```text
評価方法:
  A. Side-by-side evaluation
    - モデルAとBの出力を並べて評価者が選ぶ
    - 「どちらがより自然か」「どちらが安全か」
    
  B. Scenario review
    - Shadow Modeで収集したシーンを専門家が評価
    - 「この動作は許容できるか/改善が必要か」
    
  C. Expert ride
    - 専門ドライバーが実車で評価
    - 「快適さ」「信頼感」「自然さ」を5段階評価

人間評価の注意点:
  - 評価者の一貫性確保（ガイドライン整備）
  - 複数評価者の合意確認
  - 評価者の疲労・慣れへの対策
  - Shadow Modeとの組み合わせ
```

---

## 13.12 章のまとめ

```text
本章で設計した要素:
  1. 評価設計の重要性と失敗パターン
  2. 知覚系（BEV Head）の評価指標
  3. 予測系（Agent Future）の評価指標（ADE/FDE/MR等の数式）
  4. 計画系（Planner）の評価指標
  5. 主要ベンチマークとの対応（nuScenes, nuPlan, NAVSIM, Bench2Drive）
  6. Open-loop vs Closed-loop の違いと注意点
  7. Shadow Mode 専用の評価指標
  8. 言語条件付きの評価
  9. シナリオカバレッジとシナリオ別スコア
  10. 回帰テストのシナリオセット設計
  11. 人間評価の組み込み
```

次章では、ハードウェアプラットフォーム・量子化・OTAデプロイを述べる。
