# 第9章 製品化・安全性・法規・ODD・ログ

---

## 9.1 製品として必要な中間表現の明示化

純粋なブラックボックスE2Eでは、「なぜこの行動をとったか」を説明できない。  
製品として、次の情報が外部から観測可能でなければならない。

```text
必須の可観測中間表現:
  1. BEV World（走行可能域・占有・不確実性）
  2. Lane Topology（車線グラフ）
  3. Dynamic World（動体位置・速度）
  4. K候補軌跡
  5. 選択理由
  6. ODD判定結果
  7. センサ健全性
```

これらが観測できると：
- 事故解析でどの情報が誤っていたか特定できる
- 外部評価器がルールチェックをできる
- 安全ケースに証拠として使える
- OTA更新後の性能回帰を検出できる

---

## 9.2 K候補の安全側選択

```text
外部評価器の選択ロジック:
  1. 全K候補に対してチェックリストを実行
  2. Passed候補の中から confidence 最大を選択
  3. 全候補が Fail の場合 → MRM (Minimum Risk Maneuver)

チェックリスト:
  - Static occupancy との衝突なし
  - Dynamic Risk Map のリスク値 < 閾値
  - 走行可能域内
  - 速度制限内
  - 加速度・曲率・横G・ジャークが快適性範囲内
  - 信号機・停止線・横断歩道の制約を満たす
  - 停止線停止精度と停止時車間が目標範囲内
  - ODD内
  - 最終ルールベース速度評価を通過（map/sign/road_mark/user設定の統合）

MRM (Minimum Risk Maneuver):
  - 安全な速度での走行継続 or 安全停車
  - 人手での設計・検証が必要なモジュール
```

### 最終ルールベース速度評価

```text
目的:
  学習モデルの提案速度を、そのまま採用せず、
  法規・地図・認識・ユーザー設定の整合をとって最終評価する。

入力:
  - map由来の速度上限
  - 標識Head由来の速度上限
  - 路面標識Head由来の速度上限
  - ユーザーGUIの overspeed tolerance

評価:
  legal_limit = min(valid_map, valid_sign, valid_road_mark)
  user_margin = clamp(overspeed_tolerance, 0, policy_margin_max)
  operational_cap = min(legal_limit, legal_limit + user_margin)

  候補軌跡の速度プロファイル v(t) が operational_cap を超える場合は Fail。
```

### 横G・ジャーク評価

```text
External Evaluator は、PlannerやConverterの出力を次の物理量で検査する。

横G:
  a_y_plan = v_target^2 * kappa_target
  a_y_meas = ego_speed * ego_yaw_rate
  a_y_eval = max(|a_y_plan|, |a_y_meas|)

ジャーク:
  jerk_x = Δtarget_accel / Δt
  jerk_y = Δa_y_eval / Δt

判定:
  - a_y_eval > A_Y_MAX は Fail または強い減点
  - |jerk_x| > JERK_X_MAX は Fail または強い減点
  - |jerk_y| > JERK_Y_MAX は Fail または強い減点
  - 閾値は通常快適域と緊急回避域を分けて管理する
```

### 信号機・停止線・停止時車間評価

```text
信号機評価:
  - RED で停止線を越える候補は Fail
  - 該当方向のARROWがない右左折候補は Fail
  - YELLOW は停止可能距離内なら停止、停止不能なら交差点内急制動を避ける
  - UNKNOWNまたは低信頼度の場合は保守側（減速・停止準備）へ倒す

停止線停止精度:
  target_stop_x = stopline_x - stop_margin_m
  stop_position_error = planned_stop_x - target_stop_x
  |stop_position_error| > STOP_POS_TOL は Fail または強い減点

停止時車間:
  desired_stop_gap = max(min_gap_m, time_gap_s * ego_speed)
  actual_stop_gap = lead_vehicle_x - planned_stop_x
  actual_stop_gap < desired_stop_gap は Fail
```

---

## 9.3 外部評価器の分離理由

```text
外部評価器をニューラルネットから分離する理由:
  1. 監査可能性: ルールが明示的で、監査・検証しやすい
  2. 更新可能性: 法規・地域・ODDが変わっても、ニューラルネット再学習なしに更新可能
  3. 安全ケース証拠: 「評価器がチェックした」という証拠が残る
  4. フォールバック設計: 評価器は Planner が完全に失敗しても動作すべき

注意:
  - 外部評価器も完璧ではない（評価器自体のバグがありうる）
  - 評価器のロジックも Unit Test、シナリオテストが必要
  - 評価器は保守的すぎると「使えないシステム」になる（too conservative problem）
```

---

## 9.4 法規への対応方針

**本書は法規の法的解釈をしない。** ただし、設計として考慮すべき点を述べる。

### ISO 26262（機能安全）

```text
ISO 26262 の対象:
  - 電気・電子システムの機能安全
  - ASIL (Automotive Safety Integrity Level) A〜D

本設計との関係:
  - Neural Network は ISO 26262 の通常のアーキテクチャで直接ASIL認証できない
  - External Evaluator + Fallback Layer を ASIL-B or ASIL-D で設計する
  - Neural Network は QM (Quality Management) 扱いとし、評価器で包む
  - センサ冗長性（Tri-modal）は ASIL分解に貢献

実装指針:
  - ASIL-Dの安全モニタリングを、Plannerとは独立したCPU/MCUで動かす
  - Plannerの出力を安全監視レイヤーが常に検証する
  - 最終的な車両制御コマンドは安全監視レイヤーを通す
```

### SOTIF (ISO 21448)

```text
SOTIFの対象:
  - 「意図した機能が不十分である」ことによるリスク
  - センサの性能限界、モデルの性能限界

本設計との関係:
  - BEVの不確実性Headは「モデルが自信を持てない領域」を示す
  - ODD監視は「動作範囲外」を検出する
  - 未知場面の検出（OOD detection）が安全側の行動を促す

実装指針:
  - 不確実性が高い場面での速度低下
  - 初見シーンの記録とログ
  - ODD外検出時の速度低下 → MRM
```

### UNECE R157（ALKS）

```text
UNECE R157 の対象:
  - 自動車線維持システム（ALKS）
  - L3自動運転の国際基準

主要要件:
  - Minimum Risk Maneuver (MRM) の能力
  - ドライバー引き継ぎ要求
  - 最大速度 130km/h（高速道路限定）
  - 記録要件（Event Data Recorder）
  - 後続車への通知（ハザードランプ等）

本設計との関係:
  - MRM: Anytime Planning Level 3-4 が対応
  - 記録: ログ設計 (9.12) が対応
  - 速度: External Evaluator の速度チェックが対応
```

### 地域別の法規差分

| 地域 | 主要法規・基準 | 特徴 |
|---|---|---|
| EU | UNECE R157, EU AI Act | 型式認証必要、AI法の適用 |
| 日本 | 道路交通法改正, 国交省ガイドライン | L3は高速道路限定 |
| 米国 | FMVSS, NHTSA AV Guidance | 州法による差異が大きい |
| 中国 | GB/T 38186, 智能网联汽车政策 | 地域限定試験区 |

```text
対応方針:
  - Region / Policy コンディショニングをCondFormerに追加
  - DSL に region_code パラメータを追加
  - 速度制限・優先順位ルールを地域別テーブルで管理
```

---

## 9.5 Tri-modal センサ冗長性

```text
各センサが独立した物理原理で動作:
  - Camera: 光学
  - LiDAR: レーザー飛行時間
  - Radar: マイクロ波Doppler

一つの障害（例: 豪雨）でも:
  - Camera: 視界低下
  - LiDAR: 点群数減少
  - Radar: 悪天候に強い（継続動作）

→ Tri-modalで単一センサ障害に対する冗長性を持つ
```

### センサ冗長性の設計保証

```text
要件: 任意の1センサ障害で動作継続（制限速度付き）
検証: センサ欠損注入テスト（学習・実車両方）
ログ: 欠損センサと動作継続の記録
安全: 残存センサのみでの推定精度をODD条件として定義
```

---

## 9.6 モダリティゲート監視

モダリティゲートの出力を監視することで、センサ健全性を診断できる。

```text
監視する値:
  gate_camera: 平均ゲート値
  gate_lidar:  平均ゲート値
  gate_radar:  平均ゲート値

異常判定:
  gate < 0.1 が 5フレーム以上継続 → センサ品質低下警告
  gate < 0.05 → センサ障害疑い → ODD外フラグ

利用:
  - ODD判定
  - ログ記録
  - ダッシュボード表示（開発中）
```

---

## 9.7 BEV出力によるオンライン自己診断

BEV情報を自己診断に使う。

```text
診断チェック:
  - bev_drivable の自車位置クラスが NOT_DRIVABLE(0) または MARGINAL(2)
    → キャリブレーション異常 or 測位異常（通常走行時は自車位置が DRIVABLE(1) であるべき）

  - bev_uncertainty の自車周辺が高い
    → センサ品質低下 or モデル外の場面

  - bev_agent_occ の全面高占有
    → センサノイズまたはモデルの暴走

  - dynamic_risk_map が全面高リスク
    → 動体予測の異常

診断結果:
  - 異常フラグをODD Monitorに通知
  - 異常ログの記録
  - 重篤な場合は速度低下 or MRM
```

---

## 9.8 ODD（Operational Design Domain）の定義と監視

ODDは、システムが正常に動作することを想定した「動作条件の範囲」である。

### ODD定義例（L2+）

```text
道路種別: 高速道路、自動車専用道路
速度範囲: 0〜130 km/h
天候: 晴れ、小雨（降水量 < 10mm/h）
視程: 100m以上
時間帯: 昼夜（ただし濃霧除く）
地域: (地図が存在するエリア)
センサ: 全センサ正常（または1センサ障害まで許容）
測位: 周辺Map候補を検索できる粗い絶対位置精度
自己位置: 局所Lane Graph上のレーン所属・横方向オフセット・方位差が制御可能な精度
```

### ODD監視モジュール

```text
監視項目:
  - 粗い測位品質
  - 局所Lane Graph上の自己位置信頼度
  - センサ健全性（ゲート値）
  - BEV不確実性の統計値
  - Map候補とBEV観測の整合性スコア
  - 天候（センサから推定、またはオンライン天気情報）
  - 速度
  - 交差点幾何の複雑さ

ODD外検出時:
  - Level 1: 速度低下（20% 削減）
  - Level 2: 速度大幅低下（50% 削減）
  - Level 3: MRM（安全停車）
  - Level 4: ドライバー引き継ぎ要求
```

---

## 9.9 テスト単位の分解

```text
コンポーネントテスト:
  - SensorSyncCalib: タイムスタンプ精度テスト
  - CameraBEVEncoder: IoU精度（LiDARと比較）
  - LiDARBEVEncoder: 3D検出精度
  - RadarBEVEncoder: Doppler速度精度
  - ModalityGateFuser: センサ欠損時の動作
  - TemporalBEVMemory: ego motion warp精度
  - BEV Information Heads: 各HEADの精度指標
  - Lane Topology: 接続グラフ精度
  - Dynamic Object Branch: 動体検出・速度推定精度
  - Agent Future Predictor: minADE, minFDE, MR
  - VLA Planner: K候補軌跡のADE/FDE vs 人間軌跡
  - External Evaluator: 不安全軌跡の除外率
  - Trajectory Converter: 軌跡から舵角変換の精度

統合テスト:
  - End-to-end:センサ入力から制御指令まで（遅延、精度）
  - 複数センサ欠損での継続動作テスト
  - ODD外検出テスト

シナリオテスト:
  - 標準シナリオセット（第11章参照）
  - テールケースシナリオ
  - 地域別シナリオ
```

---

## 9.10 信頼度キャリブレーション

Plannerの confidence は「好ましさ」スコアであり、安全確率ではない。  
ただし、calibrationされた confidence は有用な情報になる。

```text
Calibrationチェック:
  - confidence = 0.9 の候補が「実際に人間軌跡に近い」割合
  - Reliability diagram (横軸: confidence, 縦軸: accuracy)
  - Perfect calibration では 45度線上に乗る

Calibration方法:
  - Temperature scaling (最も簡単)
  - Platt scaling
  - Isotonic regression

注意:
  - calibratedなconfidenceでも、安全確率として使ってはいけない
  - あくまで「External Evaluator が行う安全チェック後の選択基準」
```

---

## 9.11 説明性（Explainability）の設計

```text
レベル1: エンジニアリング向け
  - Attention visualization（BEV上の注意マップ）
  - Confidence vs. Evaluator score の対比
  - 各モジュールの中間出力

レベル2: ユーザー向け（将来）
  - T_scene_text（内部シーントークンのデコード）
  - 選択理由の簡易表示
  - 「なぜ今ここにいるか」の可視化

レベル3: 安全ケース向け
  - 事故・ニアミスのシーン再生
  - 各モジュールの動作状態の記録
  - 評価器が何をチェックして選択したか
```

---

## 9.12 ログ設計

```text
毎フレームのログ（Medium layer, 10-20Hz）:
  timestamp_ns
  ego_x, y, lat, lon, heading
  speed, yaw_rate, accel_x, accel_y, steer_angle
  
  sensor_health: {camera: [], lidar: float, radar: float}
  gate_values: {camera: float, lidar: float, radar: float}
  
  bev_uncertainty_max, bev_uncertainty_mean
  lane_mismatch_score
  ood_flags: {speed: bool, weather: bool, localization: bool, ...}
  
  n_candidates: int
  selected_candidate_id: int
  selected_reason: str
  all_candidate_pass: [bool * K]
  
  target_curvature, target_speed
  feasibility_ok
  fallback_active
  fallback_level: int
  
  processing_time_ms
  
イベントログ（条件発生時）:
  ODD外イベント
  センサ障害イベント
  Fallback発動イベント
  キャリブレーション要求イベント
  地図不整合イベント

Shadow Modeログ（追加フィールド）:
  human_trajectory: [T, D]
  system_trajectory: [T, D]
  trajectory_diff: float (ADE)
  system_candidate_human_chose: bool
```

---

## 9.13 製品化における開発ループ

```text
Stage 1: Offline Open-loop
  - ログデータでの性能評価
  - ADE/FDE, Collision Rate 等のメトリクス

Stage 2: Closed-loop Simulation
  - CARLA, nuPlan等のシミュレータでの検証

Stage 3: Shadow Mode
  - 実車でシステムを動かすが制御しない
  - システム出力と人間実際走行を比較

Stage 4: Limited Actuator Test
  - 速度・ODD制限下での実車制御試験

Stage 5: ODD限定Road Test
  - 定義したODD内での実走行試験

Stage 6: Regression & OTA
  - 定期的な性能確認
  - 問題があればOTA更新
```

---

## 9.14 章のまとめ

```text
本章で設計した要素:
  1. 製品として必要な中間表現の明示化
  2. K候補安全側選択とExternal Evaluator
  3. 外部評価器を分離する理由
  4. 法規対応（ISO 26262, SOTIF, UNECE R157）
  5. 地域別法規差分（EU/日本/米国/中国）
  6. Tri-modalセンサ冗長性
  7. モダリティゲート監視
  8. BEV出力による自己診断
  9. ODD定義と監視
  10. テスト単位の分解
  11. 信頼度キャリブレーション
  12. 説明性（3レベル）
  13. ログ設計
  14. 製品化開発ループ
```

次章では、実装モジュールとインターフェース設計の詳細を述べる。
