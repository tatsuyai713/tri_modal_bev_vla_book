# 第7章 選択軌跡から目標舵角への変換

---

## 7.1 Converterの役割

Trajectory-to-Steering Converter（以下、Converter）は、外部評価器が選択した目標軌跡を、実際の車両制御指令（目標曲率、目標速度、舵角）に変換する決定論的な変換器である。

```text
Converter の役割:
  - 軌跡 → 目標曲率・目標速度
  - 車両モデル（ホイールベース、ステアリングレシオ等）を知る唯一のモジュール
  - ルールベース・決定論的（学習パラメータなし）
  - Plannerとは独立して単体テスト可能
```

---

## 7.2 従来のトラッキングコントローラとの違い

```text
従来のトラッキングコントローラ:
  - 目標軌跡との偏差（横方向誤差、ヘディング誤差）を最小化
  - 例: PID制御、Stanley Controller, LQR
  - 軌跡を「追いかける」構造

本設計のConverter:
  - 軌跡を見て、その先の目標曲率を計算する（先読み型）
  - 「追いかける」のではなく「先を見て準備する」
  - この区別は、高速走行時と曲線接近時に顕著に現れる
```

---

## 7.3 入出力インターフェース

```text
入力:
  selected_trajectory:  [T, D]  (外部評価器が選択した1本の軌跡)
  ego_state:            速度、yaw_rate、steer_angle、timestamp
  vehicle_params:       wheelbase, steer_ratio, steer_range, ...

出力:
  target_curvature:     float32, 1/m (曲率)
  target_speed:         float32, m/s
  target_accel:         float32, m/s^2
  speed_phase:          enum (ACCEL / CRUISE / DECEL)
  target_steer_angle:   float32, rad (optional, vehicle specific)
  limit_steer:          bool (舵角制限フラグ)
  limit_accel:          bool (加速度制限フラグ)
  feasibility_ok:       bool (Plannerへのフィードバック)
  fallback_active:      bool
```

---

## 7.4 処理の流れ（1サイクル）

```python
def control_cycle(selected_traj, ego_state, vehicle_params):
    """1制御サイクル処理"""
    
    # Step 1: 軌跡の妥当性チェック
    if not validate_trajectory(selected_traj):
        return fallback_command()
    
    # Step 2: タイムスタンプ補正
    # 処理遅れによるegoのdrift分をwarp
    aligned_traj = compensate_latency(selected_traj, ego_state)
    
    # Step 3: 先読み距離の計算
    lookahead_dist = calc_lookahead(ego_state.speed)
    
    # Step 4: 先読み点の取得
    lookahead_point = get_point_at_distance(aligned_traj, lookahead_dist)
    
    # Step 5: 曲率の推定
    kappa = estimate_curvature(aligned_traj, method='pure_pursuit')
    
    # Step 6: 曲率の平滑化
    kappa_smooth = alpha_filter(kappa, prev_kappa, alpha=0.3)
    
    # Step 7: 横加速度の制限
    # 計画値: a_y_plan = v^2 * kappa、実測寄り: a_y_meas = v * yaw_rate
    a_y_plan = ego_state.speed**2 * kappa_smooth
    a_y_meas = ego_state.speed * ego_state.yaw_rate
    if max(abs(a_y_plan), abs(a_y_meas)) > A_Y_MAX:
      kappa_smooth = sign(kappa_smooth) * A_Y_MAX / max(ego_state.speed**2, 0.01)
    
    # Step 8: 二輪モデルで舵角に変換
    steer_angle = bicycle_model_to_steer(kappa_smooth, vehicle_params)
    
    # Step 9: 舵角・レートリミット
    steer_angle = apply_steer_limit(steer_angle, vehicle_params.steer_range)
    steer_angle = apply_rate_limit(steer_angle, prev_steer, vehicle_params.steer_rate_max)
    
    # Step 10: ログ
    log_control_cycle(kappa_smooth, steer_angle, ego_state, ...)
    
    speed_cmd = resolve_target_speed(aligned_traj, ego_state, speed_constraints)
    accel_cmd = resolve_target_accel(aligned_traj, ego_state, speed_constraints)
    accel_cmd = apply_jerk_limit(accel_cmd, prev_accel, JERK_MAX)

    return ControlCommand(
        target_curvature=kappa_smooth,
        target_speed=speed_cmd.target_speed,
        target_accel=accel_cmd.target_accel,
        speed_phase=accel_cmd.phase,
        target_steer_angle=steer_angle,
        feasibility_ok=True
    )
```

---

## 7.5 先読み距離（Lookahead Distance）の計算

先読み距離は速度依存で決める。

```text
lookahead_dist = speed * lookahead_time + lookahead_min

例:
  lookahead_time = 0.5s
  lookahead_min  = 3.0m
  speed = 10 m/s → lookahead = 10 * 0.5 + 3.0 = 8.0m
  speed = 30 m/s → lookahead = 30 * 0.5 + 3.0 = 18.0m

理由:
  - 高速走行時は早めに対応が必要（先を見る）
  - 低速走行時は近い点を見ればよい
  - 停車中でも最小距離を保つ（0除算防止）
```

---

## 7.6 曲率推定の方法

### Pure Pursuit 法（最も一般的）

```python
# Pure Pursuit による目標曲率
dx = lookahead_point.x  # 先読み点のx（前方）
dy = lookahead_point.y  # 先読み点のy（横方向）
ld = sqrt(dx**2 + dy**2)
kappa = 2.0 * dy / ld**2
```

### 3点円弧法

```python
# 現在点, 中間点, 先読み点の3点で円を当てはめる
kappa = circle_curvature(p0=(0,0), p1=mid_point, p2=lookahead_point)
```

### 多項式フィット法

```python
# 軌跡をy = a*x^2 + b*x + c にフィット → 曲率計算
coeffs = np.polyfit(traj_x, traj_y, 2)
a = coeffs[0]
kappa = 2.0 * a  # 近似曲率
```

### 推奨

```text
低速（〜10 m/s）: Pure Pursuit が安定
中速（10〜20 m/s）: 3点円弧法 or Pure Pursuit
高速（20 m/s〜）: 多項式フィット（より滑らか）

LaneNode.curvature_profile によるサニティチェック:
  Converter が軌跡から計算した kappa と、自車が走行中の LaneNode.curvature_profile
  の値を比較する。乖離が大きい場合は以下のどちらかが疑われる:
    - センサ誤認識によるBEV上の位置ずれ
    - HD Map / Lane Topology の推定誤り
  乖離閾値の目安: |κ_traj - κ_lane| > 0.05 [1/m] をログに記録し、
  > 0.15 [1/m] が継続する場合は bev_uncertainty を引き上げる。
```

---

## 7.7 二輪モデルによる舵角変換

曲率から舵角への変換は二輪モデルで行う。

```text
二輪モデル:
  steer_angle = arctan(wheelbase * kappa)

ここで:
  wheelbase: 前後輪軸距離 (m) (例: 2.8m)
  kappa: 目標曲率 (1/m)
  steer_angle: 前輪舵角 (rad)

EPS（電動パワーステアリング）入力への変換:
  steer_input = steer_angle * steer_ratio + steer_offset

ここで:
  steer_ratio: ステアリングレシオ (例: 14.0)
  steer_offset: センター補正
```

### ステアリングキャリブレーション

```text
実車では、「直進時に steer_angle = 0」とならない場合がある（センターズレ）。
定期的にセンター補正値を測定し、キャリブレーションテーブルを更新する。
```

---

## 7.8 マルチレート設計

Plannerが10〜20Hzで動くのに対し、制御出力は50〜100Hzが求められる。

```text
Multi-rate design:
  Planner:    10-20 Hz → selected trajectory をバッファに保存
  Converter:  50-100 Hz → バッファの軌跡を使って毎サイクル制御計算

Plannerが新しい軌跡を出すまでの間:
  - 最後の選択軌跡をEgo motion でwarpして使用
  - Warp: 前フレームの軌跡を現在のEgo位置に合わせて変換
  - 最大100〜200ms は古い軌跡でOK（物理的な慣性）
```

---

## 7.9 軌跡の妥当性チェック

Plannerが異常な軌跡を出した場合のためのチェック。

```text
チェック項目:
  1. Nan / Inf がないか
  2. 軌跡の長さが十分か（最低3点）
  3. 最初の点が自車近傍か（xy < 1.0m）
  4. 連続する2点間の距離が物理的に可能か（速度 < v_max）
  5. 急激な方向転換がないか（yaw change / step < π/4）
  6. タイムスタンプの単調増加

異常検知時:
  - fallback_command を返す
  - fallback_active = True
  - Plannerへフィードバック
  - ログ記録
```

---

## 7.10 Feasibility フィードバック

Converterが「この軌跡は実現できない」と判断した場合、Plannerへフィードバックする。

```text
フィードバック信号:
  feasibility_ok:      bool
  infeasible_reason:   str  (例: "steer_rate_exceeded", "accel_too_large")
  feasibility_margin:  float (どのくらいオーバーしているか)

Plannerへの利用:
  - feasibility_ok = False の候補は学習時のペナルティを上げる
  - External Evaluator の次回選択で考慮する
```

---

## 7.11 逆車両モデルのオプション

二輪モデルが近似的すぎる場合、より正確な逆車両モデルを使う。

```text
オプション1: ルックアップテーブル
  - (speed, curvature) → steer_angle のテーブルを実験で作成
  - 非線形特性を含められる
  - 温度・タイヤ摩耗で変化するため定期更新が必要

オプション2: 二輪モデル + 補正（本設計の推奨）
  - 基本は二輪モデル
  - 残差（実際のyaw_rateとのズレ）を補正項として加える
  - Kalman Filter等でオンライン推定

オプション3: 小型ニューラルネット
  - (speed, kappa) → steer_angle の変換を学習
  - 実車走行データで学習
  - 解釈性が落ちるため、製品での採用は慎重に
```

---

## 7.12 縦方向制御（速度計画）との統合

本章のConverterは主に横方向（舵角）を扱っているが、縦方向制御とも統合する。

```text
縦方向制御入力:
  target_speed: Plannerが出した速度プロファイルから取得
  target_accel: 速度プロファイルの差分

縦方向制御出力:
  throttle_cmd: アクセル開度
  brake_cmd:    ブレーキ圧
  target_torque: トルク（電動車の場合）

前走車追従:
  target_speed = min(planner_speed, ACC_target_speed)
  ACC_target_speed: レーダーベースの前走車速度制限

信号対応:
  stopline_dist: 停止線までの距離 (BEVから)
  target_speed = v_profile_for_stop(stopline_dist, current_speed)
```

### 速度制約の優先順位（ルールベース）

```text
入力ソース:
  1) 法規速度（map/sign/road_mark統合）
  2) ユーザーGUI許容値（overspeed tolerance）
  3) 快適性・車両限界（a_x, a_y, jerk）
  4) 先行車追従制約（ACC）

決定手順:
  base_limit = min(valid_map_limit, valid_sign_limit, valid_road_mark_limit)
  user_margin = clamp(user_overspeed_tolerance, 0, policy_margin_max)
  legal_cap = min(base_limit, base_limit + user_margin)
  comfort_cap = speed_from_curvature(kappa, a_y_max)
  follow_cap = acc_following_cap(lead_vehicle_state)
  final_target_speed = min(planner_speed, legal_cap, comfort_cap, follow_cap)

  target_accelは final_target_speed への収束率から決定し、
  phaseは sign(target_accel) により ACCEL / CRUISE / DECEL を出力する。
```

### 横G・ヨーレート・ジャーク制約

最新の学習ベースPlannerでも、最終的な車両入力は物理量で監視する。Converterでは、軌跡由来の曲率だけでなく、実車状態のヨーレートも使って横Gを二重に評価する。

```text
横加速度（横G）:
  a_y_plan = v_target^2 * kappa_target
  a_y_meas = ego_speed * ego_yaw_rate
  a_y_eval = max(|a_y_plan|, |a_y_meas|)

制約:
  a_y_eval <= A_Y_MAX
  例: 快適域 2.0〜3.0 m/s^2、緊急回避時のみ上限を別管理
```

```text
縦ジャーク:
  jerk_x = (target_accel - prev_target_accel) / dt

横ジャーク:
  jerk_y = (a_y_eval - prev_a_y_eval) / dt

制約:
  |jerk_x| <= JERK_X_MAX
  |jerk_y| <= JERK_Y_MAX

処理:
  - target_accel を jerk limit で平滑化
  - kappa / steer_angle を rate limit で平滑化
  - 制約超過が残る場合は target_speed を下げる
  - それでも不可能なら feasibility_ok=False とし、External Evaluatorへ戻す
```

```python
def apply_dynamics_constraints(kappa, target_speed, target_accel, ego_state, dt):
    a_y_plan = target_speed**2 * kappa
    a_y_meas = ego_state.speed * ego_state.yaw_rate
    a_y_eval = max(abs(a_y_plan), abs(a_y_meas))

    if a_y_eval > A_Y_MAX:
        target_speed = min(target_speed, sqrt(A_Y_MAX / max(abs(kappa), 1e-4)))

    target_accel = clamp_rate(
        target_accel,
        prev_value=prev_target_accel,
        max_rate=JERK_X_MAX,
        dt=dt,
    )

    jerk_y = (a_y_eval - prev_a_y_eval) / dt
    if abs(jerk_y) > JERK_Y_MAX:
        kappa = clamp_rate(kappa, prev_kappa, KAPPA_RATE_MAX, dt)

    return kappa, target_speed, target_accel
```

---

## 7.13 ログするべき値

```text
制御サイクルごとにログ:
  timestamp
  selected_traj_id
  lookahead_dist
  lookahead_point_x, y
  raw_kappa
  smoothed_kappa
  target_steer_angle
  actual_steer_angle (EPSフィードバック)
  steer_error = target - actual
  target_speed
  target_accel
  speed_phase
  actual_speed
  a_y = speed^2 * kappa
  a_y_meas = speed * yaw_rate
  jerk_x
  jerk_y
  feasibility_ok
  fallback_active
  infeasible_reason (if any)
  processing_time_ms
```

---

## 7.14 章のまとめ

```text
本章で設計した要素:
  1. Converterの役割と従来コントローラとの違い
  2. 入出力インターフェース
  3. 1サイクルの処理フロー（10ステップ）
  4. 先読み距離の速度依存計算
  5. 曲率推定の3方法（Pure Pursuit / 3点円弧 / 多項式）
  6. 二輪モデルによる舵角変換
  7. マルチレート設計（Planner 10-20Hz, Converter 50-100Hz）
  8. 軌跡妥当性チェック
  9. Feasibility フィードバック
  10. 逆車両モデルのオプション
  11. 縦方向制御との統合
```

次章では、実車適用のための軽量化とリアルタイム設計を詳述する。
