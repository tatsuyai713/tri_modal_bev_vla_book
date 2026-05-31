# 第5章 Language Conditioning と VLA Planner

---

## 5.1 なぜ言語をPlannerへ入力するのか

言語指示を「会話インターフェース」としてだけ使うのではなく、「行動の条件付け」として使う。

```text
アプローチA（説明生成型）:
  - システムが行動した理由を言語で説明する
  - ユーザーに安心感を与えるが行動には影響しない
  - 後付け説明なので faithfulness が保証しにくい

アプローチB（行動条件付け型、本設計）:
  - 言語指示が Planning の入力に入る
  - 「右折してください」という指示が、実際に右折軌跡を生成する条件になる
  - 行動と言語が統合的に学習される
```

本設計はアプローチBを採用する。言語は行動の条件変数である。

---

## 5.2 外部言語と内部シーン言語の分離

言語を2種類に分ける。

```text
External Language (T_inst):
  - ユーザーまたはナビゲーションからの指示
  - 例: 「次の交差点を右折」「車線を変更しないで」
  - 入力: Text Encoder → T_inst [B, N_inst, D]

Internal Scene Language (T_scene):
  - システムが自律的に生成するシーンの要約
  - 例: 「右側に歩行者がいる」「前方に割り込み可能性」
  - 入力: BEV features → Scene Tokenizer → T_scene [B, N_scene, D]
```

この分離により、外部の指示が曖昧でもシーン理解は機能し、逆にシーンが単純でも外部指示が優先される。

---

## 5.3 Text Encoder と Command Encoder

### 3段階の言語エンコード

```text
レベル1: 識別子（最小コスト）
  - action_id: int (RIGHT_TURN=1, CHANGE_LANE=2, ...)
  - Embedding(action_id) → T_inst
  - 製品での推奨: ナビから構造化コマンドが取れる場合

レベル2: 構造化コマンド（DSL）
  - {action: "turn", direction: "right", distance_m: 100.0}
  - LinearProjection(DSL features) → T_inst
  - 製品での推奨: ナビ+音声認識の組み合わせ

レベル3: 自然文（最大柔軟性）
  - "次の交差点を右折してください"
  - Text Encoder (CLIP/T5) → T_inst
  - 使用場面: テスト・研究・将来の対話型UI
```

### 自然文処理の注意点

```text
自然文はNLU（自然言語理解）のエラーが発生しやすい。
製品では自然文を直接安全制御に使わず、
DSLへの変換を挟む（意図分類 + パラメータ抽出）。

例:
  Input: "あそこのコンビニで止まって"
  NLU: {action: "stop", location: "nearby_poi", poi_type: "convenience_store"}
  → DSL → Planner
```

---

## 5.4 CondFormerの設計

BEVトークン、外部言語（T_inst）、内部シーントークン（T_scene）を統合する。

```text
CondFormer:
  Input:
    - q_planner:   Plannerのquery tokens [B, K, D]
    - BEV tokens:  [B, H*W, D]
    - T_inst:      [B, N_inst, D]
    - T_scene:     [B, N_scene, D]

  Layer 1: Cross-attention over BEV tokens
  Layer 2: Cross-attention over T_inst
  Layer 3: Cross-attention over T_scene
  Layer 4: Self-attention (統合)
  Layer 5: Instruction Priority Gate

  Output:
    - conditioned query tokens [B, K, D]
```

### 指示優先ゲート

強い外部指示（右折、停止等）が確実に反映されるよう、T_instの重みを動的に増幅する。

```python
# 指示優先ゲート（概略）
inst_cls = T_inst[:, 0, :]  # CLS token
gate = torch.sigmoid(self.gate_linear(inst_cls))  # [B, 1]
gated_out = (1.0 + gate.unsqueeze(1)) * lang_cross_attn_out
```

---

## 5.5 K-query VLA Planner の設計

Transformer Decoder 構造で、K本の候補軌跡を並列に生成する。

```text
Input Tokens to Decoder:
  - q_planner:    初期クエリ [B, K, D] (学習可能パラメータ)
  - BEV tokens:   空間情報
  - Agent tokens: 動的エージェント
  - Future risk:  Dynamic Risk Map tokens
  - CondFormer:   言語・シーン条件
  - Ego context:  速度、yaw_rate等

Decoder (4〜6 layers):
  Each layer:
    - Self-attention over K queries
    - Cross-attention over BEV (static world)
    - Cross-attention over Agent future tokens
    - Cross-attention over Dynamic Risk tokens
    - Cross-attention over CondFormer output
    - FFN
```

### Planner出力

```text
trajectories: [B, K, T, D]
  - D = (x, y, yaw, v) または (x, y, heading, speed, curvature)
  - T = 8〜16 タイムステップ
  - K = 12〜24 候補

confidence: [B, K]
  - 学習された候補の「好ましさ」スコア
  - 安全確率ではない（外部評価器が安全を判断する）
```

### Confidenceの意味

```text
confidence ≠ safety probability
confidence = モデルが予測した「この候補がどれくらい人間軌跡に近いか」

→ 安全性は外部評価器が別途チェックする
→ confidence で安全を保証するような使い方はしない
```

---

## 5.6 候補軌跡の出力形式

```text
各候補軌跡の各点:
  t:           タイムスタンプ (秒, 現在からの相対)
  x:           前方変位 (m)
  y:           側方変位 (m, 左が正)
  yaw:         方位角変化 (rad)
  v:           目標速度 (m/s)

座標系:
  - 現在のegoフレーム基準
  - 各フレームで再計算（前フレームとの累積ではない）
  - これにより vehicle-agnostic になる
```

### 速度プロファイルの整合性

```text
候補軌跡の速度プロファイルは次の制約を満たすべき:
  - 各点間の加速度が物理的に可能な範囲
  - 停止が必要な位置（信号、停止線）では速度0に収束
  - カーブでの横加速度 a_y = v^2 * κ ≤ 3 m/s^2 程度（快適性）
  - 前走車との相対速度・車間距離が適切
  - 先読みレーンの advisory_speed_mps（LaneNodeから取得した曲率ベース推奨速度）を超えない
```

---

## 5.7 外部評価器との接続

```text
External Evaluator が K候補を評価する:

入力:
  - K candidate trajectories
  - Static World (drivable, occupancy, stopline, crosswalk)
  - Dynamic Risk Map
  - Agent Futures
  - ODD status
  - confidence [B, K]

チェック項目:
  1. Static collision check (不走行域・占有域との重なり)
  2. Dynamic collision check (Dynamic Risk Map との重なり)
  3. Lane violation check (走行可能域外)
  4. Speed limit check (規制速度 vs 速度プロファイル)
  5. Advisory speed check (LaneNode.advisory_speed_mps vs 速度プロファイル)
  6. Comfort check (軌跡曲率 vs レーン曲率プロファイルとの乖離, 加速度制限)
  7. Stopline check (停止線を超えて停車していないか)
  8. ODD check (測位精度、センサ健全性)

選択ロジック:
  - チェックを全て通過した候補の中から confidence が最大のものを選択
  - 全候補がチェックを通過しない場合 → MRM (Minimum Risk Maneuver)
```

---

## 5.8 軌跡として出力する理由（直接舵角でない）

```text
舵角（δ）を直接出力しない理由:

1. 車両非依存性
   - 同じ計画軌跡でも、車種ごとにステアリングゲイン・ホイールベースが違う
   - 軌跡を出せば、各車種のConverterが適切な舵角に変換できる

2. 外部評価器のチェック可能性
   - 舵角を評価器でチェックするより、軌跡の方が物理的意味が明確
   - 「この軌跡は衝突する」は言えるが「この舵角は衝突する」は評価が難しい

3. 人間軌跡との学習整合
   - 教師信号は人間の走行ログの軌跡
   - 舵角は車両モデルを通した間接変換が必要

4. Shadow Modeでの比較
   - Shadow Modeでシステム出力と人間実際走行を比較するとき、
     軌跡ベースの方が視覚的に理解しやすい
```

---

## 5.9 Planner の損失設計

```text
Loss = L_traj + L_conf + L_smooth + L_comfort + L_risk

L_traj: 人間軌跡との誤差（best-of-K）
  - 最も近いmode kのADE/FDE
  - Σ_t || pred_k* - gt ||_2

L_conf: best modeがhigh confidenceになるよう
  - BCE(confidence[k*], 1.0) for best k
  - BCE(confidence[k≠k*], 0.0) for others
  - または cross-entropy

L_smooth: 軌跡の滑らかさ
  - Σ_t || pred_t+1 - 2*pred_t + pred_t-1 ||_2 (2nd差分)

L_comfort: 横加速度・加速度の制限
  - ReLU(|a_y| - a_y_max)^2
  - ReLU(|a_x| - a_x_max)^2

L_risk: Dynamic Risk Mapとの衝突ペナルティ（soft）
  - Σ_t risk_t(pred_kt_x, pred_kt_y)
```

---

## 5.10 MVP構成

製品プロトタイプを短期間で作る場合の最小構成。

```text
Planner:
  - Layers: 4
  - K: 8〜12
  - T: 8〜10
  - D: 2 or 4 (x, y, yaw, v)

CondFormer:
  - Layers: 3
  - Language level: 2 (DSL)
  - T_scene: N_scene = 4

BEV tokens to Planner:
  - 全BEVではなく Route Corridor ± 3mの重要領域のみ
  - N_tokens = 500〜2000

外部評価器:
  - 静的衝突チェック（必須）
  - Dynamic Risk チェック（推奨）
  - 速度・快適性チェック
```

---

## 5.11 デバッグ出力とExplainability

```text
Debug outputs:
  T_scene_text:  内部シーントークンをデコードした可読テキスト
                 例: "pedestrian crossing ahead / right turn required"

  top_k_reasons: 上位3候補の選択理由
                 例: ["safe, matches route", "safe but suboptimal comfort",
                      "rejected: static collision"]

  attention_viz:  CondFormerの注意マップ（どのBEVセルに注目しているか）

  conf_vs_rank:   confidenceと評価器スコアの対比
```

これらのデバッグ出力は、開発時の解析と安全ケース作成の両方に使う。

---

## 5.12 言語指示のDomain-Specific Language（DSL）設計

製品での言語指示は、自然文をそのまま使うのではなく、構造化コマンド（DSL）を通じて処理する。

### DSLの仕様例

```json
{
  "command_type": "navigation",
  "action": "turn",
  "direction": "right",
  "trigger": {
    "type": "intersection",
    "distance_m": 150.0,
    "confidence": 0.92
  },
  "constraints": {
    "max_speed_kmh": 30,
    "caution_level": "normal"
  }
}
```

### DSLの利点

```text
1. 曖昧さの排除
   - 自然文「右折してください」は、どこで右折するのかが曖昧
   - DSLでは位置・条件を明示できる

2. 安全制約の付与
   - DSLで max_speed や caution_level を制御側から設定できる

3. 検証可能性
   - DSLは構文的に正しいかどうかをチェックできる
   - 自然文にはこれができない

4. 多言語対応
   - 日本語・英語・中国語など、自然文は言語ごとに処理が必要
   - DSLは言語非依存
```

### NLU→DSL変換

```text
User: "あそこのコンビニで右に曲がって"
    ↓ NLU (Intent Detection + Entity Extraction)
DSL: {
  "command_type": "navigation",
  "action": "turn",
  "direction": "right",
  "trigger": {
    "type": "poi",
    "poi_type": "convenience_store",
    "distance_m": 50.0
  }
}
    ↓ Level-2 Command Encoder
T_inst → Planner
```

---

## 5.13 マルチステップ計画と階層的Planner

本章の設計は1ステップの軌跡生成だが、長距離計画は階層化するとよい。

```text
Global Planner (Route level):
  - 目的地までのルートを計画（数km単位）
  - 入力: 粗いGPS位置 + 地図候補 + 目的地
  - 出力: route waypoints, road segment candidates

Tactical Planner (100m単位):
  - 本章のVLA Planner
  - 入力: BEV world + language + local route + local lane topology
  - 出力: K候補軌跡（数秒先）

Reactive Layer (即時回避):
  - 急な障害物への反応
  - 外部評価器とConverterが担う
  - または専用のAEB/回避層
```

本設計のVLA PlannerはTactical Plannerに相当する。  
Global Plannerからの道路セグメント候補はLane Topology Branchで現在観測と照合され、局所Lane Graphとtarget_lane_sequenceとしてCondFormerへ入力される。
Reactive LayerはExternal EvaluatorとConverterが担う。

---

## 5.14 章のまとめ

```text
本章で設計した要素:
  1. 言語を行動条件付けとして使う方針
  2. 外部言語（T_inst）と内部シーン言語（T_scene）の分離
  3. 3レベルの言語エンコード（識別子/DSL/自然文）
  4. CondFormer（4〜5層、指示優先ゲート付き）
  5. K-query VLA Planner Decoder
  6. 候補軌跡の出力形式（速度プロファイル含む）
  7. External Evaluator との接続
  8. 軌跡として出力する理由
  9. Planner損失設計（5項目）
  10. MVP構成
  11. DSL設計と NLU→DSL 変換
  12. 階層的Planner（Global/Tactical/Reactive）
```

次章では、人間軌跡教師とレーンセンター非依存Planningの設計を詳述する。
