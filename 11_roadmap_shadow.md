# 第11章 実装ロードマップとShadow Mode検証

---

## 11.1 MVPビルド順序

最初から全機能を実装しようとすると失敗する確率が高い。最小実装から段階的に拡張する。

```text
Step 1: BEVFusion Backbone（Camera + LiDAR）
  - Camera BEV Encoder + LiDAR BEV Encoder
  - シンプルな結合（Gate なし）
  - nuScenesでのOccupancy/Detection評価

Step 2: BEV Information Heads
  - Static World heads（drivable, occupancy, lane）
  - BEV上での可視化・評価

Step 3: K-query Planner（Language なし）
  - 人間軌跡ラベルで学習
  - K=4から始める
  - ADE/FDE の評価

Step 4: External Evaluator（静的衝突チェックのみ）
  - 不安全軌跡の除外
  - 安全な候補選択の確認

Step 5: Radar Branch の追加
  - Radar BEV Encoder
  - Modality Gate
  - Radar による動体速度推定の改善を確認

Step 6: Language Conditioning の追加
  - DSLレベルの言語入力（右折、直進等）
  - CondFormerの学習
  - 言語条件付けによる軌跡変化を確認

Step 7: Internal Scene Tokenizer の追加
  - VLM Teacherによる蒸留学習
  - T_sceneの影響確認
```

---

## 11.2 製品指向の増分機能

```text
Phase A（基盤）:
  - BEVFusion warm start
  - BEV Heads
  - K-query Planner
  - Static Evaluator
  
Phase B（動的）:
  - Dynamic Object Branch
  - Agent Future Predictor
  - Dynamic Risk Map
  - Dynamic Evaluator
  
Phase C（言語）:
  - Language Conditioning (DSL)
  - Scene Tokenizer
  - CondFormer
  
Phase D（製品化）:
  - ODD Monitor
  - Fallback Manager
  - Log system
  - Shadow Mode
  
Phase E（拡張）:
  - Map-less degradation
  - Multi-region support
  - Fleet Learning pipeline
  - Continual Learning
```

---

## 11.3 8フェーズロードマップ

| フェーズ | 名称 | 目標 |
|---|---|---|
| Phase 0 | データ・ラベル検証 | 学習データの品質確認 |
| Phase 1 | オフライン Open-loop評価 | メトリクスと性能ベースライン確立 |
| Phase 2 | Log Replay + External Evaluator | 安全選択ロジックの検証 |
| Phase 3 | Closed-loop Simulation | シミュレータでの総合評価 |
| Phase 4 | Shadow Mode | 実車でのシステム出力収集 |
| Phase 5 | Limited Actuator Test | 制限条件下での実車制御 |
| Phase 6 | ODD限定Road Test | 定義したODD内の実走行 |
| Phase 7 | Regression + OTA | 継続的な性能監視と更新 |

---

## 11.4 Phase 0: データ・ラベル検証

```text
チェック項目:
  □ LiDAR点群の ego frame 変換が正しいか
  □ Camera キャリブレーションがBEVと一致しているか
  □ 人間軌跡ラベルの座標系が正しいか
  □ 速度プロファイルの計算が正しいか
  □ ラベルのタイムスタンプが一致しているか
  □ クラスバランスの確認（動体vs静体、シナリオ種別）
  □ データ品質フィルタリングの結果
  □ フリート地域分布の確認
  □ 悪天候・夜間データの割合確認
  □ 信号・交差点シーンの割合確認
  □ テールケースシーンの割合と内容確認
  □ 検証データの人手によるサンプルチェック（100件以上）
```

---

## 11.5 Phase 1: Offline Open-loop評価

```text
評価メトリクス:
  Planner:
    - minADE@K: K候補のうち最良候補のADE
    - minFDE@K: K候補のうち最良候補のFDE
    - MR (Miss Rate): 閾値以上外れた割合
    - Collision Rate (static): 静的占有との衝突率
    - Collision Rate (dynamic): 動体との衝突率
    - Comfort: |a_x|, |a_y| の平均・最大
    
  BEV Heads:
    - IoU (drivable, occupancy, lane)
    - BEV detection AP (agent)
    
  Agent Future:
    - minADE, minFDE, MR (per category)
    - NLL (Negative Log-Likelihood)

シナリオバケット別評価:
  - 直進（高速道路）
  - 直進（市街地）
  - 右折・左折
  - 車線変更
  - 駐車車両回避
  - 歩行者横断
  - 交差点
  - 悪天候
```

---

## 11.6 Phase 2: Log Replay + External Evaluator

```text
Log Replayとは:
  - 実走行ログを入力として、システムが出力する候補軌跡と評価結果を確認する
  - 実際には制御しない（ログをそのまま再生する）

チェック:
  - External Evaluatorが正しい候補を選択するか
  - 不安全なシーンでFallbackが発動するか
  - ODD外検出が正しく動くか
  - ログが正しく出力されるか
  
差分分析:
  - システム選択軌跡 vs 人間実際走行の比較
  - Evaluatorが「Fail」とした候補が本当に不安全か
  - 人間が実際に走った軌跡が候補に含まれているか
```

---

## 11.7 Phase 3: Closed-loop Simulation

```text
推奨シミュレータ:
  CARLA (Open-source):
    - Unreal Engineベースのリアルな3D環境
    - センサモデル（Camera, LiDAR, Radar）
    - Traffic Manager (他車両挙動)
    - Python API
    URL: https://github.com/carla-simulator/carla

  nuPlan (Waymo/nuScenes系):
    - 実走行ログベースのリアクティブシミュレーション
    - 計画の評価に強い
    - PDM-Closedなど参考実装がある
    URL: https://github.com/motional/nuplan-devkit

  SUMO:
    - 交通シミュレーション
    - CARLAとの連携可能
    URL: https://sumo.dlr.de

Open-loop vs Closed-loop の違い:
  Open-loop: 記録済みデータを再生、自車の行動を反映しない
  Closed-loop: システムの行動が環境に影響し、他車両も反応する
  
  重要: Open-loop評価が良くても Closed-loop で問題が出ることがある
  （他車両の反応、自分の行動が生む状況変化）
```

---

## 11.8 Phase 4: Shadow Mode

Shadow Modeは、実車でシステムを動かすが、制御は人間が行う状態で、システムの出力を収集・分析するフェーズである。

### Shadow Modeで確認すること

```text
□ システムの選択軌跡が人間の実際の走行と概ね一致するか
□ 人間が特別な操作をした場面でシステムがどう出力したか
□ 外部評価器が不必要に「Fail」を多発していないか
□ Fallbackが誤発動していないか
□ ODD外検出の精度
□ ログが正しく記録されているか
□ センサ健全性のモニタリングが機能しているか
□ 処理時間が目標以内か
□ 候補軌跡の多様性（K候補が似すぎていないか）
□ 言語指示への応答（DSL入力時の軌跡変化）
□ 悪天候・夜間でのBEV品質
□ 駐車車両回避・歩行者への反応
```

### Shadow Mode差分の分類

```text
Benign（良性）:
  - システムが人間より少し保守的に動いている
  - わずかな横位置の差
  → 品質向上の余地あり、特に緊急ではない

Suspicious（要調査）:
  - システムの軌跡が人間と大きく異なる
  - 評価器がほとんどの候補をFailにしている
  - 速度プロファイルが大きく異なる
  → 詳細ログを確認、モデルの改善検討

Critical（重大）:
  - システムが障害物に向かう軌跡を出力している
  - ODD外なのに検出できていない
  - Plannerが全候補をNaNで出力
  → 即座に原因究明、実車制御開始前に修正必須
```

---

## 11.9 実車制御開始前のゲート条件

```text
Shadow Modeで確認すべきゲート条件:
  □ minADE@K < 0.5m (高速道路シーン)
  □ Collision Rate (static) < 0.1%
  □ ODD外検出率 > 95%
  □ 処理時間 < 50ms (95パーセンタイル)
  □ Fallback誤発動率 < 5%
  □ Critical差分 = 0件 (直近1000km)
  □ センサ欠損テスト: 1センサ欠損での動作継続確認
  □ 速度制限: 試験速度でのConverterの正常動作確認
  □ External Evaluatorの単体テスト全通過
  □ Trajectory Converterの単体テスト全通過
```

---

## 11.10 Failure Rollback マッピング

問題が発生した場合、どのフェーズに戻るべきかを事前に定義する。

| 問題種別 | 戻るフェーズ |
|---|---|
| モデル精度の大幅低下 | Phase 1（Open-loop再評価） |
| Evaluatorの誤判定 | Phase 2（Log Replay） |
| シミュレーション環境との乖離 | Phase 3（Closed-loop修正） |
| Shadow Mode Critical差分 | Phase 4（Shadow Mode継続） |
| 実車での処理時間オーバー | Phase 5（軽量化後再試験） |
| ODD外での性能問題 | Phase 1（ODD定義見直し） |

---

## 11.11 リグレッションテスト

OTA更新後の性能確認のためのリグレッションテスト。

```text
テスト層:
  Layer 1: Offline metrics
    - nuScenesまたはカスタムデータセットでのメトリクス
    - 基準より悪化したらブロック

  Layer 2: Scenario test suite
    - 標準シナリオセット（100+シナリオ）の自動実行
    - 全通過が必須

  Layer 3: Shadow Mode comparison
    - 最新バージョンと旧バージョンのShadow Modeログ比較
    - 重要シーンで悪化していないか

  Layer 4: Real-vehicle smoke test
    - 特定ルートでの実車テスト
    - 基準Scenario setの通過
```

---

## 11.12 開発ループ（10ステップサイクル）

```text
1. 問題の特定（ログ・Shadow Mode分析）
2. 根本原因の特定（データ？アーキテクチャ？実装バグ？）
3. 修正方針の立案
4. 実装・学習
5. オフライン評価（Phase 1相当）
6. Log Replay確認（Phase 2相当）
7. シナリオテスト
8. Shadow Modeでの確認
9. リグレッションテスト
10. OTA更新 or データフィードバック
→ 1に戻る
```

---

## 11.13 シナリオカバレッジの定義

```text
シナリオの次元:
  道路種別: 高速道路 / 幹線道路 / 市街地 / 住宅地 / 駐車場
  交差点種別: T字 / 十字 / ロータリー / 右折専用 / なし
  天候: 晴れ / 曇り / 雨 / 霧 / 夜間 / 雪
  センサ状態: 全正常 / LiDAR障害 / カメラ一部障害 / Radar障害
  交通量: 低 / 中 / 高 / 渋滞
  周辺エージェント: なし / 前走車 / 並走車 / 歩行者 / 自転車 / 大型車
  特殊: 工事 / 駐車車両 / 落下物 / 右折待ち / 割り込み
  言語指示: なし / 右折 / 左折 / 車線変更 / 停止
```

---

## 11.14 Shadow Mode ログ分析ツール

Shadow Modeのログを効率的に分析するためのツール設計。

```python
# Shadow Mode差分分析（概略）
def analyze_shadow_diff(log_files, threshold_m=0.5):
    results = []
    for log in log_files:
        human_traj = log.human_trajectory
        system_traj = log.system_trajectory
        
        ade = compute_ade(system_traj, human_traj)
        
        if ade > threshold_m * 2:
            severity = "critical"
        elif ade > threshold_m:
            severity = "suspicious"
        else:
            severity = "benign"
        
        results.append({
            "timestamp": log.timestamp,
            "ade": ade,
            "severity": severity,
            "scenario_type": log.scenario_type,
            "fallback_active": log.fallback_active,
        })
    
    return results, summarize_by_scenario(results)
```

---

## 11.15 章のまとめ

```text
本章で設計した要素:
  1. MVP構築順序（7ステップ）
  2. 製品指向の増分機能（Phase A〜E）
  3. 8フェーズロードマップ
  4. Phase 0: データ・ラベル検証（12チェック項目）
  5. Phase 1: Offline Open-loop評価（メトリクス・シナリオ別）
  6. Phase 2: Log Replay + External Evaluator
  7. Phase 3: Closed-loop Simulation（CARLA, nuPlan）
  8. Phase 4: Shadow Mode（確認項目・差分分類）
  9. 実車制御開始前のゲート条件
  10. Failure Rollback マッピング
  11. リグレッションテスト（4層）
  12. 開発ループ（10ステップ）
  13. シナリオカバレッジ定義
  14. Shadow Modeログ分析ツール
```

次章では、データ収集・フリート学習・アノテーションパイプラインの設計を述べる（新規追加章）。
