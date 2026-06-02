# 第15章 MiddlewareとOS ―― 研究スタックから製品スタックまで

---

## 15.1 自動運転ソフトウェアスタックにおけるMiddlewareの役割

自動運転システムは、センサドライバから機械学習推論エンジン、計画・制御モジュール、ログ収集、OTAアップデートに至るまで、多数の異種プロセスが協調して動作しなければならない。**Middleware（MW）**は、これらのプロセス間で**データ交換・タイミング管理・障害隔離**を担うソフトウェア基盤である。

```text
MW が提供する主要機能:
  - IPC (Inter-Process Communication): プロセス/ノード間の高速データ転送
  - Publish-Subscribe / Service呼び出し: 非同期・同期通信パターン
  - 時刻同期: センサタイムスタンプとの整合
  - QoS管理: 帯域・レイテンシ・信頼性のポリシー管理
  - ライフサイクル管理: プロセスの起動・停止・再起動の制御
  - 障害隔離: あるモジュールのクラッシュが他を巻き込まない設計
  - 診断・ログ: 実行時の監視とトレース
  - OTA対応: ソフトウェアの安全なオンライン更新

自動運転でMWを正しく選ばないと:
  - センサデータの遅延・欠落が計画モジュールに伝わらない
  - 学習モデルとルールベースモジュールの時刻不整合が生じる
  - 1つのモジュールのバグが全体をクラッシュさせる
  - ISO 26262 の機能安全要件を満たせない
```

---

## 15.2 研究・プロトタイプ向けスタック: Linux + ROS 2

研究・大学・スタートアップにおける自動運転プロトタイプ開発では、**Linux（Ubuntu）+ ROS 2** の組み合わせが事実上の標準となっている。

### Linux ベースOS

```text
研究向け推奨ディストリビューション:
  - Ubuntu 22.04 LTS (Jammy): ROS 2 Humble 対応、2027年まで保守
  - Ubuntu 24.04 LTS (Noble): ROS 2 Jazzy 対応、2029年まで保守（最新推奨）
  - Yocto Project: 組み込み向け、カスタムLinuxイメージ構築（セミプロダクション向け）

特徴:
  - 豊富なデバイスドライバ（Velodyne LiDAR, FLIR カメラ, GNSS等）
  - CUDA / TensorRT との親和性（NVIDIA GPU推論）
  - コミュニティとOSS資産が充実
  - PREEMPT_RT パッチでリアルタイム性能を向上可能（50ms以下のレイテンシ）

PREEMPT_RT の考慮:
  - カーネルをPREEMPT_RTパッチでビルドすると、割り込みレイテンシを
    数100μs以下に低減できる
  - ただし、真のHard RT (< 1 ms 保証) は Linux では難しい
  - 制御出力が50ms内に収まればUbuntu+PREEMPT_RTで十分な場合が多い
```

### ROS 2 アーキテクチャ概要

ROS 2（Robot Operating System 2）は分散ノードベースのMiddlewareフレームワークである。ROS1 の後継として、DDS を通信基盤に採用した完全なリデザイン版である。

```text
ROS 2 の基本概念:

  Node:
    - 単一のプロセス単位（例: カメラドライバ、BEV推論器、Converter）
    - 複数 Node を 1 プロセスで動かす「Component」も可能（ゼロコピーIPC）

  Topic:
    - 非同期の Publish-Subscribe 通信
    - 例: /camera/image_raw, /bev/occupancy, /ego/trajectory
    - QoS プロファイルで信頼性・バッファを調整

  Service:
    - 同期のリクエスト-レスポンス通信
    - 例: /map_server/load_map, /planner/reset

  Action:
    - 長時間処理向け（キャンセル可能・フィードバック付き）
    - 例: /navigation/go_to_goal

  TF2 (Transform Framework):
    - 座標変換ツリー（センサフレーム間・ego基準変換）
    - ros2_tracing との組み合わせで座標変換のレイテンシをプロファイル可能

  Parameter Server:
    - ノードのパラメータ（閾値、重みファイルパス等）の動的変更
    - YAMLファイルでのパラメータ管理
```

### DDS（Data Distribution Service）レイヤー

```text
ROS 2 はDDS をトランスポート層として使用する。
主要な実装:

  CycloneDDS (Eclipse Foundation):
    - オープンソース、高パフォーマンス
    - ROS 2 Humble/Jazzy のデフォルト実装（humble 以降）
    - AUTOSAR Adaptive Platform と組み合わせ可能

  FastDDS (eProsima):
    - 機能豊富（SHM, Zero-Copy, Security）
    - 大規模システムでの実績
    - ROS 2 Foxy まではデフォルト実装

  RTI Connext DDS:
    - 商用ライセンス
    - 航空・医療・自動車向け認証実績
    - AUTOSAR AP の参照実装の1つ

  Zenoh (Eclipse):
    - 新世代プロトコル、DDS より低オーバーヘッド
    - クラウド/エッジ/デバイス統一通信
    - ROS 2 Zenoh plugin で ROS 2 エコシステムと統合可能
    - 2025年現在、研究・OSSコミュニティで急速に普及中

QoS プロファイルの使い分け:
  センサデータ（高頻度）:
    reliability: BEST_EFFORT
    history: KEEP_LAST(1)
    → 最新フレームのみが重要、古いフレームは破棄

  安全クリティカルデータ（コマンド等）:
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
    → 受信確認付き、接続前のデータも受け取れる

  地図・設定（低頻度・変化なし）:
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
    history: KEEP_ALL
```

### ゼロコピーIPC（Intra-Process Communication）

```text
同一マシン上のノード間では、DDS のシリアライズを回避し
共有メモリでデータを渡すことでレイテンシを大幅削減できる。

ROS 2 Intra-Process Communication:
  - Publisher と Subscription が同一プロセス内にある場合、
    shared_ptr のポインタ渡しで実質ゼロコピー
  - Component Container でノードを1プロセスに束ねて使用

iceoryx2 (Eclipse):
  - 共有メモリベースのゼロコピーIPCライブラリ
  - ROS 2 rmw_iceoryx plugin で ROS 2 と統合
  - 8K カメラや LiDAR の大きなバッファを効率転送する場合に有効
  - レイテンシ: ~1μs オーダー（vs DDS UDP ~100μs）
```

---

## 15.3 ROS 2 開発ツールチェーン

```text
ビルドシステム:
  colcon:
    - ROS 2 の標準ビルドツール
    - $ colcon build --symlink-install --packages-select my_planner
    - $ colcon test  でユニットテスト実行

  ament_cmake / ament_python:
    - CMake / Python パッケージ宣言形式
    - 依存関係は package.xml に記述

起動・設定:
  launch ファイル:
    - Python ベースの launch.py で複数ノードを一括起動
    - 条件分岐・パラメータ渡しが柔軟
    - $ ros2 launch my_stack full_system.launch.py

  composable nodes:
    - LoadComposableNodes アクションで同一プロセス内にロード
    - メモリフットプリントを削減、IPC レイテンシを最小化

デバッグ:
  $ ros2 topic echo /bev/trajectory         # トピックの内容を逐次表示
  $ ros2 topic hz /camera/image_raw          # 受信レート計測
  $ ros2 topic bw /lidar/points              # 帯域計測
  $ ros2 node list / ros2 node info          # ノード一覧と接続情報
  $ ros2 doctor                              # 診断ツール

ライフサイクル管理:
  - Lifecycle Node: Configure -> Activate -> Deactivate -> Cleanup の状態遷移
  - 起動シーケンスの制御（センサ前にロケータを起動等）
  - 異常時の安全な停止シーケンス
```

---

## 15.4 ROS 2 可視化ツールの最大限の活用

自動運転の開発・デバッグには、多様なセンサ・推論結果を同時に可視化するツールが不可欠である。

### RViz2

```text
ROS 2 の標準 3D 可視化ツール。

主な Display タイプ:
  PointCloud2:
    - LiDAR 点群の表示（高さ/反射強度などでカラーリング）
    - Voxel フィルタリングで重い点群も快適に表示

  MarkerArray:
    - 検出物体の 3D バウンディングボックス、速度矢印
    - BEV Grid の可視化（占有率ヒートマップ）

  Path / PoseArray:
    - 候補軌跡 K 本の可視化（色分けでスコア表示）
    - 選択軌跡の強調表示

  Image / CompressedImage:
    - カメラ前方映像とオーバーレイ（検出結果）

  TF2:
    - 座標フレームツリーのリアルタイム可視化
    - センサキャリブレーション確認に必須

カスタムプラグイン:
  - rviz_common を継承したカスタム Display を Python/C++ で作成可能
  - BEV グリッドを専用のテクスチャ描画で高速表示
  - 候補軌跡のコンフィデンス値をヒートマップで重ねる

設定の保存:
  - .rviz 設定ファイルでパネル・表示設定を保存・共有
  - launch ファイルから RViz2 を引数付きで起動
```

### Foxglove Studio

```text
ブラウザベースの次世代可視化プラットフォーム。
2023年以降、研究・プロト向け開発で急速に普及。

特徴:
  - Web ブラウザまたはデスクトップアプリとして動作
  - WebSocket で ROS 2 に接続 (rosbridge / foxglove-bridge)
  - MCAP ファイルのオフライン再生に最適
  - 複数パネルの自由なレイアウト配置

主要パネル:
  3D パネル:
    - PointCloud、MarkerArray、TF、Image を統合表示
    - フレームレートとレイテンシが RViz2 より良好な場合あり
  
  Plot パネル:
    - 速度、加速度、ジャーク、舵角の時系列グラフ
    - X 軸をメッセージ時刻 or 距離に設定可能
  
  Image パネル:
    - カメラ映像 + 検出結果オーバーレイ
  
  Raw Messages パネル:
    - トピックの生データを JSON 形式で確認

MCAP 形式:
  - Foxglove が主導する次世代ロボティクスログフォーマット
  - チャンク単位のインデックス → 任意時刻への高速シーク
  - rosbag2 より大きなファイルでのアクセスが高速
  - ROS 2 メッセージ以外（カスタム JSON 等）も格納可能
  - Foxglove SDK (Python / TypeScript) で変換・解析

チーム連携:
  - ブラウザで動くため、共有 URL で遠隔レビュー可能
  - Foxglove Data Platform (クラウド): フリート録画のオンライン管理

注記 (2024年以降):
  - Foxglove Studio は 2023 年頃にクローズドソース化・商用化方向に転換し、
    無料ティアに制限が加わった
  - 完全 OSS の代替として Lichtblick が急速に普及している
```

### Lichtblick (完全 OSS の Foxglove フォーク)

```text
Foxglove Studio の OSS 部分からフォークした完全オープンソースの可視化ツール。
ライセンス: Mozilla Public License v2.0 (MPL 2.0)
GitHub: https://github.com/lichtblick-suite/lichtblick

背景:
  - Foxglove が 2023 年頃にプロプライエタリ化したのを受け、
    完全 OSS を維持したいコミュニティがフォークして開発を継続
  - 2025 年時点で v1.25+ に到達、96 名のコントリビュータ、活発にメンテ中
  - Docker / Web / デスクトップアプリ (Linux / Windows / macOS) で動作

特徴:
  - Foxglove Studio とほぼ同一の UI・パネル構成を継承
    (3D / Plot / Image / Raw Messages / Map 等)
  - MCAP ネイティブ対応 (チャンク単位インデックスで高速シーク)
  - WebSocket で ROS 2 に接続 (foxglove-bridge / rosbridge)
  - TypeScript + React でカスタムパネルを自由に拡張可能
  - クラウドサービス不要・ライセンスコストゼロ

起動方法:
  # Docker で即時起動（インストール不要）
  docker run --rm -p 8080:8080 \
    ghcr.io/lichtblick-suite/lichtblick:latest
  # -> ブラウザで http://localhost:8080 を開く

  # ソースからビルド
  git clone https://github.com/lichtblick-suite/lichtblick.git
  corepack enable && yarn install
  yarn run web:serve          # Web アプリを起動
  yarn desktop:serve          # デスクトップアプリ (Electron)

Lichtblick と Foxglove の使い分け:
  Lichtblick を選ぶ場面:
    - クラウド連携・商用プランが不要な研究・開発環境
    - ライセンスや安全保障上の理由から完全 OSS が必須
    - MPL 2.0 でカスタムパネルをコミュニティに公開したい
    - 予算制約のあるスタートアップ・研究機関

  Foxglove を選ぶ場面:
    - Foxglove Data Platform によるフリート録画クラウド管理が必要
    - 製品開発において商用サポート付きライセンスが求められる
```

### PlotJuggler

```text
信号解析特化のタイムシリーズプロッタ。

主な用途:
  - 速度・加速度・ジャーク・舵角のリアルタイムプロット
  - ROS 2 トピックを複数同時プロット
  - FFT パネルで振動成分分析（サスペンション・EPS 異音分析）
  - 信号間の差分・比率を計算して重ね合わせ

ROS 2 連携:
  - ros2 plugin をビルドして ROS 2 トピックを直接受信
  - rosbag2 / MCAP ファイルの読み込みにも対応

典型的な使い方:
  - a_y_plan と a_y_meas を重ねて横G追跡誤差を分析
  - jerk_x の正規分布を確認してジャーク制御を調整
  - stopline_distance と v_target の相関を見て停止プロファイルを評価
```

### rqt_graph と rqt_console

```text
rqt_graph:
  - ノード/トピック/サービスの依存関係グラフを自動生成
  - 意図しない接続・切断の発見に有効
  - 大規模スタックの全体像把握

rqt_console:
  - 全ノードのログ（DEBUG/INFO/WARN/ERROR）を一覧表示
  - フィルタリング・ハイライトで重要ログを素早く発見

rqt_plot:
  - シンプルなリアルタイムプロット（PlotJuggler ほど高機能ではない）
  - 素早い確認用
```

### ros2_tracing と LTTng によるレイテンシ分析

```text
ros2_tracing:
  - ROS 2 ノードの実行時レイテンシをカーネルレベルで計測
  - tracetools パッケージで Callback, Subscription, Timer の時刻を記録
  - LTTng (Linux Trace Toolkit Next Generation) と連携

計測できること:
  - センサ受信からBEV推論完了までの End-to-End レイテンシ
  - 特定の Callback が目標周期内に収まっているか
  - DDS スケジューリングのジッタ

使い方:
  $ sudo apt install ros-jazzy-tracetools
  $ ros2 run tracetools_launch trace  --session-name my_session
  → Chromeトレース形式またはperfetto UIで可視化
  
評価方法:
  - センサタイムスタンプ → Callback 実行 → パブリッシュ の時刻差を計測
  - 99パーセンタイルレイテンシが 50ms 以内か確認
  - ワーストケースが発生する条件（GPU 競合等）を特定
```

### rosbag2 と MCAP によるデータ記録

```text
rosbag2 (ROS 2 標準):
  - SQLite3 または CDR (MCAP) バックエンド
  - $ ros2 bag record -a  # 全トピック記録
  - $ ros2 bag record /camera/image_raw /lidar/points /ego/trajectory
  - $ ros2 bag play my_bag.db3  # 再生

MCAP 形式への移行:
  - rosbag2 の MCAP バックエンドプラグインで MCAP 記録可能
  - $ ros2 bag record -a --storage mcap
  - Foxglove Studio での再生・スクラブが高速
  - オフライン解析スクリプトに mcap Python ライブラリを使用

記録設計のベストプラクティス:
  - センサ生データ（/camera/image_raw 等）は圧縮して記録
  - 推論結果（/bev/trajectory 等）は非圧縮で記録（解析時に正確な値が必要）
  - タイムスタンプは ROS_TIME（wall clock）ではなく SENSOR_TIME を使用
  - gPTP (IEEE 802.1AS) で全センサのハードウェアタイムスタンプを同期
```

---

## 15.5 自動運転向け主要 ROS 2 パッケージ・OSS スタック

```text
Autoware.universe (Tier IV):
  - 完全なオープンソース自動運転スタック
  - ROS 2 Humble/Jazzy 対応
  - 構成: Sensing → Localization → Perception → Planning → Control
  - Planning: behavior planner, motion planner, parking planner
  - Localization: NDT matching, EKF state estimator
  - Perception: lidar detection, camera fusion, tracking, prediction
  - Sensing: sensor drivers（Velodyne, Hesai, 各社カメラ）
  - 多くのスタートアップ・研究機関が採用・派生利用

Apollo (Baidu):
  - CyberRT ミドルウェア（ROS 互換 API）
  - 独自のDagアーキテクチャ（グラフ実行エンジン）
  - Chinese OEM の多くが参照・採用
  - CARLA / Apollo Simulator との統合

CARMA Platform (US DoT):
  - 協調型自動運転（CAV）向け研究スタック
  - V2X 通信統合に強み
  - ROS 2 ベース

ros2_control:
  - アクチュエータの抽象化（EPS / スロットル / ブレーキ）
  - Hardware Interface でベンダー固有 CAN メッセージを抽象化
  - ステアリング制御への適用例多数
```

---

## 15.6 製品向けOS・Middlewareの選択

量産製品では、研究向けのUbuntu+ROS 2に加えて、機能安全・リアルタイム性・OTA可用性を満たすOSとMWが必要になる。

### 15.6.1 AUTOSAR Classic Platform (CP)

```text
対象: 従来型安全クリティカルECU（ブレーキ、EPS、SAS等）

特徴:
  - OSEK/VDX ベースの決定論的RTOS
  - 固定周期タスク、静的メモリ確保
  - COM スタック: CAN, LIN, FlexRay, Automotive Ethernet
  - MCAL / BSW / RTE / SWC の4層アーキテクチャ
  - ISO 26262 ASIL-D 認証済みコンポーネント（Vector, ETAS, Elektrobit等）
  - ツール: Vector CANdb++, AUTOSAR Builder, DaVinci Configurator

採用箇所:
  - 制動力制御（AEB/ESC アクチュエータ）
  - EPS（電動パワーステアリング）制御
  - 安全監視 ECU（Watchdog, Plausibility Check）

課題:
  - ML推論に必要な大容量メモリ・GPU には不向き
  - 動的メモリ確保・動的リンクが基本的に禁止
  - 開発スピードが遅い（ツール・ライセンスコストも高い）
```

### 15.6.2 AUTOSAR Adaptive Platform (AP)

```text
対象: 高演算量ADAS/ADコンピュータ（NVIDIA Orin, Qualcomm SA8775P等）

特徴:
  - POSIX 準拠OS（Linux, QNX, Green Hills INTEGRITY等）の上で動作
  - Service-Oriented Architecture (SOA):
      ara::com   : 通信API（SOME/IP + ローカルIPC）
      ara::exec  : 実行管理（SM: State Manager, EM: Execution Manager）
      ara::diag  : UDS 診断（DTC 管理、CAN/DoIP 診断サービス）
      ara::ucm   : OTAアップデート管理
      ara::iam   : IDアクセス管理（セキュリティ）
      ara::per   : 永続化（Key-Value, ファイル）
      ara::log   : ログ管理
      ara::nm    : ネットワーク管理
  - ISO 26262 ASIL-D まで mixed-criticality 設計で対応可能
  - E2E プロテクションプロファイル（安全データの転送時CRC保護）

SOME/IP (Scalable service-Oriented MiddlewarE over IP):
  - AUTOSAR 標準のサービス間通信プロトコル
  - UDP/TCP over Automotive Ethernet (100BASE-T1, 1000BASE-T1)
  - サービスディスカバリ: OfferService / FindService
  - メソッド呼び出し（同期・非同期）
  - イベント通知（fire-and-forget, field notification）

対応ツール・ベンダー:
  - Vector: MICROSAR Adaptive
  - Elektrobit: EB corbos AdaptiveCore
  - ETAS: RTA-VRTE
  - Apex.AI: Apex.OS（ROS 2 互換、ISO 26262認証）

AP の現状（2025年）:
  - R23-11 が最新リリース（年2回更新サイクル）
  - 量産採用は徐々に増加中（BMW ADAS、Mercedes MBUX、Stellantis等）
  - CP との並存が当面続く（ハイブリッドアーキテクチャ）
```

### 15.6.3 QNX Neutrino RTOS

```text
対象: 安全クリティカルな AD コンピュータの Safety Island、または AD 全体

特徴:
  - マイクロカーネルアーキテクチャ:
      ドライバ・ファイルシステム・ネットワークスタックがユーザー空間で動作
      カーネル本体は最小限（割り込み、スケジューラ、IPC のみ）
      ドライバのクラッシュがシステム全体に波及しない
  
  - メモリ保護:
      プロセス間の完全なハードウェアレベルメモリ分離
      ASIL-D の Freedom from Interference (FFI) 要件を満たしやすい

  - 安全認証:
      ISO 26262 ASIL-D
      IEC 61508 SIL 3
      EN 50128 (鉄道)
      DO-178C (航空)
      ← 複数ドメインの認証実績が最も豊富な商用 RTOS の一つ

  - POSIX 準拠:
      pthread, mmap, POSIX IPC が使用可能
      Linux 向けに書いたコードの多くが移植可能

  - 現バージョン: QNX Neutrino RTOS 8.0 (BlackBerry QNX)

QNX Hypervisor:
  - Type-1 ハイパーバイザー
  - QNX RTOS (安全機能) と Linux Guest (ML 推論) を同一 SoC 上で分離実行
  - 時間分割スケジューリング: 安全パーティションが常に CPU コアを確保
  - Virtual Device Driver: ゲスト間でデバイスを安全に共有
  - 採用例: Renesas R-Car + QNX Hypervisor（Toyota 系）

QNX の AD での位置づけ:
  - Safety Monitor: ASIL-D 判定、watchdog、fail-safe 管理
  - 制御出力パス: 目標舵角・加速度の最終検証と出力
  - 推論は Linux Guest (QNX Hypervisor 配下) で実行
  - 安全パーティションと推論パーティションの明確な分離

採用企業（情報公開ベース）:
  - GM (OnStar, ADAS), Ford (Co-Pilot360)
  - BMW (ADAS ECU), BMW-Mobileye 協業
  - Toyota (Denso 経由の車両制御 ECU)
  - Continental, ZF, Aptiv（各社 ADAS ドメインコントローラ）
```

### 15.6.4 その他の製品向けRTOS

```text
Green Hills INTEGRITY RTOS:
  - ASIL-D / DO-178C 認証
  - Separation kernel（ハイパーバイザー内蔵）
  - 採用: F-35 戦闘機・Mobileye EyeQ チップ
  - 価格・ライセンスが高いがミッションクリティカル実績最大級

Wind River VxWorks:
  - 航空・宇宙での長い実績
  - 車載向け: VxWorks Cert
  - QNX ほどの車載市場シェアはないが一定の採用

RIOT-OS:
  - IoT / 超組み込み向けオープンソース RTOS
  - 自動運転コンピュータには非採用（リソース過小）
  - ゾーンコントローラの末端ノード向けに議論はある
```

---

## 15.7 AUTOSAR を使わないアプローチ: QNX + DDS / 自前 MW

AUTOSAR AP はツールチェーンが高価で開発速度が遅い。このため、特に中国系・韓国系 EV OEM とスタートアップは **AUTOSAR を回避して** 自前の MW スタックを QNX または Linux の上に構築するケースが増えている。

### DDS を直接利用する構成

```text
構成:
  OS: QNX 8.0 (Safety Island) + Linux (Inference)
  IPC: Eclipse CycloneDDS on QNX/Linux
  サービス発見: DDS の組み込みディスカバリを使用
  シリアライズ: CDR (DDS 標準) または Cap'n Proto (高速)

利点:
  - AUTOSAR AP の高コストなツールチェーンが不要
  - オープンソースの DDS 実装（CycloneDDS, FastDDS）で無償利用可
  - ROS 2 との親和性が高い（ROS 2 は DDS ベース）
  - 開発・テストサイクルが速い

欠点:
  - AUTOSAR の E2E Profile や SOME/IP などの安全機能を自前実装必要
  - 機能安全認証（ASIL-D）を取得するためのエビデンス量が膨大
  - Tier1 サプライヤからのサポートが限定的

採用例:
  - Nuro (US): DDS + Linux ベースの独自スタック
  - Motional (Hyundai + Aptiv): ROS 2 + 独自安全レイヤー
  - 一部中国 EV スタートアップ
```

### 完全自前 MW（Tesla / Waymo モデル）

```text
Tesla の方針（情報公開ベース）:
  - 独自設計の pub-sub システム（ROS 非互換）
  - 高頻度内部トピック（カメラ/推論）は共有メモリ + ロックフリーキュー
  - ECU 間通信は Automotive Ethernet 上の独自プロトコル
  - FSD Chip D1 向けの専用 ML ランタイム
  - OTA: コアシステムごと更新する「世代アップ」モデル
  
  Tesla 方式の利点:
    - 目的に特化しているためオーバーヘッドが最小
    - FSD のイテレーション速度が非常に速い（週次/月次更新）
  
  Tesla 方式の課題:
    - 完全な内製スタックのため外部検証が困難
    - ISO 26262 プロセスとの折り合いが課題

Waymo の方針:
  - Google 内製 RPC フレームワーク（gRPC ベース）をカスタマイズ
  - 独自のコンピュートクラスタとシミュレーションパイプライン
  - Lidar + カメラの大規模フュージョンパイプラインを独自で構築
  - Driver 1.0 から完全無人化に向けたソフト段階的展開

Waymo モデルの示唆:
  - 「最強の汎用ツール」より「自社アーキテクチャに特化した最適ツール」
  - ただし、資金力・エンジニア人員がなければ維持困難
```

### SOME/IP の利活用

```text
SOME/IP (Scalable service-Oriented MiddlewarE over IP):
  - AUTOSAR 標準だが、非 AUTOSAR 環境でも独立して使用可能
  - vsomeip (open source): CommonAPI ベースの OSS 実装
  - RoutingManager で ECU 間のサービスを中継

SOME/IP を使う利点:
  - Tier1 ECU との相互運用性（従来の AUTOSAR CP ECU と通信可能）
  - Automotive Ethernet（100BASE-T1, 1000BASE-T1）の標準プロトコル
  - wireshark プラグインで診断・デバッグが可能

SOME/IP を使わない場合:
  - gRPC + Protobuf: Google 系スタック、可読性・クロスプラットフォーム
  - Zenoh: 低オーバーヘッド、クラウド統合が容易
  - ZeroMQ: シンプルな非同期通信、研究向け
```

---

## 15.8 時刻同期・センサデータ整合

マルチセンサシステムでは、異なるセンサのデータを同一時刻基準で扱うことが必須である。

```text
gPTP (IEEE 802.1AS) / PTP (IEEE 1588):
  - 車載 Ethernet ネットワーク上のハードウェアタイムスタンプ同期
  - 精度: <100 ns（ハードウェア同期時）
  - TSN (Time-Sensitive Networking): gPTP を基盤にしたフレームスケジューリング
  - 全センサ（カメラ、LiDAR、GNSS-IMU）が同一時刻基準を持つ

センサタイムスタンプの種類:
  - GNSS 時刻 (UTC): 絶対時刻基準
  - TAI: UTC からうるう秒をなくした単調増加時刻
  - ROS_TIME: wall clock（デバッグ向け）
  - HARDWARE_TIME: センサのハードウェアタイムスタンプ（最も正確）

推奨:
  - 記録・処理は常に HARDWARE_TIME ベースで行う
  - ROS 2 の message_filters::ApproximateTimeSynchronizer で複数センサを同期

典型的なタイムスタンプ処理フロー:
  Camera  (30 Hz, Trigger 同期): gPTP タイムスタンプ付き
  LiDAR   (10 Hz, スキャン終端時刻): gPTP タイムスタンプ付き
  GNSS-IMU (100 Hz): GPS PPS で 1μs 精度の時刻スタンプ
  
  → BEV Fusion 前に最も近い時刻のデータをアライメント
  → タイムスタンプずれが大きいデータはドロップ（max_time_diff_ms 閾値）
```

---

## 15.9 機能安全との統合

```text
ISO 26262 要件とMW設計の関係:

  FFI (Freedom from Interference):
    - あるモジュールの障害が他の ASIL レベルのモジュールに影響しない
    - ハイパーバイザー / MMU によるメモリ分離が主な実現手段
    - QNX マイクロカーネルは FFI に適した設計
    - AUTOSAR AP は E2E Profile で論理的 FFI を補強

  ASIL 分解 (ASIL Decomposition):
    - 例: ASIL-D = ASIL-C + ASIL-A (独立した2経路で実現)
    - 計画出力を2つの独立したパスで検証（Planner + Rule-based Evaluator）
    - 本設計の External Evaluator は ASIL-B 相当の安全監視役

  Watchdog パターン:
    - 計画モジュールが一定周期内に応答しない場合、フェイルセーフ状態へ
    - ハードウェア Watchdog Timer (WDT) + ソフトウェア Watchdog の二重構成

  Safe State 管理:
    - ASIL-D Safety Manager（QNX 側）が常に Safe State への移行を監視
    - 例: Planner クラッシュ → Converter がフォールバック制御へ移行
          (前フレームの軌跡を使いながら速度を緩やかに落とす)

  E2E プロテクション（AUTOSAR E2E Library）:
    - 安全関連データには CRC + カウンタを付加（送受信の整合確認）
    - 例: 目標舵角・目標加速度の転送には E2E Profile 6 を適用
```

---

## 15.10 OTA（Over-The-Air）アップデート

```text
AUTOSAR UCM (Update and Configuration Management):
  - AP 上での標準 OTA アーキテクチャ
  - パッケージ管理、インストール、ロールバックを定義
  - ara::ucm で実装

UPTANE プロトコル:
  - TUF (The Update Framework) の自動車向け拡張
  - 複数ディレクタサーバによる信頼チェーン
  - ECU への不正ファームウェア書き込みを防止
  - ISO 24089 (Software Update Engineering) の参照モデル

ML モデルの OTA 更新の特殊性:
  - バイナリ（モデル重み）のサイズが大きい（数GB）
  - 更新後の性能劣化リスクがある → Shadow Mode で事前検証必須
  - A/B パーティション: 新モデルを B に展開し、問題なければ切り替え
  - ロールバック条件: Shadow Mode KPI が基準値を下回ったら旧バージョンへ

OTA セキュリティ:
  - TLS 1.3 以上の通信暗号化
  - HSM (Hardware Security Module) によるパッケージ署名検証
  - UNECE WP.29 (R155/R156) への適合
  - ダウングレード攻撃の防止（バージョン単調増加制約）
```

---

## 15.11 自動車メーカーの開発動向（2024-2026）

### アーキテクチャのトレンド: ゾーナル E/E アーキテクチャ

```text
従来のドメインアーキテクチャ:
  - ドメインECU: シャシー、ボディ、ADAS、インフォテインメント
  - 各ドメインが独立した CAN バスを持つ
  - 問題: ECU 数が多い（最大150+）、配線ハーネス重い・高コスト

ゾーナルアーキテクチャ（2024-2027年に主流化見込み）:
  - 少数のゾーンECU（車両の物理位置: 前/後/左/右）
  - 高演算量のセントラルコンピュータ（AD/ADAS）
  - ゾーンECU ← Ethernet → セントラル + ゾーンECU ← CAN → センサ/アクチュエータ
  - 配線ハーネス重量を最大30%削減（EV 航続距離改善）

主要OEMの取り組み:
  - VW Group (SSP): CARIAD が中心、VW.OS + 統合Ethernet バックボーン
  - Mercedes-Benz: MB.OS (Mercedes-Benz OS) + 独自ゾーンアーキテクチャ
  - GM: Ultium EV プラットフォームでゾーナル採用
  - Toyota: Arene OS (Woven by Toyota) で次世代 E/E を構築中
```

### 欧州 OEM の動向

```text
Volkswagen Group / CARIAD:
  - ソフトウェア子会社 CARIAD が VW.OS を開発
  - Android Auto + Linux ベース + AUTOSAR AP for safety functions
  - SSP (Scalable Systems Platform): 2028年に向けた次世代プラットフォーム
  - 課題: 開発遅延が続いており、Porsche Taycan/Audi E-tron の
    ソフトウェア問題でリコール（2022-2023）
  - 2024: 対処策として Rivian との提携、Google との連携強化

BMW:
  - Mobileye との長期協業（EyeQ SoC + 独自ソフト）
  - 一部機能は Mobileye AV Kit に依存
  - 独自のADAS ECU に AUTOSAR AP 採用

Mercedes-Benz:
  - MB.OS: 独自 OS をゼロから開発
  - DRIVE PILOT (Level 3 on Autobahn): 2023年より限定販売
  - MBOS コア: Linux + AUTOSAR AP + 独自サービス層

Continental / ZF / Bosch（Tier1）:
  - 各社が統合ドメインコントローラを提供
  - Continental: ADAS Domain Controller (ADC)
  - ZF: ZF ProConnect (AUTOSAR AP ベース)
  - Bosch: Cross-Domain Computing Solution
```

### 米国 OEM・スタートアップの動向

```text
Tesla:
  - 完全内製: HW4 (FSD Computer v4) + 独自ソフトウェアスタック
  - FSD (Full Self-Driving): ASIL 認証より先行開発・商用化を優先
  - D1 Dojo: 訓練用スパコン、AI 学習に特化
  - Shadow Mode: 実走行データをクラウドで収集・学習
  - 強み: 速い開発サイクル、大量のフリートデータ
  - 課題: 安全認証・規制対応の透明性

GM Cruise:
  - Chevy Bolt をベースにした Origin で無人タクシー開発
  - 2023年の事故後、運営縮小。2025年に事業再開模索中
  - ソフトウェアスタック: Linux + 独自MW（非ROS 2）

Waymo:
  - Waymo One でフェニックス/SF で無人ロボタクシー展開中
  - 完全独自スタック（Google 内製フレームワーク）
  - LiDAR + カメラ + Radar の大規模フュージョン
  - Jaguar I-PACE + 独自コンピュータ
  - 強み: 安全性実績、規制当局との信頼関係

Motional (Hyundai + Aptiv):
  - ioniq 5 ベースのロボタクシー
  - ROS 2 + 独自安全レイヤー + AUTOSAR AP 一部採用
```

### 日本 OEM の動向

```text
Toyota / Woven by Toyota:
  - Arene OS: AUTOSAR 互換の次世代車両 OS
  - Woven Planet (現 Woven by Toyota): AD 開発の中核子会社
  - CARLA / Isaac Sim での大規模シミュレーション
  - プロトタイプは ROS 2 + Autoware.universe ベース
  - 量産は Arene OS + AUTOSAR AP への移行を計画
  - 2025年: Woven City（静岡富士山麓）で実証実験

Honda:
  - SENSING Elite: Level 3 の高速道路自動運転（限定販売）
  - Renesas R-Car ベースのコンピュート
  - AUTOSAR Classic/Adaptive の組み合わせ
  - 独自の安全アーキテクチャ（Honda 独自の Safety Monitor）

日産 / INCA (Nissan + Renault):
  - ProPILOT 2.0: ROS 2 + 独自MW の組み合わせ
  - Renault との FlexEVA プラットフォーム

Denso (Tier1):
  - Tier4 に出資し Autoware.universe の商用化を支援
  - AUTOSAR CP/AP の双方を車種に応じて提供
  - 独自 AD ECU (ADAS Domain Controller)
```

### 中国 OEM・テクノロジー企業の動向

```text
BYD:
  - 社内でソフトウェア大量内製（DiLink 4.0 + 次期 OS）
  - NVIDIA Orin ベースの ADAS コンピュータ
  - 独自サービス指向MW（ROS 2 互換APIを参考に設計）

NIO / Li Auto / Xpeng:
  - いずれも NVIDIA Orin + 独自ソフトウェアスタック
  - 開発速度重視: AUTOSAR 最小化、SOA ベースの独自 MW
  - OTA 頻度: 月次以上のペースで機能追加
  - NIO NOP+: 高速道路自動運転（2023年量産）
  - Xpeng XNGP: 都市部自動運転（2024年量産）

Huawei (MDC / ADS):
  - MDC (Mobile Data Center): 独自 AI コンピュートボード
  - Harmony OS Automotive + 独自 CSDA MW
  - OEM への供給: 北汽、アファファ（AITO）等
  - HiCar: スマートフォン連携

Baidu Apollo:
  - CyberRT: 高性能ロボティクス向け MW（ROS 互換 API）
  - Apollo Go: ロボタクシーサービス（武漢・北京等）
  - 中国 OEM 20社超が Apollo プラットフォームを採用・参照
```

---

## 15.12 高性能コンピュートと MW の組み合わせ

```text
NVIDIA DRIVE Platform:
  - SoC: DRIVE Thor (2025年以降)、Orin (現行)
  - DriveOS: Linux ベース（QNX Safety Island オプション）
  - DRIVE AV SDK: Orin 向けの AD 推論フレームワーク
  - TensorRT + CUDA for efficient inference
  - DRIVE Constellation: シミュレーション基盤（Isaac Sim 統合）

Qualcomm Snapdragon RIDE:
  - SoC: SA8775P (現行 flagship)
  - OS 構成: QNX Safety Island + Linux AP
  - FastConnect 7800: V2X/C-V2X 無線統合
  - AI Performance: 30+ TOPS (INT8)

Renesas R-Car:
  - SoC: R-Car V4H (2023年以降)
  - 採用: Toyota Arene OS 対応、Denso 経由
  - QNX + Linux dual OS 構成
  - IPL (Image Processing Library): 独自カメラ処理パイプライン

NXP S32 Platform:
  - S32G: ゲートウェイ + Safety Island
  - S32E: ADAS 向け
  - SafeAssure: ISO 26262 ASIL-D 認証
  - 採用: Continental, ZF のゾーンコントローラ

組み合わせの典型パターン:
  研究段階:
    Intel NUC / NVIDIA Jetson Orin + Ubuntu 22.04 + ROS 2 Jazzy

  プロトタイプ:
    NVIDIA Drive Orin + DriveOS (Linux) + ROS 2 + 独自安全レイヤー

  量産 Level 2+/3:
    NVIDIA Drive Thor + QNX Safety Island + AUTOSAR AP
    または Qualcomm SA8775P + QNX + CycloneDDS

  量産 Level 4 (ロボタクシー):
    Waymo 独自 / Tesla FSD / 自前コンピュート + 独自スタック
```

---

## 15.13 研究から製品への移行パス

```text
Phase 1 — 研究・大学・スタートアップ初期:
  OS:  Ubuntu 24.04 LTS
  MW:  ROS 2 Jazzy (CycloneDDS)
  可視化: Lichtblick (OSS) / Foxglove Studio / RViz2 / PlotJuggler
  記録: MCAP / rosbag2
  シミュレーション: CARLA / Gazebo / Isaac Sim

Phase 2 — プロトタイプ・デモ:
  OS:  Ubuntu 24.04 + PREEMPT_RT
  MW:  ROS 2 Jazzy + ros2_control + iceoryx2 (ゼロコピー IPC)
  セーフティ: ルールベース Evaluator (Python/C++) + Watchdog
  HW:  NVIDIA Drive Orin 開発ボード

Phase 3 — 量産評価・SOP準備:
  OS:  QNX 8.0 (Safety) + Linux (Inference) [QNX Hypervisor]
  MW:  AUTOSAR AP (ara::com + SOME/IP) + CycloneDDS
  安全: E2E Profile, ISO 26262 ASIL decomposition
  HW:  NVIDIA Drive Thor / Qualcomm SA8775P

Phase 4 — 量産 (SOP):
  OS:  QNX + AUTOSAR CP (アクチュエータECU)
  MW:  AUTOSAR AP + SOME/IP (OEM 標準) または 自前 SOA MW
  OTA: UPTANE + UCM, A/B パーティション
  認証: ISO 26262 ASIL-D, UNECE WP.29 (R155/R156)

移行のポイント:
  - ROS 2 で動作確認したアルゴリズムを、
    AUTOSAR/QNX 上でもインターフェース非依存に移植できるよう
    「アルゴリズム層」と「MW 依存層」を最初から分離して設計する
  - 具体的には: 推論・計画コアを pure C++/Python ライブラリとして実装し、
    ROS 2 ノードや AUTOSAR SWC からそのライブラリを呼び出す形にする
  - これにより Phase 1 → Phase 4 へのコード資産を最大限に流用できる
```

---

## 15.14 章のまとめ

```text
本章で解説した要素:
  1. 自動運転における MW の役割（IPC, 時刻同期, 障害隔離, OTA）
  2. 研究向けスタック: Ubuntu + ROS 2 Jazzy + CycloneDDS
  3. ROS 2 開発ツールチェーン（colcon, launch, lifecycle）
  4. 可視化ツールの最大活用（RViz2, Foxglove, PlotJuggler, ros2_tracing）
  5. rosbag2 / MCAP によるデータ記録設計
  6. 製品向け AUTOSAR Classic / Adaptive Platform の構造
  7. QNX Neutrino RTOS: マイクロカーネル, 安全認証, QNX Hypervisor
  8. AUTOSAR 非使用アプローチ: DDS 直利用 / Tesla-Waymo 型自前 MW
  9. SOME/IP の役割と VSomeIP OSS 実装
  10. 時刻同期 (gPTP) とセンサタイムスタンプ整合
  11. ISO 26262 との統合: FFI, ASIL分解, E2E Profile, Safe State
  12. OTA: UPTANE, UCM, A/B パーティション, ML モデル更新
  13. 欧米日中各 OEM の最新開発動向（2024-2026）
  14. 高性能コンピュート (Orin/Thor, SA8775P, R-Car) との組み合わせ
  15. 研究 → 量産への移行パス（4フェーズ）

設計原則:
  アルゴリズム実装と MW 依存層を最初から分離することで、
  研究段階のコード資産を量産まで最大限に活用できる。
```
