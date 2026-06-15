import random
import numpy
from material_collecting_agent import MaterialCollectingAgent
from material_collecting_agent import MaterialCollectingAgentParameters

class TestMaterialCollectingAgent(MaterialCollectingAgent):

    def __init__(self) -> None:
        super().__init__()
        self.count = 0

        # --- 状態定義と内部変数 ---
        self.mode = "SEARCH"            # 現在の行動モード (SEARCH / APPROACH など)
        self.leader_id = 0              # リーダーのID
        self.min_leader_distance = 60   # フォロワー用判定値（フェーズ2以降で使用）
        self.max_leader_distance = 130  # フォロワー用判定値（フェーズ2以降で使用）
        self.lost_material_count = 0    # 資源を見失ったときのカウント用

    def act(self):
       # =================================================================
        # 1. センサ情報の取得 (毎フレーム必ず読み込みます)
        # =================================================================
        sensor_type = self.params.sensor_object_type       
        sensor_distance = self.params.sensor_object_distance   
        sensor_attribute = self.params.sensor_object_attribute 
        collision = self.params.collision_sensor           

        # =================================================================
        # 2. ターミナルへのデバッグ出力（ID0のリーダーのみ出力）
        # =================================================================
        if self.id == self.leader_id:
            print("\n" + "="*50)
            print(f"[エージェント ID:{self.id} のセンサ情報判定] / 現在のモード: 【{self.mode}】")
            print("="*50)

            # ① 衝突センサの判定
            is_collided = (collision == MaterialCollectingAgentParameters.SENSE_COLLIDED)
            print(f"■ 衝突センサ(collision_sensor) : {collision} （判定: {'★衝突中！' if is_collided else '安全' }）")
            print("-" * 50)

            # ② 5方向センサ情報の出力
            print("■ 5方向センサ（0:左端, 1:左前, 2:正面(青線), 3:右前, 4:右端）")
            type_labels = {0: "なし (NONE)", 1: "他エージェント (AGENT)", 2: "障害物 (OBSTACLE)", 3: "資源 (MATERIAL)"}

            for i in range(5):
                t_val = sensor_type[i]
                dist = sensor_distance[i]
                attr = sensor_attribute[i]
                label = type_labels.get(t_val, f"未知の数値({t_val})")
                
                attr_info = ""
                if t_val == MaterialCollectingAgentParameters.SENSE_MATERIAL:
                    attr_info = f" -> [資源の半径: {attr:.2f}]"
                elif t_val == MaterialCollectingAgentParameters.SENSE_AGENT:
                    attr_info = f" -> [検知した他ロボット of ID: {attr}]"

                print(f"  ・セグメント [{i}]: 種別={label} | 距離={dist:.1f}{attr_info}")
            print("="*50)

            # 💡 【デバッグ用ストッパー】
            # もし獲得中（停止中）にログが流れるのが早すぎて見えない場合は、
            # 下の2行のコメントアウト(#)を外すと、獲得中だけEnterキーでの「コマ送り」にできます。
            # if self.mode == "COLLECT":
            #     input("【獲得中ストップ】Enterキーを押すと次の1コマ（フレーム）に進みます...")

        # =================================================================
        # 3. 役割に応じた行動分岐
        # =================================================================
        
        # 3-A. フォロワー（ID1～4）の処理：フェーズ1では安全のために完全停止
        if self.id != self.leader_id:
            self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
            self.params.communication_message = f"STANDSTILL:{self.id}"
            return

        # 3-B. リーダー（ID0）の最優先処理：障害物・他ロボットとの衝突回避
        if collision == MaterialCollectingAgentParameters.SENSE_COLLIDED:
            self.mode = "SEARCH" # 衝突したら一度回収を諦めて検索モードへリセット
            self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
            self.params.angular_velocity = random.uniform(40, 80)
            self.params.communication_message = f"LEADER_AVOID:{self.id}"
            return

        # 3-C. リーダー（ID0）の通常行動
        # 周囲5方向に資源（SENSE_MATERIAL = 3）があるか一括チェック
        material_idx = -1
        for i in range(5):
            if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_MATERIAL:
                material_idx = i
                break

        # -----------------------------------------------------------------
        # ▼ パターンA：すでに「回収モード(COLLECT)」に入っている場合
        # -----------------------------------------------------------------
        if self.mode == "COLLECT":
            if material_idx == -1:
                # 周囲から完全に資源の反応が消えた ＝ 資源量が0になって本当に消滅した瞬間！
                print("【システムログ】資源の消滅（回収完了）を確認！探索モードに戻ります。")
                self.mode = "SEARCH"
                self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD # すぐ次の探索へ
                self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.7
            else:
                # まだ周囲に資源の反応がある（まだ残量がある）なら、その場で停止して吸い続ける
                self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                self.params.communication_message = f"COLLECTING:{self.id}"

        # -----------------------------------------------------------------
        # ▼ パターンB：「探索モード(SEARCH)」の場合
        # -----------------------------------------------------------------
        elif self.mode == "SEARCH":
            # 資源が見つかっているなら接近判定
            if material_idx != -1:
                material_radius = sensor_attribute[material_idx] # 資源の半径
                distance = sensor_distance[material_idx]          # 資源までの距離

                # 【距離 <= 半径】になったら、資源の内部に入ったので回収モードにロック！
                if distance <= material_radius:
                    print("【システムログ】資源内部への突入を検知。回収モード(COLLECT)を開始します。")
                    self.mode = "COLLECT"
                    self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                    self.params.communication_message = f"START_COLLECT:{self.id}"
                else:
                    # まだ外側にいるので、資源がある方向に向かって近づく
                    if material_idx == 2:
                        self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                        self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    elif material_idx < 2:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = -25
                    else:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = 25
                    self.params.communication_message = f"LEADER_APPROACH:{self.id}"

            # 資源がどこにも見つからないときはランダム探索
            else:
                action_dice = random.randint(0, 4)
                if action_dice != 4:
                    self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.7
                    self.params.communication_message = f"LEADER_FORWARD:{self.id}"
                else:
                    self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                    self.params.angular_velocity = random.uniform(-30, 30)
                    self.params.communication_message = f"LEADER_ROTATE:{self.id}"