import random
import numpy
from material_collecting_agent import MaterialCollectingAgent
from material_collecting_agent import MaterialCollectingAgentParameters

class TestMaterialCollectingAgent(MaterialCollectingAgent):

    # クラス変数：全エージェントで1つのランキング表（辞書型）を共有
    shared_ranking = {}

    def __init__(self) -> None:
        super().__init__()
        self.count = 0

        # --- 状態定義と内部変数 ---
        self.mode = "SEARCH"            
        self.leader_id = 0              
        self.min_leader_distance = 60   
        self.max_leader_distance = 130  
        self.lost_material_count = 0    
        self.radar_timer = 0            
        self.locked_direction = 0       
        self.avoid_phase = 0            
        self.is_holding_material = False 
        self.material_seen_count = 0    # 資源を視界に捉えた累積カウント

    def act(self):
        self.count += 1

        # =================================================================
        # 1. センサ情報の取得
        # =================================================================
        sensor_type = self.params.sensor_object_type       
        sensor_distance = self.params.sensor_object_distance   
        sensor_attribute = self.params.sensor_object_attribute 
        collision = self.params.collision_sensor           

        # =================================================================
        # 2. 役割に応じた初期カット（全5台の封印を解除：ID5以上のみ停止）
        # =================================================================
        if self.id > 4:
            self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
            return

        # =================================================================
        # 3. 5方向センサのスキャン ＆ 資源カウントの蓄積
        # =================================================================
        material_idx = -1
        leader_idx = -1
        is_material_in_view = False

        for i in range(5):
            if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_MATERIAL:
                if material_idx == -1: material_idx = i
                is_material_in_view = True 
            if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_AGENT and sensor_attribute[i] == self.leader_id:
                if leader_idx == -1: leader_idx = i

        # 資源が視界に入っていたらカウントを増やす
        if is_material_in_view:
            self.material_seen_count += 1

        # =================================================================
        # 3.5 共有ボードを使った動的リーダー選出（開幕30Fは全員で分散探索）
        # =================================================================
        # ① 自分の最新のカウントは毎フレーム常にボードに書き込む
        TestMaterialCollectingAgent.shared_ranking[self.id] = self.material_seen_count

        # ② 開幕30フレームまでは全員を「独立したリーダー（フォロワーを作らない）」扱いにする
        if self.count < 20:
            self.leader_id = self.id  # 自分自身をリーダーにすることで、全員がバラバラにパトロールする
        
        # ③ 30フレーム以降は、40フレームごとにランキングで真のリーダーを決定
        elif self.count % 45 == 0:
            if TestMaterialCollectingAgent.shared_ranking:
                new_leader = max(TestMaterialCollectingAgent.shared_ranking, key=TestMaterialCollectingAgent.shared_ranking.get)
                if self.id == 0 and new_leader != self.leader_id:
                    print(f"[Time:{self.count}] 👑 リーダーが交代しました: ID {self.leader_id} -> ID {new_leader} (最大カウント: {TestMaterialCollectingAgent.shared_ranking[new_leader]})")
                self.leader_id = new_leader

        # =================================================================
        # 4. 最優先処理：障害物との衝突回避（100度すれ違い・全員共通基本動作）
        # =================================================================
        if not self.is_holding_material and (collision == MaterialCollectingAgentParameters.SENSE_COLLIDED or self.avoid_phase > 0):
            self.mode = "SEARCH" 

            if self.avoid_phase == 0:
                left_dist, right_dist = 150.0, 150.0
                is_agent_collision = False

                for i in range(5):
                    if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_AGENT:
                        if sensor_distance[i] < 40.0: 
                            is_agent_collision = True

                    if sensor_type[i] in [MaterialCollectingAgentParameters.SENSE_OBSTACLE, MaterialCollectingAgentParameters.SENSE_AGENT]:
                        if i in [0, 1] and sensor_distance[i] < left_dist: left_dist = sensor_distance[i]
                        if i in [3, 4] and sensor_distance[i] < right_dist: right_dist = sensor_distance[i]
                
                # 💡 回転角度を 45 から 25 に小さく刻む（ピンポン現象の防止）
                self.locked_direction = 25 if left_dist <= right_dist else -25

                if is_agent_collision:
                    self.avoid_phase = 2
                    self.radar_timer = 3  
                else:
                    self.avoid_phase = 1
                    self.radar_timer = 0

            if self.avoid_phase == 2:
                self.radar_timer -= 1
                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                # 💡 エージェントとのすれ違い時は少し強めに回る（30度）
                self.params.angular_velocity = 30 if self.locked_direction > 0 else -30
                self.params.communication_message = f"AVOID_AGENT_TURN:{self.id}"

                if self.radar_timer <= 0:
                    self.avoid_phase = 1
                    self.radar_timer = 2  
                return

            elif self.avoid_phase == 1:
                if self.radar_timer > 0:
                    self.radar_timer -= 1
                    self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.6
                    self.params.communication_message = f"AVOID_AGENT_PASS:{self.id}"
                    
                    if self.radar_timer <= 0:
                        self.avoid_phase = 0  
                    return
                else:
                    if collision != MaterialCollectingAgentParameters.SENSE_COLLIDED and sensor_type[2] != MaterialCollectingAgentParameters.SENSE_OBSTACLE:
                        self.avoid_phase = 0  
                    else:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        # 💡 通常の壁回避も角度を 30度 に抑えて滑らかに受け流す
                        self.params.angular_velocity = 30 if self.locked_direction > 0 else -30
                        self.params.communication_message = f"AVOID_WALL"
                        return

        # =================================================================
        # 5. 通常時の行動分岐
        # =================================================================
        if self.mode == "COLLECT":
            still_inside = False
            current_radius = 0.0
            if material_idx != -1:
                current_radius = sensor_attribute[material_idx]   
                current_distance = sensor_distance[material_idx]  
                if current_distance <= current_radius:
                    still_inside = True

            if still_inside:
                self.is_holding_material = True
                self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                # 💡 資源回収中のメッセージに「資源の規模（半径）」を載せて発信する
                self.params.communication_message = f"COLLECT_SIZE:{current_radius}:ID:{self.id}"
            else:
                self.is_holding_material = False
                self.mode = "SEARCH"
                self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD 
                self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.7
                self.params.communication_message = f"FORWARD:{self.id}"

        elif self.mode == "SEARCH":
            self.is_holding_material = False

            if material_idx != -1:
                material_radius = sensor_attribute[material_idx]
                distance = sensor_distance[material_idx]
                remaining_dist = distance - material_radius

                if remaining_dist <= 0:
                    self.mode = "COLLECT"
                    self.is_holding_material = True
                    self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                    self.radar_timer = 0 
                else:
                    # 💡 正面（セグメント2）に捉えている場合は、タイマーをリセットして直進
                    if material_idx == 2:
                        self.radar_timer = 0  # 首振りロック解除
                        self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                        if remaining_dist < 30.0:
                            slow_factor = 0.2 + (remaining_dist / 30.0) * 0.6
                            self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * slow_factor
                        else:
                            self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    
                    # 💡 左右に旋回する場合、首振りチャタリングを防止するためにタイマーで固定する
                    else:
                        # まだ旋回持続タイマーが動いていない場合のみ、新しい旋回方向を決める
                        if self.radar_timer <= 0:
                            self.radar_timer = 3  # 3フレームは同じ方向へ回り続ける
                            self.locked_direction = -20 if material_idx < 2 else 20
                        
                        self.radar_timer -= 1
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = self.locked_direction

            # 🗺️ 【優先度：低】資源が視界にない場合
            else:
                # ---------------------------------------------------------
                # 👑 【リーダーの行動】通常通り前進パトロール
                # ---------------------------------------------------------
                if self.id == self.leader_id:
                    self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    self.params.communication_message = f"LEADER_SEARCH"
                
                # ---------------------------------------------------------
                # 🏃 【フォロワーの行動】リーダー外枠集合 ＆ 満員スルーロジック
                # ---------------------------------------------------------
                else:
                    leader_msg_received = False
                    is_resource_full = False

                    if hasattr(self.params, 'received_messages') and len(self.params.received_messages) > 0:
                        for msg in self.params.received_messages:
                            # 📡 誰かが「COLLECT_SIZE」を発信しているかチェック
                            if "COLLECT_SIZE" in msg:
                                try:
                                    parts = msg.split(":")
                                    size_val = float(parts[1])
                                    sender_id = int(parts[3])

                                    # 💡 【判断処理】
                                    # 資源の半径が小さく（例: 40.0未満）、かつその送信元エージェントが
                                    # 現在自分が認識している「真のリーダー」の場合
                                    if size_val < 40.0 and sender_id == self.leader_id:
                                        # 自分がスペースに入れない「ハズレ（満員）」とみなす
                                        is_resource_full = True
                                except (ValueError, IndexError):
                                    pass

                            # 通常のリーダー電波受信チェック
                            if "LEADER" in msg:
                                leader_msg_received = True

                    # 🔄 満員（スルー判定）なら、リーダーを無視して別の場所を開拓（直進ダッシュ）
                    if is_resource_full:
                        self.radar_timer = 0
                        self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                        self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.9
                        self.params.communication_message = f"RESOURCE_FULL_IGNORE:{self.id}"

                    # 🔄 入るスペースがある（今まで通り）なら、通常の外枠集合
                    else:
                        if leader_msg_received:
                            if self.radar_timer <= 0:
                                self.radar_timer = 4  

                            if self.radar_timer > 0:
                                self.radar_timer -= 1
                                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                                self.params.angular_velocity = 30  
                                self.params.communication_message = f"EDGE_LOCK_TURN:{self.id}"
                            else:
                                self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                                self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.5
                                self.params.communication_message = f"EDGE_FOLLOWING:{self.id}"
                        else:
                            self.radar_timer = 0  
                            self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                            self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.9  
                            self.params.communication_message = f"RADIO_LOST_PATROL:{self.id}"

        # =================================================================
        # リーダーが回収中（COLLECTモード）のときの発信上書き
        # =================================================================
        if self.id == self.leader_id and self.mode == "COLLECT":
            # 💡 リーダー自身が回収中の場合も、規模の数値をしっかり乗せて叫びます
            if material_idx != -1:
                leader_radius = sensor_attribute[material_idx]
                self.params.communication_message = f"COLLECT_SIZE:{leader_radius}:LEADER_ID:{self.id}"
            else:
                self.params.communication_message = f"COLLECT_SIZE:30.0:LEADER_ID:{self.id}"