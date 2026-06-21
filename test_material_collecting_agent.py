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
        # 1. センサ情報の取得
        # =================================================================
        sensor_type = self.params.sensor_object_type       
        sensor_distance = self.params.sensor_object_distance   
        sensor_attribute = self.params.sensor_object_attribute 
        collision = self.params.collision_sensor           

        # =================================================================
        # 2. 役割に応じた行動分岐（初期化・最優先処理）
        # =================================================================
        
        # ID2〜4 のフォロワーは完全停止のまま固定
        if self.id > 1:
            self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
            return

        # -----------------------------------------------------------------
        # 2-A. 【リーダー（ID0）の衝突回避】※完全に以前の「壁（OBSTACLE）専用」に固定
        # -----------------------------------------------------------------
        if self.id == self.leader_id:
            if collision == MaterialCollectingAgentParameters.SENSE_COLLIDED:
                self.mode = "SEARCH" 
                left_wall_dist, right_wall_dist = 150.0, 150.0
                for i in [0, 1]:
                    if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_OBSTACLE:
                        if sensor_distance[i] < left_wall_dist: left_wall_dist = sensor_distance[i]
                for i in [3, 4]:
                    if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_OBSTACLE:
                        if sensor_distance[i] < right_wall_dist: right_wall_dist = sensor_distance[i]

                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                self.params.angular_velocity = 45 if left_wall_dist <= right_wall_dist else -45 
                return

        # -----------------------------------------------------------------
        # 2-B. 【フォロワー（ID1）の衝突回避】※壁に加えて、リーダー（AGENT）も避ける
        # -----------------------------------------------------------------
        else:
            if collision == MaterialCollectingAgentParameters.SENSE_COLLIDED:
                self.mode = "SEARCH" 
                left_dist, right_dist = 150.0, 150.0
                # フォロワーは壁（OBSTACLE）もリーダー（AGENT）も「ぶつかる障害物」として距離を測る
                for i in [0, 1]:
                    if sensor_type[i] in [MaterialCollectingAgentParameters.SENSE_OBSTACLE, MaterialCollectingAgentParameters.SENSE_AGENT]:
                        if sensor_distance[i] < left_dist: left_dist = sensor_distance[i]
                for i in [3, 4]:
                    if sensor_type[i] in [MaterialCollectingAgentParameters.SENSE_OBSTACLE, MaterialCollectingAgentParameters.SENSE_AGENT]:
                        if sensor_distance[i] < right_dist: right_dist = sensor_distance[i]

                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                self.params.angular_velocity = 45 if left_dist <= right_dist else -45 
                return

        # 5方向センサから「資源」と「リーダー」のインデックスを見つける
        material_idx = -1
        leader_idx = -1
        for i in range(5):
            if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_MATERIAL:
                if material_idx == -1: material_idx = i
            if sensor_type[i] == MaterialCollectingAgentParameters.SENSE_AGENT and sensor_attribute[i] == self.leader_id:
                if leader_idx == -1: leader_idx = i

        # -----------------------------------------------------------------
        # ▼ パターンA：回収モード(COLLECT)の場合（全エージェント共通仕様で固定）
        # -----------------------------------------------------------------
        if self.mode == "COLLECT":
            still_inside = False
            if material_idx != -1:
                current_radius = sensor_attribute[material_idx]   
                current_distance = sensor_distance[material_idx]  
                if current_distance <= current_radius:
                    still_inside = True

            if still_inside:
                self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                self.params.communication_message = f"COLLECTING:{self.id}"
            else:
                self.mode = "SEARCH"
                self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD 
                self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.7
                self.params.communication_message = f"FORWARD:{self.id}"

        # -----------------------------------------------------------------
        # ▼ パターンB：探索・移動モード(SEARCH)の場合
        # -----------------------------------------------------------------
        elif self.mode == "SEARCH":
            # ① 目の前に資源があるなら最優先で突入（全エージェント共通仕様で固定）
            if material_idx != -1:
                material_radius = sensor_attribute[material_idx]
                distance = sensor_distance[material_idx]
                brake_threshold = material_radius + 5.0

                if distance <= brake_threshold:
                    self.mode = "COLLECT"
                    self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                else:
                    if material_idx == 2:
                        self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                        self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    elif material_idx < 2:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = -25
                    else:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = 25

            # ② 資源がない場合の行動（ここで完全に役割を分離！）
            else:
                # ---------------------------------------------------------
                # 【リーダー（ID0）の行動】※一切他人に惑わされず、常に巡回探索
                # ---------------------------------------------------------
                if self.id == self.leader_id:
                    self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    self.params.communication_message = f"LEADER_PATROL"

                # ---------------------------------------------------------
                # 【フォロワー（ID1）の行動】
                # ---------------------------------------------------------
                else:
                    # リーダーが視界にいるなら距離をキープしながら追従
                    if leader_idx != -1:
                        leader_dist = sensor_distance[leader_idx]

                        if leader_dist > self.max_leader_distance:
                            if leader_idx == 2:
                                self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                                self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                            elif leader_idx < 2:
                                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                                self.params.angular_velocity = -25
                            else:
                                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                                self.params.angular_velocity = 25
                        elif leader_dist < self.min_leader_distance:
                            self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                        else:
                            if leader_idx == 2:
                                self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                            elif leader_idx < 2:
                                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                                self.params.angular_velocity = -20
                            else:
                                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                                self.params.angular_velocity = 20
                    
                    # リーダーが視界にいないなら自律して巡回探索
                    else:
                        self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                        self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                        self.params.communication_message = f"FOLLOWER_PATROL"