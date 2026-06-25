import random
import numpy
from material_collecting_agent import MaterialCollectingAgent
from material_collecting_agent import MaterialCollectingAgentParameters

class RederMaterialCollectingAgent(MaterialCollectingAgent):

    def __init__(self) -> None:
        super().__init__()

    def act(self):
        # =================================================================
        # 1. 役割に応じた初期カット（全5台の封印を解除：ID5以上のみ停止）
        # =================================================================
        if self.id > 4:
            self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
            return

        if not hasattr(self.params, 'communication_message') or self.params.communication_message is None or "MY_DATA" not in self.params.communication_message or len(self.params.communication_message.split(':')) < 7:
            self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:0:MODE:SEARCH:ROT:0"

        # =================================================================
        # 2. 5方向センサのスキャン（資源位置の特定）
        # =================================================================
        material_idx = next((idx for idx in [2, 1, 3, 0, 4] if self.params.sensor_object_type[idx] == MaterialCollectingAgentParameters.SENSE_MATERIAL), -1)

        # =================================================================
        # 3. 動的リーダー選出
        # =================================================================
        current_leader = max(
            [int(msg.split(':')[2]) for msg in self.params.received_messages if "MY_DATA" in msg and len(msg.split(':')) > 4] + [self.id],
            key=lambda uid: next(
                (int(msg.split(':')[4]) for msg in self.params.received_messages if f"ID:{uid}:" in msg and len(msg.split(':')) > 4),
                int(self.params.communication_message.split(':')[4]) if uid == self.id else 0
            )
        )
        current_leader = self.id if max([int(msg.split(':')[4]) for msg in self.params.received_messages if "MY_DATA" in msg and len(msg.split(':')) > 4] + [int(self.params.communication_message.split(':')[4])]) < 15 else current_leader

        # =================================================================
        # 4. 最優先処理：障害物との衝突回避（random.uniform によるランダム転回）
        # =================================================================
        if self.params.collision_sensor == MaterialCollectingAgentParameters.SENSE_COLLIDED or \
           (self.params.sensor_object_type[2] in [MaterialCollectingAgentParameters.SENSE_OBSTACLE, MaterialCollectingAgentParameters.SENSE_AGENT] and self.params.sensor_object_distance[2] < 25.0):
            
            self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
            self.params.angular_velocity = random.uniform(-120, 120)
            
            # 衝突時はタイマーを0にリセットして脱出
            self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{self.params.communication_message.split(':')[4] if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0}:MODE:SEARCH:ROT:0"
            return

        # =================================================================
        # 5. 行動分岐：回収モード（COLLECT）
        # =================================================================
        if "MODE:COLLECT" in self.params.communication_message:
            
            if (self.params.sensor_object_type[2] == MaterialCollectingAgentParameters.SENSE_MATERIAL and self.params.sensor_object_distance[2] <= self.params.sensor_object_attribute[2]) or \
               (self.params.sensor_object_type[1] == MaterialCollectingAgentParameters.SENSE_MATERIAL and self.params.sensor_object_distance[1] <= self.params.sensor_object_attribute[1]) or \
               (self.params.sensor_object_type[3] == MaterialCollectingAgentParameters.SENSE_MATERIAL and self.params.sensor_object_distance[3] <= self.params.sensor_object_attribute[3]) or \
               (self.params.sensor_object_type[0] == MaterialCollectingAgentParameters.SENSE_MATERIAL and self.params.sensor_object_distance[0] <= self.params.sensor_object_attribute[0]) or \
               (self.params.sensor_object_type[4] == MaterialCollectingAgentParameters.SENSE_MATERIAL and self.params.sensor_object_distance[4] <= self.params.sensor_object_attribute[4]):
                
                self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{int(self.params.communication_message.split(':')[4]) + 1 if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 1}:MODE:COLLECT:SIZE:{self.params.sensor_object_attribute[2] if self.params.sensor_object_type[2] == MaterialCollectingAgentParameters.SENSE_MATERIAL else (self.params.sensor_object_attribute[1] if self.params.sensor_object_type[1] == MaterialCollectingAgentParameters.SENSE_MATERIAL else (self.params.sensor_object_attribute[3] if self.params.sensor_object_type[3] == MaterialCollectingAgentParameters.SENSE_MATERIAL else 30.0))}"
            else:
                self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD 
                self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.7
                self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{self.params.communication_message.split(':')[4] if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0}:MODE:SEARCH:ROT:0"

        # =================================================================
        # 6. 行動分岐：通常探索モード（SEARCH）
        # =================================================================
        else:
            if material_idx != -1:
                if (self.params.sensor_object_distance[material_idx] - self.params.sensor_object_attribute[material_idx]) <= 0:
                    self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
                    self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{int(self.params.communication_message.split(':')[4]) + 1 if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 1}:MODE:COLLECT:SIZE:{self.params.sensor_object_attribute[material_idx]}"
                else:
                    if material_idx == 2:
                        self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                        if (self.params.sensor_object_distance[2] - self.params.sensor_object_attribute[2]) < 30.0:
                            self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * (0.2 + ((self.params.sensor_object_distance[2] - self.params.sensor_object_attribute[2]) / 30.0) * 0.6)
                        else:
                            self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    elif material_idx < 2:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = -20
                    else:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = 20
                    self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{self.params.communication_message.split(':')[4] if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0}:MODE:SEARCH:ROT:0"

            # 🗺️ 資源が視界にない場合（リーダーパトロール ＆ フォロワーの電波無視クールダウン追従）
            else:
                # 💡 メッセージから現在の「索敵・無視タイマー（ROT値）」を直に取得
                rot_count = int(self.params.communication_message.split(':')[8] if "MY_DATA" in self.params.communication_message and len(self.params.communication_message.split(':')) > 8 else 0)

                if self.id == current_leader:
                    self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{self.params.communication_message.split(':')[4] if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0}:MODE:SEARCH:ROT:0"
                
                # 💡 【追加ロジック】カウントが12〜30の間は、リーダーの電波を完全にシャットアウト（強制通常前進）
                elif 12 <= rot_count < 45:
                    self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    # フレームごとにカウントを+1していき、一定時間電波を弾く
                    self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{self.params.communication_message.split(':')[4] if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0}:MODE:SEARCH:ROT:{rot_count + 1}"
                
                else:
                    # カウントが45（クールダウン終了）に達したら 0 にリセットして再索敵可能にする
                    next_rot = 0 if rot_count >= 45 else rot_count
                    
                    # 従来の「旋回1：前進2」の索敵処理
                    if (int(self.params.communication_message.split(':')[4] if "MY_DATA" in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0) % 3) == 0:
                        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                        self.params.angular_velocity = 30
                        # 旋回した時だけ索敵カウントを+1（12に達するまで回る）
                        self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{self.params.communication_message.split(':')[4] if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0}:MODE:SEARCH:ROT:{next_rot + 1}"
                    else:
                        self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                        self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.5
                        self.params.communication_message = f"MY_DATA:ID:{self.id}:COUNT:{self.params.communication_message.split(':')[4] if 'MY_DATA' in self.params.communication_message and len(self.params.communication_message.split(':')) > 4 else 0}:MODE:SEARCH:ROT:{next_rot}"