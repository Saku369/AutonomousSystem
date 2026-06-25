import random
import numpy
from material_collecting_agent import MaterialCollectingAgent
from material_collecting_agent import MaterialCollectingAgentParameters

class ReaderMaterialCollectingAgent(MaterialCollectingAgent):

    def __init__(self) -> None:
        super().__init__()

    def act(self):
        
        # 5方向センサーから資源の位置を特定
        material_idx = next((idx for idx in [2, 1, 3, 0, 4] if self.params.sensor_object_type[idx] == MaterialCollectingAgentParameters.SENSE_MATERIAL), -1)

        # 自分が資源の回収位置にいるか判定
        is_my_collecting = False
        if material_idx != -1:
            if (self.params.sensor_object_distance[material_idx] - self.params.sensor_object_attribute[material_idx]) <= 0:
                is_my_collecting = True

        # 周囲の電波から回収中の大隊メンバーとソロの救助信号をスキャン
        active_collectors = []
        is_solo_calling = False 

        for msg in self.params.received_messages:
            if "MODE:COLLECT" in msg:
                try:
                    parts = msg.split(':')
                    other_id = int(parts[1])
                    if other_id != 4: 
                        active_collectors.append(other_id)
                except:
                    pass
            if "MODE:COME_HERE" in msg:
                is_solo_calling = True 

        is_team_collecting = len(active_collectors) > 0

        # 自分の送信メッセージを決定
        if self.id == 4:
            if is_my_collecting:
                self.params.communication_message = f"ID:{self.id}:MODE:SEARCH"
            elif is_team_collecting:
                self.params.communication_message = f"ID:{self.id}:MODE:COME_HERE"
            else:
                self.params.communication_message = f"ID:{self.id}:MODE:SEARCH"
        else:
            my_current_mode = "COLLECT" if is_my_collecting else "SEARCH"
            self.params.communication_message = f"ID:{self.id}:MODE:{my_current_mode}"

        # ソロ機が孤立した回収者を検知した時のログ
        if self.id == 4 and is_team_collecting:
            print(f"[Agent 4 SOLO] 🚨 Detected Isolated Leader {active_collectors}! Broadcasting COME_HERE Beacon!")

        # 衝突回避処理（広い方を選択して強ランダム旋回）
        if not is_my_collecting:
            if self.params.collision_sensor == MaterialCollectingAgentParameters.SENSE_COLLIDED or \
               (self.params.sensor_object_type[2] in [MaterialCollectingAgentParameters.SENSE_OBSTACLE, MaterialCollectingAgentParameters.SENSE_AGENT] and self.params.sensor_object_distance[2] < 25.0):
                
                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                if self.params.sensor_object_type[2] == MaterialCollectingAgentParameters.SENSE_AGENT:
                    self.params.angular_velocity = 180
                else:
                    left_blocked = (self.params.sensor_object_type[1] == MaterialCollectingAgentParameters.SENSE_OBSTACLE)
                    right_blocked = (self.params.sensor_object_type[3] == MaterialCollectingAgentParameters.SENSE_OBSTACLE)
                    
                    if left_blocked and not right_blocked:
                        self.params.angular_velocity = random.uniform(-150.0, -30.0)
                    elif right_blocked and not left_blocked:
                        self.params.angular_velocity = random.uniform(30.0, 150.0)
                    elif left_blocked and right_blocked:
                        self.params.angular_velocity = random.uniform(150.0, 210.0)
                    else:
                        self.params.angular_velocity = random.uniform(30.0, 150.0) if random.choice([True, False]) else random.uniform(-150.0, -30.0)
                return

        # 回収処理（資源が目の前にある場合）
        if is_my_collecting:
            self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
            return

        # 移動・探索処理（資源が目の前にない場合）
        if material_idx != -1:
            # 視界に資源がある場合は接近
            if material_idx == 2:
                self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
                if (self.params.sensor_object_distance[2] - self.params.sensor_object_attribute[2]) < 30.0:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * (0.2 + ((self.params.sensor_object_distance[2] - self.params.sensor_object_attribute[2]) / 30.0) * 0.6)
                else:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
            elif material_idx < 2:
                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                self.params.angular_velocity = -12
            else:
                self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
                self.params.angular_velocity = 12

        else:
            # 視界に資源がない場合の巡回ロジック
            self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
            
            if self.id == 4:
                # ソロ（ID4）の挙動：大隊が近くにいればホバリング、いなければ高速巡回
                if is_team_collecting:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.2
                    self.params.angular_velocity = random.uniform(-10.0, 10.0)
                else:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                    self.params.angular_velocity = random.uniform(-4.0, 4.0)
            
            else:
                # 大隊（ID0-3）の挙動：状況に応じて追尾蛇行か分散探索かを選択
                if is_team_collecting:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.4
                    self.params.angular_velocity = random.choice([25.0, -25.0])
                elif is_solo_calling:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.5
                    self.params.angular_velocity = random.choice([35.0, -35.0])
                else:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.7
                    self.params.angular_velocity = random.uniform(-3.0, 3.0)