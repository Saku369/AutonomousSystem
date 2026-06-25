import random
import numpy
from material_collecting_agent import MaterialCollectingAgent
from material_collecting_agent import MaterialCollectingAgentParameters

class ReaderMaterialCollectingAgent(MaterialCollectingAgent):

    def __init__(self) -> None:
        super().__init__()

    def act(self):
        
        # 1. 資源の位置特定
        material_idx = next((idx for idx in [2, 1, 3, 0, 4] if self.params.sensor_object_type[idx] == MaterialCollectingAgentParameters.SENSE_MATERIAL), -1)

        # 2. 自身の回収状態判定
        is_my_collecting = False
        if material_idx != -1:
            if (self.params.sensor_object_distance[material_idx] - self.params.sensor_object_attribute[material_idx]) <= 0:
                is_my_collecting = True

        # 3. 受信電波の解析
        active_readers = []
        old_readers = []
        is_solo_calling = False 

        for msg in self.params.received_messages:
            try:
                parts = msg.split(':')
                other_id = int(parts[1])
                if other_id != 4:
                    if "MODE:READER" in msg:
                        active_readers.append(other_id)
                    elif "MODE:OLD_READER" in msg:
                        old_readers.append(other_id)
            except:
                pass
            if "MODE:COME_HERE" in msg:
                is_solo_calling = True 

        other_active_reader_exists = len([cid for cid in active_readers if cid != self.id]) > 0
        is_team_collecting = len(active_readers) > 0

        # 4. 周囲の仲間ロボットの密集度測定
        near_agents_count = sum(1 for idx in range(5) if 
                                self.params.sensor_object_type[idx] == MaterialCollectingAgentParameters.SENSE_AGENT and 
                                self.params.sensor_object_distance[idx] < 40.0)

        # 5. 送信メッセージ（モード）の決定
        my_current_mode = "SEARCH"
        
        if self.id == 4:
            # 【ID4完全独立】他機の電波は一切無視。資源上でもあえて隠密（SEARCH）
            my_current_mode = "SEARCH"
        else:
            if is_my_collecting:
                my_current_mode = "READER"
            else:
                if near_agents_count >= 2 and not other_active_reader_exists:
                    my_current_mode = "OLD_READER"
                else:
                    my_current_mode = "SEARCH"

        self.params.communication_message = f"ID:{self.id}:MODE:{my_current_mode}"

        # 6. 衝突回避処理
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

        # 7. 回収動作
        if is_my_collecting:
            self.params.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
            return

        # 8. 移動・探索動作
        if material_idx != -1:
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
            self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
            
            if self.id == 4:
                # 【移動も独立】大隊のリーダーが近くにいても減速せず、常に最高速度で爆走
                self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.8
                self.params.angular_velocity = random.uniform(-4.0, 4.0)
            
            else:
                # 大隊（ID0-3）の優先度分岐
                if len(active_readers) > 0:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.4
                    self.params.angular_velocity = random.choice([25.0, -25.0])
                    
                elif len(old_readers) > 0 or my_current_mode == "OLD_READER":
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.5
                    self.params.angular_velocity = random.choice([20.0, -20.0])
                    
                elif is_solo_calling:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.5
                    self.params.angular_velocity = random.choice([35.0, -35.0])
                    
                else:
                    self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY * 0.7
                    self.params.angular_velocity = random.uniform(-3.0, 3.0)