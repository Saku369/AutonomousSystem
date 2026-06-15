# -*- coding: utf-8 -*-

import time
from environment import Environment
from pyglet import shapes
import pyglet
import random
from material_collecting_agent import MaterialCollectingAgent
from material_collecting_agent import MaterialCollectingAgentParameters
import numpy
import geometric_utilities as geo

#debug_mode = True
debug_mode = False

class MaterialCollectingEnvironment(Environment):


    def __init__(self, frame_x, frame_y) -> None:
        """
        Initializes the environment with the given frame dimensions.

        Args:
            frame_x: The width of the frame.
            frame_y: The height of the frame.

        Returns:
            None
        """
        super().__init__()

        # Batch for rendering
        self.batch = None

        # Frame dimensions
        self.frame_x = frame_x
        self.frame_y = frame_y

        # Agent attributes
        self.agent_positions = []
        self.agent_directions = []
        self.agent_update_interval = 1.0
        self.agent_size = 20
        self.dir_rec_width = self.agent_size / 5
        self.dir_rec_height = self.agent_size * 1.5

        # Communication and sensor ranges
        self.communication_range_radius = 250.0 #通信可能範囲
        self.sensor_range_radius = 150.0 #距離センサの検知範囲
        self.sensor_angle = 120.0 #〃

        # Agent colors and opacities
        self.agent_body_color = (200, 0, 0)
        self.agent_edge_color = (10, 10, 10)
        self.agent_dir_color = (0, 0, 255)
        self.agent_com_range_color = (0, 200, 0)
        self.agent_sensor_range_color = (0, 0, 200)
        self.agent_com_range_opacity = 10
        self.agent_sensor_range_opacity = 50

        # Obstacle attributes
        self.num_max_obstacle = 10
        self.obstacle_prob = 0.5
        self.obstacle_pos_margin = 20.0
        self.obstacle_width_max = 300.0
        self.obstacle_width_min = 100.0
        self.obstacle_height_max = 300.0
        self.obstacle_height_min = 100.0

        # Material attributes
        self.num_max_material = 5
        self.material_pos_margin = 20.0
        self.material_prob = 0.5
        self.material_radius_max = 100.0
        self.material_radius_min = 30.0
        self.material_amount_max = 50
        self.material_amount_min = 10
        self.material_color = (200, 200, 0)
        self.material_opacity_max = 127
        self.material_opacity_min = 30
        self.material_center_color = (50, 50, 0)
        self.material_center_opacity = 80
        self.material_center_radius = 10

        # Material properties
        self.material_shapes = []
        self.material_amounts = []

        # Collected materials
        self.collected_materials = 0

        # Agent time and experiment end
        self.elapsed_agent_time = 0
        self.elapsed_agent_time_end = 200
        self.collected_materials_time_series = numpy.zeros((self.elapsed_agent_time_end, 1))
        self.collected_materials_time_series_for_each_agent = None
        self.collected_materials_sum_for_each_agent = []
        self.experiment_end = False

        # Debug mode
        if debug_mode:
            self.dist_line_shapes = []


    def update(self, dt):
        """
        Update the position and status of the agents based on their actions and collision checks.
        Parameters:
            dt: float - time interval for the update
        Returns:
            None
        """
        super().update(dt)
        
        self.info_tag_material_amount.text = "Amount:" + str(self.collected_materials)
        self.info_tag_elapsed_agent_time.text = "Time:" + str(self.elapsed_agent_time)

        if self.elapsed_agent_time < self.elapsed_agent_time_end:

            #prepare collision detection status for agents as an array
            self.agent_collision_status = numpy.full(len(self.agents), MaterialCollectingAgentParameters.SENSE_NOT_COLLIDED)

            for agent, index in zip(self.agents, range(len(self.agent_positions))):
                action = agent.params.action

                ## keep previous position for collision check
                prev_pos = self.agent_positions[index]
                next_pos = self.agent_positions[index]

                ## update positions and status
                if action == MaterialCollectingAgentParameters.ACT_GO_FORWARD:
                    direction = self.agent_directions[index]
                    dir_vec = numpy.array([numpy.sin(numpy.deg2rad(direction)),numpy.cos(numpy.deg2rad(direction))])
                    velocity = numpy.clip(agent.params.velocity, -MaterialCollectingAgentParameters.MAX_VELOCITY, MaterialCollectingAgentParameters.MAX_VELOCITY)
                    mag = dt * velocity
                    dir_vec = dir_vec *  mag
                    
                    next_pos = self.agent_positions[index] + dir_vec

                elif action == MaterialCollectingAgentParameters.ACT_ROTATE:
                    direction = self.agent_directions[index]
                    angular_velocity = numpy.clip(agent.params.angular_velocity, -MaterialCollectingAgentParameters.MAX_ANGULAR_VELOCITY, MaterialCollectingAgentParameters.MAX_ANGULAR_VELOCITY)
                    next_dir = direction + dt * angular_velocity
                    self.agent_directions[index] = next_dir
                elif action == MaterialCollectingAgentParameters.ACT_STANDSTILL:
                    pass

                ### collision check and update positions
                #### collision check between agents(circle and circle)
                for index2 in range(len(self.agent_positions)):
                    if index != index2:
                        pos2 = self.agent_positions[index2]
                        dist = numpy.linalg.norm(next_pos - pos2)
                        if dist <= self.agent_size * 2:
                            next_pos = prev_pos
                            ##update collision status
                            self.agent_collision_status[index] = MaterialCollectingAgentParameters.SENSE_COLLIDED
                            self.agent_collision_status[index2] = MaterialCollectingAgentParameters.SENSE_COLLIDED

                #### collision check between current agent and obstacles
                for obst, index2 in zip(self.object_shapes, range(len(self.object_shapes))):
                    collis = geo.circle_rect_collision(next_pos[0], next_pos[1], self.agent_size, obst.x, obst.y, obst.width, obst.height)
                    if collis:
                        next_pos = prev_pos
                        ##update collision status
                        self.agent_collision_status[index] = MaterialCollectingAgentParameters.SENSE_COLLIDED
                

                self.agent_positions[index] = next_pos

                ## update agent shape information
                
                self.agent_shapes[index][0].x = self.agent_positions[index][0]
                self.agent_shapes[index][0].y = self.agent_positions[index][1]

                self.agent_shapes[index][1].x = self.agent_positions[index][0]
                self.agent_shapes[index][1].y = self.agent_positions[index][1]

                self.agent_shapes[index][2].x = self.agent_positions[index][0]
                self.agent_shapes[index][2].y = self.agent_positions[index][1]

                self.agent_shapes[index][3].x = self.agent_positions[index][0]
                self.agent_shapes[index][3].y = self.agent_positions[index][1]

                self.agent_shapes[index][4].x = self.agent_positions[index][0]
                self.agent_shapes[index][4].y = self.agent_positions[index][1]

                self.agent_shapes[index][0].rotation = self.agent_directions[index]
                self.agent_shapes[index][1].rotation = self.agent_directions[index]
                self.agent_shapes[index][2].rotation = self.agent_directions[index]
                self.agent_shapes[index][3].rotation = self.agent_directions[index]
                self.agent_shapes[index][4].rotation = self.agent_directions[index]

                self.agent_tags[index].x = self.agent_positions[index][0]
                self.agent_tags[index].y = self.agent_positions[index][1] + self.agent_size * 2

                if debug_mode:
                    position = self.agent_positions[index]
                    direction_angle = self.agent_directions[index]

                    for i in range(MaterialCollectingAgentParameters.NUM_SENSOR):

                        delta_theta = self.sensor_angle / MaterialCollectingAgentParameters.NUM_SENSOR
                        current_direction_angle = direction_angle + delta_theta * ( i - (MaterialCollectingAgentParameters.NUM_SENSOR - 1) / 2) 
                        direction = [numpy.sin(numpy.deg2rad(current_direction_angle)), numpy.cos(numpy.deg2rad(current_direction_angle))]

                        obst_dist_min = self.sensor_range_radius
                        for obst, index2 in zip(self.object_shapes, range(len(self.object_shapes))):
                            rect_x_min = obst.x
                            rect_x_max = obst.x + obst.width
                            rect_y_min = obst.y
                            rect_y_max = obst.y + obst.height

                            obst_dist = geo.get_ray_rect_intersection_distance(position[0], position[1], direction[0], direction[1], rect_x_min, rect_y_min, rect_x_max, rect_y_max)
                            if obst_dist >= 0:
                                obst_dist_min = min(obst_dist, obst_dist_min)


                        self.dist_line_shapes[index][i].x = position[0]
                        self.dist_line_shapes[index][i].y = position[1]
                        self.dist_line_shapes[index][i].x2 = position[0] + numpy.sin(numpy.deg2rad(current_direction_angle)) * obst_dist_min
                        self.dist_line_shapes[index][i].y2 = position[1] + numpy.cos(numpy.deg2rad(current_direction_angle)) * obst_dist_min



            if self.agent_update_ready:

                #record collected materials
                self.collected_materials_time_series[self.elapsed_agent_time] = self.collected_materials
                #record collected materials for each agent
                self.collected_materials_time_series_for_each_agent[self.elapsed_agent_time, :] = self.collected_materials_sum_for_each_agent

                self.elapsed_agent_time += 1

                ## material collection
                for agent, index in zip(self.agents, range(len(self.agent_positions))):
                    for mat, index2 in zip(self.material_shapes, range(len(self.material_shapes))):
                        distance = numpy.linalg.norm(self.agent_positions[index] - numpy.array([mat[0].x, mat[0].y]))
                        mat_range = self.material_shapes[index2][0].radius
                        if distance <= mat_range:
                            ##collect material
                            if self.material_amounts[index2] > 0:
                                self.material_amounts[index2] -= 1
                                self.collected_materials += 1
                                self.collected_materials_sum_for_each_agent[index] += 1

                ## remove material points if they are all collected
                for mat, index2 in zip(reversed(self.material_shapes), reversed(range(len(self.material_shapes)))):
                    if self.material_amounts[index2] == 0:
                        self.material_shapes.pop(index2)
                        self.material_amounts.pop(index2)

                ## add new material points
                for i in range(self.num_max_material - len(self.material_shapes)):
                    self.add_material()

                ## update material's opacity
                for mat, index3 in zip(self.material_shapes, range(len(self.material_shapes))):
                    mat[0].opacity = int((float(self.material_amounts[index3]) / self.material_amount_max) * (self.material_opacity_max - self.material_opacity_min)) + self.material_opacity_min

                ## update communication information between agents
                for agent, index in zip(self.agents, range(len(self.agent_positions))):
                    agent.params.received_messages = []
                    for agent2, index2 in zip(self.agents, range(len(self.agent_positions))):
                        if index != index2:
                            if numpy.linalg.norm(self.agent_positions[index] - self.agent_positions[index2]) <= self.communication_range_radius:
                                agent.params.received_messages.append(agent2.params.communication_message)                           

                ## update agent sensor information and action
                for agent, index in zip(self.agents, range(len(self.agent_positions))):

                    ##initialize agent sensor information
                    for i in range(MaterialCollectingAgentParameters.NUM_SENSOR):
                        agent.params.sensor_object_distance[i] = self.sensor_range_radius
                        agent.params.sensor_object_type[i] = MaterialCollectingAgentParameters.SENSE_NONE
                        agent.params.sensor_object_attribute[i] = 0

                        agent.params.collision_sensor = MaterialCollectingAgentParameters.SENSE_NOT_COLLIDED

                    ## pass sensor data to the agent
                    ###
                    ## collision information
                    agent.params.collision_sensor = self.agent_collision_status[index]

                    ##get agent's forward vector
                    position = self.agent_positions[index]
                    direction_angle = self.agent_directions[index]
                    
                    delta_theta = self.sensor_angle / MaterialCollectingAgentParameters.NUM_SENSOR
                    delta_theta_rad = numpy.deg2rad(delta_theta)

                    for i in range(MaterialCollectingAgentParameters.NUM_SENSOR):
                        
                        ## obstacles
                        current_direction_angle = direction_angle + delta_theta * ( i - (MaterialCollectingAgentParameters.NUM_SENSOR - 1) / 2) 

                        ####direction in pyglet:(0,1) at angle 0. crockwise
                        direction = [numpy.sin(numpy.deg2rad(current_direction_angle)), numpy.cos(numpy.deg2rad(current_direction_angle))]

                        obst_dist_min = self.sensor_range_radius
                        for obst, index2 in zip(self.object_shapes, range(len(self.object_shapes))):
                            rect_x_min = obst.x
                            rect_x_max = obst.x + obst.width
                            rect_y_min = obst.y
                            rect_y_max = obst.y + obst.height

                            obst_dist = geo.get_ray_rect_intersection_distance(position[0], position[1], direction[0], direction[1], rect_x_min, rect_y_min, rect_x_max, rect_y_max)
                            if obst_dist >= 0:
                                obst_dist_min = min(obst_dist, obst_dist_min)
                        
                        ##update the sensor value if the distance is smaller than current distance
                        if agent.params.sensor_object_distance[i] > obst_dist_min:
                            agent.params.sensor_object_distance[i] = obst_dist_min
                            agent.params.sensor_object_type[i] = MaterialCollectingAgentParameters.SENSE_OBSTACLE

                        ##material
                        mat_dist_min = self.sensor_range_radius
                        target_mat_radius = self.material_radius_max
                        for mat, index2 in zip(self.material_shapes, range(len(self.material_shapes))):
                            mat_radius = mat[0].radius
                            mat_x = mat[0].x
                            mat_y = mat[0].y
                            pos_diff = numpy.array([mat_x, mat_y]) - position
                            angle = geo.get_angle(pos_diff, direction)
                            
                            if angle <= delta_theta_rad/2:###sensor direction is center of sensor division
                                ##material center is between its sensor range angle. Next, check the distance.
                                distance = numpy.linalg.norm(pos_diff)
                                if distance < mat_dist_min:
                                    mat_dist_min = distance
                                    target_mat_radius = mat_radius

                        if agent.params.sensor_object_distance[i] > mat_dist_min:
                            agent.params.sensor_object_distance[i] = mat_dist_min
                            agent.params.sensor_object_type[i] = MaterialCollectingAgentParameters.SENSE_MATERIAL
                            agent.params.sensor_object_attribute[i] = target_mat_radius

                        ##agent
                        agent_dist_min = self.sensor_range_radius
                        agent_id_dist_min = -1
                        for ag, index2 in zip(self.agents, range(len(self.agent_positions))):
                            if index != index2:    
                                position2 = self.agent_positions[index2]
                                ag_x = position2[0]
                                ag_y = position2[1]
                                pos_diff = numpy.array([ag_x, ag_y]) - position
                                angle = geo.get_angle(pos_diff, direction)

                                if angle <= delta_theta_rad / 2:###sensor direction is center of sensor division
                                    ##material center is between its sensor range angle. Next, check the distance.
                                    distance = numpy.linalg.norm(pos_diff)
                                    if distance < agent_dist_min:
                                        agent_dist_min = distance
                                        agent_id_dist_min = ag.id

                        if agent.params.sensor_object_distance[i] > agent_dist_min:
                            agent.params.sensor_object_distance[i] = agent_dist_min
                            agent.params.sensor_object_type[i] = MaterialCollectingAgentParameters.SENSE_AGENT
                            agent.params.sensor_object_attribute[i] = agent_id_dist_min              

                    
                    ## have the agent acts
                    agent.act()

        elif self.elapsed_agent_time == self.elapsed_agent_time_end and self.experiment_end is False:
            self.save_agent_statistics()
            self.experiment_end = True
            self.request_reset = True




        

    def register_agent(self, agent):
        """
        Register an agent and set its initial position, direction, and shape. 
        Check for overlaps with other agents and obstacles, and handle the registration of the agent's shape and tags. 
        If in debug mode, create line shapes for each sensor of the agent. 
        """
        #check agent class
        class_check = isinstance(agent, MaterialCollectingAgent)
        if class_check:

            #prepare time series data
            if self.collected_materials_time_series_for_each_agent is None:
                self.collected_materials_time_series_for_each_agent = numpy.zeros((self.elapsed_agent_time_end, 1))
            else:
                #add a new column
                self.collected_materials_time_series_for_each_agent = numpy.column_stack((self.collected_materials_time_series_for_each_agent, numpy.zeros((self.elapsed_agent_time_end, 1))))

            self.collected_materials_sum_for_each_agent.append(0)

            #register and set id
            super().register_agent(agent)

            #position initialization and overlap check
            ini_pos_x = ini_pos_y = 0
            for i in range(1000):###maximum retry count is 1000

                no_overlap = True

                ini_pos_x = numpy.random.random() * (self.frame_x - self.agent_size * 4) + self.agent_size * 2
                ini_pos_y = numpy.random.random() * (self.frame_y - self.agent_size * 4) + self.agent_size * 2

                ##overlap between agents
                for pos in self.agent_positions:
                    dist = numpy.linalg.norm(pos - numpy.array([ini_pos_x, ini_pos_y]))
                    if dist < self.agent_size * 2:
                        no_overlap = False

                ##overlap between agent and obstacle
                for obs in self.object_shapes:
                    collision = geo.circle_rect_collision(ini_pos_x,ini_pos_y,self.agent_size, obs.x, obs.y, obs.width, obs.height)
                    if collision:
                        no_overlap = False
                
                if no_overlap:
                    break


            initial_pos = numpy.array((ini_pos_x, ini_pos_y))
            
            self.agent_positions.append(initial_pos)
            initial_angle = numpy.random.random() * 360
            self.agent_directions.append(initial_angle)

            agent_pos = self.agent_positions[-1]
            agent_dir = self.agent_directions[-1]

            agent_shape = shapes.Circle(agent_pos[0], agent_pos[1], self.agent_size, color=self.agent_edge_color, batch=self.batch, group=self.main_group)
            agent_shape2 = shapes.Circle(agent_pos[0], agent_pos[1], self.agent_size * 0.8, color=self.agent_body_color, batch=self.batch, group=self.main_group)
            
            agent_shape3 = shapes.Rectangle(agent_pos[0], agent_pos[1], self.dir_rec_width, self.dir_rec_height, color=self.agent_dir_color, batch=self.batch, group=self.main_group)
            agent_shape3.anchor_x = self.dir_rec_width / 2

            agent_shape4 = shapes.Circle(agent_pos[0], agent_pos[1], self.communication_range_radius, color=self.agent_com_range_color, batch=self.batch, group=self.main_group)

            agent_shape5 = shapes.Sector(agent_pos[0], agent_pos[1], self.sensor_range_radius, angle=self.sensor_angle, start_angle=90-self.sensor_angle/2, color=self.agent_sensor_range_color,batch=self.batch, group=self.main_group)

            agent_tag = pyglet.text.Label("ID:" + str(agent.id), font_name='Times New Roman', font_size=14, x=agent_pos[0], y=agent_pos[1] + self.agent_size*2,anchor_x='center', anchor_y='center',batch=self.batch, group=self.main_group)
            
            agent_shape.rotation = agent_dir
            agent_shape2.rotation = agent_dir
            agent_shape3.rotation = agent_dir
            agent_shape4.rotation = agent_dir
            agent_shape5.rotation = agent_dir

            agent_shape4.opacity = self.agent_com_range_opacity
            agent_shape5.opacity = self.agent_sensor_range_opacity
            
            self.agent_shapes.append([agent_shape, agent_shape2, agent_shape3, agent_shape4, agent_shape5])
            self.agent_tags.append(agent_tag)

            if debug_mode:

                agent_line_shapes = []
                for i in range(agent.params.NUM_SENSOR):

                    line_shape = shapes.Line(ini_pos_x, ini_pos_y, ini_pos_x, ini_pos_y, 1, (0,0,0), batch=self.batch)
                    agent_line_shapes.append(line_shape)
                
                self.dist_line_shapes.append(agent_line_shapes)

    def add_material(self):
        #position initialization and overlap check
        mat_pos_x = mat_pos_y = 0
        for i in range(1000):###maximum retry count is 1000

            no_overlap = True
        
            mat_pos_x = numpy.random.random() * (self.frame_x - self.material_pos_margin * 2) + self.material_pos_margin
            mat_pos_y = numpy.random.random() * (self.frame_y - self.material_pos_margin * 2) + self.material_pos_margin

            ##overlap between material center and obstacle
            for obs in self.object_shapes:
                collision = geo.point_rect_collision(obs.x, obs.y, obs.width, obs.height, mat_pos_x, mat_pos_y)
                if collision:
                    no_overlap = False
            
            if no_overlap:
                break

        #register shape
        material_radius = numpy.random.random() * (self.material_radius_max - self.material_radius_min) + self.material_radius_min
        material_amount = numpy.random.randint(self.material_amount_min, self.material_amount_max)
        mat_shape1 = shapes.Circle(mat_pos_x, mat_pos_y, material_radius, color=self.material_color, batch=self.batch, group=self.background_group)
        mat_shape1.opacity = int((float(material_amount) / self.material_amount_max) * (self.material_opacity_max - self.material_opacity_min)) + self.material_opacity_min
        mat_shape2 = shapes.Circle(mat_pos_x, mat_pos_y, self.material_center_radius, color=self.material_center_color, batch=self.batch, group=self.background_group)
        mat_shape2.opacity = self.material_center_opacity
        self.material_shapes.append([mat_shape1, mat_shape2])
        self.material_amounts.append(material_amount)        

    def delete_shapes(self):
        super().delete_shapes()

        ##delete all obstacles
        for obs in self.object_shapes:
            obs.delete()
        
        ##delete all materials
        for mat in self.material_shapes:
            mat[0].delete()
            mat[1].delete()
        
        ##delete all agents
        for agent_s in self.agent_shapes:
            agent_s[0].delete()
            agent_s[1].delete()
            agent_s[2].delete()
            agent_s[3].delete()
            agent_s[4].delete()

        for agent_t in self.agent_tags:
            agent_t.delete()

        ##delete all line shapes
        if debug_mode:
            for line_s in self.dist_line_shapes:
                for line in line_s:
                    line.delete()

        ##delete all info tags
        for info_t in self.info_tags:
            info_t.delete()



    def register_shapes(self, batch):
        """
        Register shapes for walls, obstacles, and materials in the given batch.
        """
        super().register_shapes(batch)

        self.batch = batch

        ## information
        self.info_tag_material_amount = pyglet.text.Label("Amount:" + str(self.collected_materials), font_name='Times New Roman', font_size=14, x=0, y=10,anchor_x='left', anchor_y='bottom',batch=batch, group=self.foreground_group)
        self.info_tag_elapsed_agent_time = pyglet.text.Label("Time:" + str(self.elapsed_agent_time), font_name='Times New Roman', font_size=14, x=0, y=30,anchor_x='left', anchor_y='bottom',batch=batch, group=self.foreground_group)
        self.info_tags.append(self.info_tag_material_amount)
        self.info_tags.append(self.info_tag_elapsed_agent_time)

        ## object shapes
        ### walls of window edge
        wall_width = 1000
        wall_n = shapes.Rectangle(0, self.frame_y, self.frame_x, wall_width, color=(0,0,0),batch=batch,group=self.background_group)
        wall_e = shapes.Rectangle(self.frame_x, 0, wall_width, self.frame_y, color=(0,0,0),batch=batch,group=self.background_group)
        wall_w = shapes.Rectangle(-wall_width, 0, wall_width, self.frame_y, color=(0,0,0),batch=batch,group=self.background_group)
        wall_s = shapes.Rectangle(0, -wall_width, self.frame_x, wall_width, color=(0,0,0),batch=batch,group=self.background_group)

        self.object_shapes.append(wall_n)
        self.object_shapes.append(wall_e)
        self.object_shapes.append(wall_w)
        self.object_shapes.append(wall_s)
        
        ### obstacles
        for i in range(self.num_max_obstacle):
            if numpy.random.random() > self.obstacle_prob:

                obs_pos_x = numpy.random.random() * (self.frame_x - self.obstacle_pos_margin * 2) + self.obstacle_pos_margin
                obs_pos_y = numpy.random.random() * (self.frame_y - self.obstacle_pos_margin * 2) + self.obstacle_pos_margin
                obs_w = numpy.random.random() * (self.obstacle_width_max - self.obstacle_width_min) + self.obstacle_width_min
                obs_h = numpy.random.random() * (self.obstacle_height_max - self.obstacle_height_min) + self.obstacle_height_min

                #register shape
                obs_shape = shapes.Rectangle(obs_pos_x, obs_pos_y, obs_w, obs_h, color=(200,200,200),batch=batch,group=self.background_group)
                self.object_shapes.append(obs_shape)

        ### materials
        for i in range(self.num_max_material):
            self.add_material()

    def save_agent_statistics(self):
        """
        Save agent statistics to a CSV file with a timestamp as part of the filename.
        """
        time_str = time.strftime("%Y%m%d-%H%M%S")
        filename = "agent_statistics_" + time_str + ".csv"
        data = numpy.column_stack((self.collected_materials_time_series, self.collected_materials_time_series_for_each_agent))
        numpy.savetxt(filename, data, delimiter=",", fmt="%d")

            


