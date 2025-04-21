import math
import random
import string
import keyboard
from controller import Robot
import numpy as np
from astar.search import AStar
import time
MAX_SPEED = 7.0
WHEEL_DISTANCE = 0.10
MAP_SIZE = 100
MAP_RESOLUTION = 0.05
MAX_COMM_RANGE = 2
WHEEL_RADIUS = 0.042
SWARM_SIZE = 9


GOAL = [-1.94,-0.21]
BASE = [2,2]
 

class SwarmRobot():
    def __init__(self):
        self.robot = Robot()
        self.ID = ''.join(random.choice(string.digits + string.ascii_lowercase) for _ in range(16))
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motor_r = self.robot.getDevice('motor_r')
        self.motor_l = self.robot.getDevice('motor_l')
        self.lidar = self.robot.getDevice('lidar')
        self.gps = self.robot.getDevice('gps')
        self.imu = self.robot.getDevice('imu')
        self.emitter = self.robot.getDevice('emitter')
        self.receiver = self.robot.getDevice('receiver')
        self.display = self.robot.getDevice('display')
        self.led = self.robot.getDevice('led')
        self.camera = self.robot.getDevice('camera')
        self.pen = self.robot.getDevice('pen')
        
        self.memorized_path = []
        
        self.ds_sensors = []
        
        self.pen.setInkColor(0x00FF00, 1)
        self.pen.leadSize = 1
        self.pen.write(True)
        self.isTargetReached = False
        self.memorized_pos_reached = False
        self.memorized_path_index  = 0
        self.max_path_length = 10
        self.swarm_class = "Keeper"
        self.path_library = []
        self.last_position = [0,0,0]
        self.victim_found = False
        
        #self.queen_best_path2 = [
        #[[10, 7.713501243541234, [[1.1664102809527932, 2.210813419426397, 0.021859187285718697], [2.302022744908678, 2.2446852403351785, 0.02185905830272154], [1.1645589417260858, 2.1517476985801456, 0.021859208904410468], [2.3055026429835404, 2.2521579699217025, 0.021859233437849915], [1.1668774080682256, 2.225282622330517, 0.02185911711382772], [1.954223613844069, 2.306116609109526, 0.02185930734289966], [2.3023113418926204, 2.265572376070636, 0.021859257164729553], [1.2040380414064853, 2.303520373352087, 0.021859236369379996], [1.1559107538361408, 1.7774524683273925, 0.021859160426775538], [1.164581944967862, 1.3948364286158812, 0.021859110100878522]], 'failed', 'not_investigated']]
        #]
        
        self.queen_best_path = None
        self.queen_best_path_index = 0
        self.path_investigated = False
        
        for i in range(5):
            self.ds_sensors.append(self.robot.getDevice('ds'+str(i)))
            
            
        
        self.motor_l.setPosition(float('inf'))
        self.motor_r.setPosition(float('inf'))
        
        self.lidar.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.receiver.enable(self.timestep)
        #self.camera.recognitionEnable(self.timestep)
        self.led.set(0)
        
        for ds in self.ds_sensors:
            ds.enable(self.timestep)
        
        self.state = 0
        self.command = 0
        self.point_of_interest = [0,0,0]
        
        self.map = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]
        
    def centroid(self, robots):
        try:
            return [sum(pos[i] for pos in (robot["pos"] for robot in robots)) / len(robots) for i in range(3)]
        
        except:
            return [0,0,0]   
            
    def turn_to_point(self, target, polarity=1):
        pos, rot = self.getPose()
        angle_to_target = self.angle(pos, target)
        
        if polarity == 1:
            angle_diff = (angle_to_target - rot[2]) + math.pi
        else:
            angle_diff = (angle_to_target - rot[2])
        
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) > 3:
            self.stop()
            return 1
            
        elif abs(angle_diff) < 1:
            self.motor_l.setVelocity(-10)
            self.motor_r.setVelocity(10)
            
            return 0
        else:
            self.drive(MAX_SPEED * angle_diff, -MAX_SPEED * angle_diff)
            return 0
    
    def flip_id(self, alphanumeric_id):
        return alphanumeric_id[::-1]
        
    def isMoving(self):
        current_position, _ = self.getPose()
        if self.distance(current_position, self.last_position) < 0.001:
            self.last_position = current_position
            return False
        else:
            self.last_position = current_position
            return True
    
    def getVelocity(self):
        return [self.motor_l.getVelocity(), self.motor_r.getVelocity()]    
    def move_to_point(self, target, dist=0.3):
        pos, rot = self.getPose()
        if self.turn_to_point(target, 1) == 1:
            if self.distance(pos, target) > dist:
                self.motor_l.setVelocity(10)
                self.motor_r.setVelocity(10)
            else:
                self.stop()
                
    
    
    def move_away_from_point(self, target, dist=0.3):
        pos, rot = self.getPose()
        if self.turn_to_point(target, 0) == 1:
            if self.distance(pos, target) < dist:
                self.motor_l.setVelocity(10)
                self.motor_r.setVelocity(10)
            else:
                self.stop()
              
            

    def turn(self, degrees, direction=1):
        wheel_travel_distance = (math.pi * WHEEL_DISTANCE * degrees) / 360.0
        speed = MAX_SPEED if direction > 0 else -MAX_SPEED
        self.drive(speed+1, -speed+1)
        time_to_turn = abs(wheel_travel_distance / (WHEEL_RADIUS * MAX_SPEED))
        self.robot.step(int(time_to_turn * 1000))
    
    def move(self, dist, direction=1):
        time = dist / MAX_SPEED
        speed = MAX_SPEED if direction > 0 else -MAX_SPEED
        self.drive(speed+1, speed+1)
        self.robot.step(int(time_to_turn * 1000))
                

    def distance(self, pos_1, pos_2):
        return math.sqrt((pos_2[0] - pos_1[0])**2 + (pos_2[1] - pos_1[1])**2)
    
    def angle(self, pos_1, pos_2):
        return math.atan2(pos_2[1] - pos_1[1],pos_2[0] - pos_1[0])
        
    def getPose(self):
        pos = self.gps.getValues()
        rot = self.imu.getRollPitchYaw()
        
        return pos, rot    
    
    def drive(self, left_speed, right_speed):
        l_speed, r_speed = 0, 0
        if abs(left_speed) > MAX_SPEED:
            l_speed = math.copysign(MAX_SPEED, left_speed)
        if abs(right_speed) > MAX_SPEED:
            r_speed = math.copysign(MAX_SPEED, right_speed)
        
        self.motor_l.setVelocity(l_speed)
        self.motor_r.setVelocity(r_speed)
        
    def broadcast(self, message):
        self.emitter.send(str(message).encode())
        
    def isHighestRankedKeeper(self, neighbours):
        return all(int(self.ID, 36) > int(n["id"], 36) for n in neighbours)
    
    def isFarthestFromBaseKeeper(self, neighbours):
        pos, _ = self.getPose()
        return all(self.distance(pos, BASE) > self.distance(n["pos"], BASE) for n in neighbours)
    
    def isFarthestFromCentroidKeeper(self, neighbours):
        pos, _ = self.getPose()
        return all(self.distance(pos, self.centroid(neighbours)) > self.distance(n["pos"], self.centroid(neighbours)) for n in neighbours)
           
    
           
    def receive(self):
        
        neighbours = []
        self.neighbour_positions = []
        self.keeper_neighbours = []
        self.recruit_neighbours = []
        self.scout_neighbours = []
        pos, rot = self.getPose()
    
        while self.receiver.getQueueLength() > 0:
            recv_data = self.receiver.getString()
            self.receiver.nextPacket()
            data_list = eval(recv_data)
            
            neighbour = {
                "id": data_list[0],
                "pos": data_list[1],
                "rot": data_list[2],
                "state": data_list[3],
                "cmd": data_list[4],
                "class": data_list[5],
                "path": data_list[6],
            }
            
            
            if self.distance(pos, neighbour["pos"]) < MAX_COMM_RANGE:
                neighbours.append(neighbour)
                self.neighbour_positions.append(neighbour["pos"])
                
                if neighbour["class"] == "Keeper":
                    self.keeper_neighbours.append(neighbour)
                    
                
                if neighbour["class"] == "Scout":
                    self.scout_neighbours.append(neighbour)
                   
                    
                if neighbour["class"] == "Crawler":
                    self.recruit_neighbours.append(neighbour)
                   

        
        if self.distance(pos, BASE) < 1 and self.checkRoyalPresence(self.keeper_neighbours) == False and self.isHighestRankedKeeper(neighbours) == True:
            self.swarm_class = "Queen"
            
        elif self.distance(pos, BASE) < 1 and self.isFarthestFromBaseKeeper(self.keeper_neighbours) == True and self.swarm_class != "Queen" and len(self.scout_neighbours) <= 1:
            self.swarm_class = "Scout"
                
        elif self.distance(pos, BASE) < 1 and self.isFarthestFromBaseKeeper(self.keeper_neighbours) == True and self.swarm_class != "Queen" and len(self.recruit_neighbours) <= 0 and len(self.scout_neighbours) == 2 and self.state != "base_return":
            self.swarm_class = "Crawler"
            
        elif self.distance(pos, BASE) > 1 and self.swarm_class == "Crawler" and self.state != "base_return":
            pass
            
        elif self.distance(pos, BASE) > 1 and self.swarm_class == "Scout":
            self.swarm_class = "Scout"
            
        else:
            if self.state == "base_return":
                pass
            else:
                self.swarm_class = "Keeper"
        
        
          
                            
            
            
             
        for neighbour in neighbours:  
            if neighbour["class"] == "Scout":
                if self.swarm_class == "Queen":
                    if neighbour["state"] == "base_return":
                        if not any(neighbour["path"] == path for path in self.path_library):
                            if neighbour["path"] == []:
                                pass
                            else:
                                self.path_library.append(neighbour["path"])
                                
                              
        
              
        for neighbour in neighbours:
            if neighbour["class"] == "Queen":
                if self.swarm_class == "Crawler":
                    self.queen_best_path = neighbour["path"]

                
                    
                    
            
            
    def wait(self, wait_time_seconds=1):
        """
        Make the robot wait for the specified amount of time using timesteps.
    
        Parameters:
            robot (Robot): The robot instance.
            wait_time_seconds (float): The amount of time to wait in seconds.
        """
        time_step = int(self.robot.getBasicTimeStep())
        wait_time_steps = int(wait_time_seconds * 1000 / time_step)
    
        while self.robot.step(time_step) != -1:
            if wait_time_steps > 0:
                wait_time_steps -= 1
            else:
                break     
 
        
            
    
    def checkRoyalPresence(self, neighbours):
        return any(n['class'] == 'Queen' for n in neighbours)
        
       
    def stop(self):
        self.motor_l.setVelocity(0.0)
        self.motor_r.setVelocity(0.0)
    
    def getLidarData(self):
        return self.lidar.getRangeImage()
        
    def getDistanceData(self):
       vals = []
       
       for ds in self.ds_sensors:
           vals.append(ds.value)
       
       return vals
    
    def patrol(self):
        threshold = 999
        pos, rot = self.getPose()
        ds_data = self.getDistanceData()
        
        if self.distance(pos, GOAL) < 0.5 or len(self.memorized_path) == self.max_path_length:
            self.stop()
            self.isTargetReached = True
            if self.distance(pos, GOAL) < 0.5:
                self.victim_found = True
            self.rememberPose()
        else:
            self.isTargetReached = False
            if ds_data[0] < 999:
                self.motor_l.setVelocity(10.0)
                self.motor_r.setVelocity(-10.0)
                self.rememberPose()
            elif ds_data[1] < 999:
                self.motor_l.setVelocity(-10.0)
                self.motor_r.setVelocity(10.0)
                self.rememberPose()
            elif ds_data[2] < 999:
                self.motor_l.setVelocity(10.0)
                self.motor_r.setVelocity(-10.0)
                self.rememberPose()
            elif ds_data[3] < 999:
                self.motor_l.setVelocity(-10.0)
                self.motor_r.setVelocity(10.0)
                self.rememberPose()
            elif ds_data[4] < 999:
                self.motor_l.setVelocity(10.0)
                self.motor_r.setVelocity(-10.0)
                self.rememberPose()
            else:
                self.motor_l.setVelocity(10.0)
                self.motor_r.setVelocity(10.0)
        
    
    def obstacle_avoidance(self):
        threshold = 999
        ds_data = self.getDistanceData()
        
        if ds_data[0] < 999:
            self.motor_l.setVelocity(10.0)
            self.motor_r.setVelocity(-10.0)
        elif ds_data[1] < 999:
            self.motor_l.setVelocity(-10.0)
            self.motor_r.setVelocity(10.0)
        elif ds_data[2] < 999:
            self.motor_l.setVelocity(10.0)
            self.motor_r.setVelocity(-10.0)
        elif ds_data[3] < 999:
            self.motor_l.setVelocity(-10.0)
            self.motor_r.setVelocity(10.0)
        elif ds_data[4] < 999:
            self.motor_l.setVelocity(10.0)
            self.motor_r.setVelocity(-10.0)
            
        else:
            self.motor_l.setVelocity(10.0)
            self.motor_r.setVelocity(10.0)
    
    
    
    def obstacle_avoidance2(self):
        threshold = 999
        ds_data = self.getDistanceData()
        
        if ds_data[0] < 999:
            self.turn(20, 1)
        elif ds_data[1] < 999:
            self.turn(20, -1)
        elif ds_data[2] < 999:
            self.turn(20, 1)
        elif ds_data[3] < 999:
            self.turn(20, -1)
        elif ds_data[4] < 999:
            self.turn(20, 1)
            
        else:
            self.motor_l.setVelocity(10.0)
            self.motor_r.setVelocity(10.0)
    
    
    
    def distBug(self, target):
        threshold = 650
        ds_data = self.getDistanceData()
        front = ds_data[0]
        right = ds_data[1]
        left = ds_data[2]
        
        if front < threshold or left < threshold or right < threshold:
            self.wall_follow()
        else:
            self.move_to_point(target, 0.2)
        
    
    def wall_follow(self):

        desired_dist = 0.6
        ds_data = self.getDistanceData()
        threshold = 600
        
        front = ds_data[0]
        right = ds_data[1]
        left = ds_data[2]
        pos, rot = self.getPose()
        
        if self.distance(pos, GOAL) < 0.5 or len(self.memorized_path) == self.max_path_length:
            self.stop()
            self.isTargetReached = True
            self.rememberPose()
        else:
            self.isTargetReached = False
            if self.obstacleDetected() == True:
                if front < threshold:
                    self.drive(MAX_SPEED+5, -MAX_SPEED+5)
                    self.rememberPose()
                elif left < threshold-100:
                    self.drive(MAX_SPEED+5, -MAX_SPEED+5)
                    self.rememberPose()
                elif left > threshold:
                    self.drive(-MAX_SPEED+5, MAX_SPEED+5)
                    self.rememberPose()
                else:
                    self.drive(MAX_SPEED+5, MAX_SPEED+5)
            else:
                self.drive(MAX_SPEED+5, MAX_SPEED+5)

    
        

    
    
    
    
    def obstacleDetected(self):
        ds_data = self.getDistanceData()
        if ds_data[0] < 999 or ds_data[1] < 999 or ds_data[2] < 999 or ds_data[3] < 999 or ds_data[4] < 999:
            return True
        else:
            return False
        
    
    def rememberPose(self):
        pos, rot = self.getPose()
        self.memorized_path.insert(0, pos)

        if len(self.memorized_path) > 1:
            if self.distance(pos, self.memorized_path[1]) < 0.3:
                self.memorized_path.pop(0)
                
                
    def investigate_path(self):
        if len(self.queen_best_path) > 0:
        
            pos, _ = self.getPose()
            if self.distance(self.queen_best_path[0][2][self.queen_best_path_index], pos) < 0.2:
                if len(self.queen_best_path[0][2]) - (self.queen_best_path_index + 1) > 0:
                    self.queen_best_path_index += 1
                else:
                    self.path_investigated = True
                    #self.queen_best_path_index = 0
                    #self.queen_best_path = None
                    self.stop()
            #print(self.queen_best_path[0][2][self.queen_best_path_index])
            
            self.move_to_point(self.queen_best_path[0][2][len(self.queen_best_path[0][2]) - self.queen_best_path_index], 0.2)
            
            
            
            
    
    
    def follow_memorized_path(self):
        
        pos, _ = self.getPose()
        if self.distance(pos, self.memorized_path[self.memorized_path_index]) < 0.05:
            if self.memorized_path_index + 1 < len(self.memorized_path):
                self.memorized_path_index += 1
            else:
                self.isTargetReached = False
                self.memorized_path_index = 0
                
                if self.victim_found == True:
                    self.path_library.append([len(self.memorized_path), self.getConsecutivePathDist(self.memorized_path), self.memorized_path, "found", "not_investigated"])
                else:
                    self.path_library.append([len(self.memorized_path), self.getConsecutivePathDist(self.memorized_path), self.memorized_path, "failed", "not_investigated"])
                
                
                self.victim_found = False
                self.memorized_path = []
                #self.path_library = []
                self.memorized_path.append(BASE)
                self.stop()
        try:
            self.move_to_point(self.memorized_path[self.memorized_path_index], 0.05)
        except:
            pass
    
    def keeper(self):
        self.led.set(5)
        self.stop()
        
        pos, _ = self.getPose()
        
        if self.obstacleDetected() == True:
            self.obstacle_avoidance()
        
        if self.distance(pos, BASE) > 0.8:
            self.move_to_point(BASE)
         
        else:
            self.obstacle_avoidance() 
        
    def queen(self):
        self.led.set(3)

        
        #print(self.path_library)
        self.move_to_point(BASE, 0.1)
        if self.obstacleDetected() == True:
            self.obstacle_avoidance()
        
        self.command = -1

        
    
    def employed(self):
        self.led.set(2)
        self.stop()
        
    def crawler(self):
        self.led.set(1)
        self.stop()
        self.state = "waiting"

        if self.path_investigated == False:
            #print(self.queen_best_path)
            if self.queen_best_path != None and self.state != "investigating":
                self.state = "investigating"
                self.investigate_path()
        else:
            self.stop()
    
    
    def scout(self):
        self.led.set(4)
        #self.stop()
        pos, _ = self.getPose()
        if self.distance(pos, BASE) > 1:
            print(self.state)
        if self.isTargetReached == False:
           
            self.state = "patrol"
            self.patrol()
        else:
            self.state = "base_return"
            self.follow_memorized_path()
            
            
            
    def getConsecutivePathDist(self, path):
        total_dist = 0
        for i in range(len(path)-1):
            total_dist += self.distance(path[i], path[i+1])
        return total_dist   
    
    def switch_class(self):
        if self.swarm_class == "Keeper":
            self.keeper()
        elif self.swarm_class == "Employed":
            self.employed()
        elif self.swarm_class == "Scout":
            self.scout()
        elif self.swarm_class == "Crawler":
            self.crawler()
        elif self.swarm_class == "Queen":
            self.queen()
        
     
    def run(self):
        self.stop()
        self.state = "avoidance"
        
        

        while self.robot.step(self.timestep) != -1:
            ldr_data = self.getLidarData()
            pos, rot = self.getPose()
            
            
            #self.swarm_class = "Keeper"
            #self.led.set(5)
            self.switch_class()
            
            if self.swarm_class == "Crawler":
                print("crawler's path:", self.queen_best_path)
                #print("crawler's state:", self.state)

            if self.swarm_class == "Queen":
            
                #print(self.path_library)
                print("path count:", len(self.path_library))
                                
                successful_paths = [path for path in self.path_library if path[0][3] == "found"]
                if len(successful_paths) > 0:
                    best_path = min(successful_paths, key=lambda x: x[0][1])
                    #print("best path:", best_path)
                    message = [self.ID, pos, rot, self.state, self.command, self.swarm_class, best_path]   
                else:
                    message = [self.ID, pos, rot, self.state, self.command, self.swarm_class, self.path_library]
            else:
                message = [self.ID, pos, rot, self.state, self.command, self.swarm_class, self.path_library]
            self.broadcast(message)
            self.receive()
            

robot = SwarmRobot()
robot.run()