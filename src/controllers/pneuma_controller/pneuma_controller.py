import math
import random
import string
import time
import numpy as np
import keyboard
from astar.search import AStar
from controller import Robot


# CONSTANTS
MAX_SPEED = 7.0
WHEEL_DISTANCE = 0.10
MAP_SIZE = 100
MAP_RESOLUTION = 0.05
MAX_COMM_RANGE = 2
WHEEL_RADIUS = 0.042
SWARM_SIZE = 9
MAX_PATH_LENGTH = 5
GOAL = [-1.94,-0.21]
BASE = [1.5,1.5]


MINIMUM_KEEPERS = 2
MAX_RECRUITS = 3
MAX_SCOUTS = 3


PATH = [
[0.84, 1.67],
[-0.03, 1.23],
[0.54, 0.43],
[-0.65, -0.62],
[-0.14, -1.06]
]


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
        self.pen = self.robot.getDevice('pen')
        
        self.ds_sensors = []
        
        for i in range(5):
            self.ds_sensors.append(self.robot.getDevice('ds'+str(i)))
        
        self.motor_l.setPosition(float('inf'))
        self.motor_r.setPosition(float('inf'))
        
        self.lidar.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.receiver.enable(self.timestep)
        self.led.set(0)
        
        for ds in self.ds_sensors:
            ds.enable(self.timestep)
        
        self.state = 0
        self.command = ["x",0]
        
        # DEFAULT ROBOT CLASS
        self.swarm_class = "keeper"
        
        
        # ROBOT NEIGHBOURS
        self.neighbours = []
        self.Queen_neighbours =[]
        self.keeper_neighbours = []
        self.recruit_neighbours = []
        self.scout_neighbours = []
        

        # SCOUT PATH FOLLOWING VARIABLES AND FLAGS
        self.memorized_path = []
        self.memorized_path_index = 0
        self.completed_scout_path = []
        
        # SCOUT PATH FOLLOWING CONDITION FLAGS
        self.isPatrolFinished = False
        self.isVictimFound = False
        
        # RECRUIT PATH INVESTIGATION VARIABLES AND FLAGS
        self.victim_path = [[-1.4424510321426105, -0.22535250301942134, 0.02185907275896693], [1.2511137129418086, -0.3138192918003388, 0.02185926248952307], [2,2]]
        self.victim_path_index = 0
        
        # RECRUIT PATH INVESTIGATION CONDITION FLAGS
        self.isVictimReached = False
        
        # QUEEN VARIABLES
        self.queen_path_library = []
        

        
        
        
        
        
        
        
        
        
        
        
        
        
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
        
   
    def rememberPose(self):
        pos, rot = self.getPose()
        self.memorized_path.insert(0, pos)

        if len(self.memorized_path) > 1:
            if self.distance(pos, self.memorized_path[1]) < 0.3:
                self.memorized_path.pop(0)
      
        
    def obstacleDetected(self):
        ds_data = self.getDistanceData()
        if ds_data[0] < 999 or ds_data[1] < 999 or ds_data[2] < 999 or ds_data[3] < 999 or ds_data[4] < 999:
            return True
        else:
            return False
            
    
    def getHighestRankedAgent(self, neighbours):
        return max(neighbours, key=lambda n: int(n["id"], 36), default=None)
    
    def isHighestRankedAgent(self, neighbours):
        return all(int(self.ID, 36) > int(n["id"], 36) for n in neighbours)
    
    def getLowestRankedAgent(self, neighbours):
        return min(neighbours, key=lambda n: int(n["id"], 36), default=None)

    
    def isFarthestFromBaseKeeper(self, neighbours):
        pos, _ = self.getPose()
        return all(self.distance(pos, BASE) > self.distance(n["pos"], BASE) for n in neighbours)
    
    
    def scout_patrol(self):
        threshold = 999
        pos, rot = self.getPose()
        ds_data = self.getDistanceData()
        
        
        if self.distance(pos, GOAL) < 0.5 or len(self.memorized_path) == MAX_PATH_LENGTH:
            self.stop()
            self.isPatrolFinished = True
            if self.distance(pos, GOAL) < 1:
                self.isVictimFound = True
            self.rememberPose()
        else:
            self.isPatrolFinished = False
            if ds_data[0] < threshold:
                self.drive(10,-10)
                self.rememberPose()
            elif ds_data[1] < threshold:
                self.drive(-10,10)
                self.rememberPose()
            elif ds_data[2] < threshold:
                self.drive(10,-10)
                self.rememberPose()
            elif ds_data[3] < threshold:
                self.drive(-10,10)
                self.rememberPose()
            elif ds_data[4] < threshold:
                self.drive(10,-10)
                self.rememberPose()
            else:
                self.drive(10,10)        
            
            
        
    def obstacle_avoidance(self):
        threshold = 999
        ds_data = self.getDistanceData()
        if ds_data[0] < threshold:
            self.drive(10,-10)
        elif ds_data[1] < threshold:
            self.drive(-10,10)
        elif ds_data[2] < threshold:
            self.drive(10,-10)
        elif ds_data[3] < threshold:
            self.drive(-10,10)
        elif ds_data[4] < threshold:
            self.drive(10,-10)
        else:
            self.drive(10,10)
            
    
    def drive(self, left_speed, right_speed):
        l_speed, r_speed = 0, 0
        if abs(left_speed) > MAX_SPEED:
            l_speed = math.copysign(MAX_SPEED, left_speed)
        if abs(right_speed) > MAX_SPEED:
            r_speed = math.copysign(MAX_SPEED, right_speed)
        
        self.motor_l.setVelocity(l_speed)
        self.motor_r.setVelocity(r_speed)        

    def stop(self):
        self.motor_l.setVelocity(0.0)
        self.motor_r.setVelocity(0.0)
        
    def getDistanceData(self):
        vals = []
       
        for ds in self.ds_sensors:
            vals.append(ds.value)
       
        return vals
        
   
    def getConsecutivePathDist(self, path):
        total_dist = 0
        for i in range(len(path)-1):
            total_dist += self.distance(path[i], path[i+1])
        return total_dist
        
   
   
    def investigate_path(self):
        pos, _ = self.getPose()
        
        reversed_path = self.victim_path[::-1]
        
        if self.distance(pos, reversed_path[self.victim_path_index]) < 0.1:
            if (self.victim_path_index  + 1) < len(reversed_path):
                self.victim_path_index += 1
            else:
                self.isVictimReached = True
                self.victim_path_index = 0
        else:
            try:
                self.move_to_point(reversed_path[self.victim_path_index], 0.1)
            except:
                pass
   
   
   
   
    def follow_memorized_path(self):
        pos, _ = self.getPose()
        if self.distance(pos, self.memorized_path[self.memorized_path_index]) < 0.05:
            if (self.memorized_path_index + 1) < len(self.memorized_path):
                self.memorized_path_index += 1
            else:
                if self.isVictimFound == True:
                    self.completed_scout_path = [self.memorized_path, self.getConsecutivePathDist(self.memorized_path), "found"]
                else:
                    self.completed_scout_path = [self.memorized_path, self.getConsecutivePathDist(self.memorized_path), "failed"]
                
                self.isPatrolFinished = False
                self.memorized_path_index = 0
                self.memorized_path = []
                self.memorized_path.append(BASE)
                
                if self.distance(pos, BASE) < 0.5:
                    self.isVictimFound = False
                    self.swarm_class = "scout"
        else:
            try:
                self.move_to_point(self.memorized_path[self.memorized_path_index], 0.05)
            except:
                pass
        
        
    def broadcast(self, message):
        self.emitter.send(str(message).encode())
        
        
    
    def receive(self):
        
        self.neighbours = []
        self.queen_neighbours = []
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
                self.neighbours.append(neighbour)
                
                if neighbour["class"] == "keeper":
                    self.keeper_neighbours.append(neighbour)
                
                elif neighbour["class"] == "recruit":
                    self.recruit_neighbours.append(neighbour)
 
                elif neighbour["class"] == "scout":
                    self.scout_neighbours.append(neighbour)
                   
                elif neighbour["class"] == "queen":
                    self.queen_neighbours.append(neighbour)
                    
                    
   
        for queen in self.queen_neighbours:
            #print("HERE!!!",queen["cmd"])
            if queen["cmd"][0] == self.ID and self.swarm_class == "keeper" and queen["cmd"][1] == "become_scout":
                self.swarm_class = "scout"
                
            elif queen["cmd"][0] == self.ID and self.swarm_class == "keeper" and queen["cmd"][1] == "become_recruit":
                self.swarm_class = "recruit"
                print("trying to be a recruit!")
        
        
        for scout in self.scout_neighbours:
            if len(scout["path"]) > 0 and self.swarm_class == "queen":
                if not any(scout["path"] == path for path in self.queen_path_library):
                    if scout["path"][2] == "found" and len(scout["path"][0]) > 1:
                        self.queen_path_library.append(scout["path"])
                
                
                

    def switch_class(self):
        pos, _ = self.getPose()
        if self.swarm_class == "keeper":
            self.led.set(5)
            if self.isHighestRankedAgent(self.neighbours) and self.distance(pos, BASE) < 1:
                self.swarm_class = "queen"
                    
        
        elif self.swarm_class == "recruit":
            self.led.set(1)
        
        elif self.swarm_class == "scout":
            self.led.set(4)
            
        elif self.swarm_class == "queen":
            self.led.set(3)
            if not self.isHighestRankedAgent(self.neighbours) or self.distance(pos, BASE) > 1:
                self.swarm_class = "keeper" 
            self.scout_order()
            
            if len(self.queen_path_library) > 0:
                self.recruit_order()
            
        else:
            self.swarm_class = "keeper"   
            
            
    
    def scout_order(self):
        if self.swarm_class == "queen":
            
            try:
                if len(self.scout_neighbours) < 2 and len(self.keeper_neighbours) > MINIMUM_KEEPERS:
                    self.command = [self.getHighestRankedAgent(self.keeper_neighbours)["id"], "become_scout"]
            except:
                pass 
        else:
            pass
    
    
    def recruit_order(self):
        if self.swarm_class == "queen":
            print(1)
            try:
                print(2)
                if len(self.recruit_neighbours) < 1 and len(self.keeper_neighbours) > MINIMUM_KEEPERS - MAX_RECRUITS:
                    print(3)
                    self.command = [self.getLowestRankedAgent(self.keeper_neighbours)["id"], "become_recruit"]
            except:
                pass 
        else:
            pass
               
    



    def do_class_task(self):
    
        if self.swarm_class == "queen":
            
            print("queen path library:", self.queen_path_library)
            print("queen path library length:", len(self.queen_path_library))
        
            self.led.set(3)
            self.state = "queen_idle"
            self.move_to_point(BASE, 0.1)
            if self.obstacleDetected() == True:
                self.obstacle_avoidance()
                
        
        elif self.swarm_class == "scout":
            self.led.set(4)
            pos, _ = self.getPose()
            
            #print("completed path:", self.completed_scout_path)
            #print("current path:", self.memorized_path)
            #print("scout state:", self.state)
            
            if self.isPatrolFinished == True or self.isVictimFound == True:
                self.state = "scout_return"
                self.follow_memorized_path()
            else:
                self.state = "scout_patrol"
                self.scout_patrol()
            
            
        
        elif self.swarm_class == "recruit":
            self.led.set(1)
            if self.isVictimReached == False:
                self.investigate_path()
            else:
                self.stop()
                
                
        elif self.swarm_class == "keeper":
            self.led.set(5)
            pos, _ = self.getPose()
            if self.obstacleDetected() == True:
                self.obstacle_avoidance()
            if self.distance(pos, BASE) > 0.5:
                self.move_to_point(BASE)
            else:
                self.obstacle_avoidance()
        else:
            self.led,set(0)
            self.stop()


        
    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.stop()
            self.switch_class()
            self.do_class_task()
            pos, rot = self.getPose()
            
            
            if self.swarm_class == "scout":
                message = [self.ID, pos, rot, self.state, self.command, self.swarm_class, self.completed_scout_path]
            else:
                message = [self.ID, pos, rot, self.state, self.command, self.swarm_class, self.memorized_path]
            
            self.broadcast(message)
            self.receive()
                
        
        
        
        
        
robot = SwarmRobot()
robot.run()
        
        