import math
from controller import Robot

MAX_SPEED = 10
GOAL_THRESHOLD = 0.3
K_ATT = 1
K_REP = 1
WHEEL_DISTANCE = 0.10
GOAL = [1,1,0.03]

class SwarmRobot():
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motor_r = self.robot.getDevice('motor_r')
        self.motor_l = self.robot.getDevice('motor_l')
        self.lidar = self.robot.getDevice('lidar')
        self.gps = self.robot.getDevice('gps')
        
        self.motor_l.setPosition(float('inf'))
        self.motor_r.setPosition(float('inf'))
        
        self.lidar.enable(self.timestep)
        self.gps.enable(self.timestep)
    
    def getPose(self):
        pos = self.gps.getValues()
        #rot = self.imu.getRollPitchYaw()
        
        return [pos]
        
    def distance(self, pos_1, pos_2):
        return math.sqrt((pos_2[0] - pos_1[0])**2 + (pos_2[1] - pos_1[1])**2)
    
        
    def repulsive_force(self, pos, obstacle_pos):
        dist = self.distance(obstacle_pos, pos)
        return K_REP * (1/dist) * (1/dist) * (1/dist)
        
        return force
                
    def attractive_force(self, pos, goal_pos):
        dist = self.distance(goal_pos, pos)
        if dist < GOAL_THRESHOLD:
            return [0,0]
            
        else:
            force = dist * K_ATT  
            return force  
        
    
    def drive(self, left_vel, right_vel):
    
        norm_left_vel = abs(MAX_SPEED*2) * ((1/(1 +(math.e**-left_vel)))-0.5)
        norm_right_vel = abs(MAX_SPEED*2) * ((1/(1 +(math.e**-right_vel)))-0.5)
            
        self.motor_l.setVelocity(norm_left_vel)
        self.motor_r.setVelocity(norm_right_vel)
        
        print(norm_left_vel)
        print(norm_right_vel)
        
    def stop(self):
        self.motor_l.setVelocity(0.0)
        self.motor_r.setVelocity(0.0)
    
    def getLidarData(self):
        return self.lidar.getRangeImage()
        
    def obstacle_avoidance(self):
        self.drive(10, 10)
        
         
    def run(self):
        while self.robot.step(self.timestep) != -1:
        
            att = self.attractive_force(self.getPose()[0], GOAL)
            rep = sum(self.repulsive_force(self.getPose()[0], obstacle_pos) for obstacle_pos in self.getLidarData())
            

    
            



robot = SwarmRobot()
robot.run()