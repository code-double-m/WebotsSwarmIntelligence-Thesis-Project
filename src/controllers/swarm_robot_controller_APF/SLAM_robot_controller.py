import math
from controller import Robot

MAX_SPEED = 10.0
WHEEL_DISTANCE = 0.10

GRID_RESOLUTION = 0.1
GRID_SIZE_X = 70
GRID_SIZE_Y = 70

GOAL = [0,0]
 

class SwarmRobot():
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motor_r = self.robot.getDevice('motor_r')
        self.motor_l = self.robot.getDevice('motor_l')
        self.lidar = self.robot.getDevice('lidar')
        self.gps = self.robot.getDevice('gps')
        self.imu = self.robot.getDevice('imu')
        self.emitter = self.robot.getDevice('emitter')
        self.receiver = self.robot.getDevice('receiver')
        self.display = self.robot.getDevice('display')
        
        self.motor_l.setPosition(float('inf'))
        self.motor_r.setPosition(float('inf'))
        
        self.lidar.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.receiver.enable(self.timestep)
        
        self.map = [[0] * GRID_SIZE_Y for _ in range(GRID_SIZE_X)]
        
        self.robot_x = int(GRID_SIZE_X/2)
        self.robot_y = int(GRID_SIZE_Y/2)
    
    def new_map(self):
        self.map = [[0] * GRID_SIZE_Y for _ in range(GRID_SIZE_X)]
    
    def distance(self, pos_1, pos_2):
        return math.sqrt((pos_2[0] - pos_1[0])**2 + (pos_2[1] - pos_1[1])**2)
    
    def angle(self, pos_1, pos_2):
        return math.atan2(pos_2[1] - pos_1[1],pos_2[0] - pos_1[0])
        
    def getPose(self):
        pos = self.gps.getValues()
        rot = self.imu.getRollPitchYaw()
        
        return pos, rot    
    
    def drive(self, left_speed, right_speed):
        self.motor_l.setVelocity(left_speed)
        self.motor_r.setVelocity(right_speed)
        
    def broadcast(self, message):
        self.emitter.send(str(message).encode())
        
    def recieve(self):
        
        neighbours = []
    
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
                "point_of_interest": data_list[5], 
            }
            
            self.neighbours.append(neighbour)
        
    def stop(self):
        self.motor_l.setVelocity(0.0)
        self.motor_r.setVelocity(0.0)
    
    def getLidarData(self):
        return self.lidar.getRangeImage()
        
    def render_map(self, ldr_data):
        self.new_map()
            
        for x in range(GRID_SIZE_X):
            for y in range(GRID_SIZE_Y):
                    self.display.setColor(0x000000)
                    self.display.drawPixel(x, y)    
        
        
        for i, distance in enumerate(ldr_data):
            if distance > 0:
                angle = self.lidar.getFov() * (i/len(ldr_data)) - (self.lidar.getFov()/2)
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
               
                if math.isfinite(x) and math.isfinite(y):
                    grid_x = int((x + (GRID_SIZE_X * GRID_RESOLUTION)/2)/GRID_RESOLUTION)
                    grid_y = int((y + (GRID_SIZE_Y * GRID_RESOLUTION)/2)/GRID_RESOLUTION)
                    if 0 <= grid_x < GRID_SIZE_X and 0 <= grid_y < GRID_SIZE_Y:
                        self.map[grid_x][grid_y] = 1
                        
                        
        for x in range(GRID_SIZE_X):
           for y in range(GRID_SIZE_Y):
               if self.map[x][y] == 1:
                   self.display.setColor(0x7700FF)
                   self.display.drawPixel(x, y)
                   
        #robot's relative position
        self.display.setColor(0xFF0000)
        self.display.drawPixel(self.robot_x, self.robot_y)
    
        
         
    def run(self):
        while self.robot.step(self.timestep) != -1:
            ldr_data = self.getLidarData()
            self.drive(MAX_SPEED, MAX_SPEED)
            
            self.render_map(ldr_data)
                        
            
            if self.map[self.robot_x + 1][self.robot_y] == 1:
                self.drive(-MAX_SPEED, MAX_SPEED)
                
            elif self.map[self.robot_x][self.robot_y + 1] == 1:
                self.drive(-MAX_SPEED, MAX_SPEED)
            
            elif self.map[self.robot_x][self.robot_y - 1] == 1:
                self.drive(MAX_SPEED, -MAX_SPEED)
                        
            
    
            



robot = SwarmRobot()
robot.run()