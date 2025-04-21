import math
from controller import Robot

MAX_SPEED = 10.0
WHEEL_DISTANCE = 0.10

class SwarmRobot():
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motor_r = self.robot.getDevice('motor_r')
        self.motor_l = self.robot.getDevice('motor_l')
        self.lidar = self.robot.getDevice('lidar')
        
        self.motor_l.setPosition(float('inf'))
        self.motor_r.setPosition(float('inf'))
        
        self.lidar.enable(self.timestep)
        
    
    def drive(self, left_speed, right_speed):
        self.motor_l.setVelocity(left_speed)
        self.motor_r.setVelocity(right_speed)
        
    def stop(self):
        self.motor_l.setVelocity(0.0)
        self.motor_r.setVelocity(0.0)
    
    def getLidarData(self):
        return self.lidar.getRangeImage()
        
    def obstacle_avoidance(self, lidar_data, kp=1):
        min_distance = min(lidar_data)
        
        if math.isinf(min_distance):
            error = 0.0  # No obstacle detected
        else:
            error = 1.0 - min_distance  # Proportional error
        
        # Calculate proportional control
        proportional_gain = kp  # Adjust this value as needed
        control = proportional_gain * error
        
        # Apply control to the wheels
        left_speed = MAX_SPEED - control
        right_speed = MAX_SPEED + control
       
        # Make sure speeds are within limits
        left_speed = min(max(left_speed, -MAX_SPEED), MAX_SPEED)
        right_speed = min(max(right_speed, -MAX_SPEED), MAX_SPEED)
        
        self.drive(left_speed, right_speed)
        
         
    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.obstacle_avoidance(self.getLidarData(), 5)
            
            #print(self.getLidarData())
            
    
    
            



robot = SwarmRobot()
robot.run()