"""govno_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import time
# create the Robot instance.
robot = Robot()
camera = robot.getDevice('camera')
camera.enable(32)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# Main loop:
# - perform simulation steps until Webots is stopping the controller
i = 0
while robot.step(timestep) != -1:
    i+=1
    camera.saveImage(f'img{i}.jpg', 50)
    time.sleep(2)
