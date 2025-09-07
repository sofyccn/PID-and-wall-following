# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math
# Create the robot instance.
robot = MyRobot()
# Loads the environment from the maze file
maze_file = '../../worlds/Spring25/maze2.xml'
robot.load_environment(maze_file)
robot.move_to_start()
#PID constants
kp = 0.5
ki = 0.05
kd = 0.05
#variables to store error values
total_error = 0
last_error = 0
dt = 0.032
#target distance from the wall
goal = 1.0
#PID computations
def pid(goal, current_distance, kp, ki, kd, total_error, last_error, dt):
error = current_distance - goal
total_error = total_error + (error * dt)
error_rate_of_change = (error - last_error) / dt
P = kp * error
I = ki * total_error
D = kd * error_rate_of_change
pid_control = P + I + D
last_error = error
return pid_control, total_error, last_error
#Main control loop
while robot.experiment_supervisor.step(robot.timestep) != -1:
#Get lidar distance
lidar = robot.get_lidar_range_image()
front_distance = lidar[400]
#print the distance to the front wall
print(f"Current distance to the wall: {front_distance}")
#Calculate velocity using PID controller
pid_control, total_error, last_error = pid(goal, front_distance, kp, ki, kd,
total_error, last_error, dt)
#Print the motor velocity
print(f"pid:{pid_control}")
#apply the calculated velocity to the motor
robot.set_left_motors_velocity(pid_control)
robot.set_right_motors_velocity(pid_control)
#stop when the robot is close enough to the 1 meter goal
if abs(front_distance - goal)< 0.05:
robot.stop()
print("Robot stopped. Reached 1 meter from the wall")
break
