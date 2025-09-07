# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math
# Create the robot instance
robot = MyRobot()
# Loads the environment from the maze file
maze_file = '../../worlds/Spring25/maze3.xml'
robot.load_environment(maze_file)
robot.move_to_start()
# PID constants
kp = 0.5
ki = 0.01
kd = 0.01
# Variables to store error values
total_error = 0
last_error = 0
dt = 0.032
# Target distance from the wall
goal = 0.5
max_speed = 15
wall_following_mode = "right"
# PID computation
def pid(goal, current_distance, kp, ki, kd, total_error, last_error, dt):
error = current_distance - goal
total_error += error * dt
error_rate_of_change = (error - last_error) / dt
# PID control signal
P = kp * error
I = ki * total_error
D = kd * error_rate_of_change
pid_control = P + I + D
return pid_control, total_error, error # Return error for last_error update
# Check for an obstacle in front
def is_obstacle_in_front():
lidar = robot.get_lidar_range_image()
front_distance = lidar[400] # Use the 400th lidar reading for the front
return front_distance < 0.5 # Detect obstacles within 0.5 meters
# Get the distance from the wall on the side
def get_side_distance():
lidar = robot.get_lidar_range_image()
if wall_following_mode == "left":
side_distance = lidar[200] # Left LIDAR sensor reading
else:
side_distance = lidar[600] # Right LIDAR sensor reading
return side_distance
# Rotate the robot by 90 degrees (maintain wall-following mode)
def rotate_90_degrees():
if wall_following_mode == "left":
# Rotate right (clockwise) 90 degrees to continue following the left wall
robot.set_left_motors_velocity(9)
robot.set_right_motors_velocity(0)
else:
# Rotate left (counterclockwise) 90 degrees to continue following the right
wall
robot.set_left_motors_velocity(0)
robot.set_right_motors_velocity(9)
start_time = robot.experiment_supervisor.getTime()
while robot.experiment_supervisor.step(robot.timestep) != -1:
current_time = robot.experiment_supervisor.getTime()
if current_time - start_time >= 1: #time for 90-degree rotation
robot.stop()
break
def rotate_edge():
if wall_following_mode == "left":
# Rotate right (clockwise) 90 degrees to continue following the left wall
robot.set_left_motors_velocity(0)
robot.set_right_motors_velocity(9)
else:
# Rotate left (counterclockwise) 90 degrees to continue following the right
wall
robot.set_left_motors_velocity(9)
robot.set_right_motors_velocity(0)
start_time = robot.experiment_supervisor.getTime()
while robot.experiment_supervisor.step(robot.timestep) != -1:
current_time = robot.experiment_supervisor.getTime()
if current_time - start_time >= 1: #time for 90-degree rotation
robot.stop()
break
def move_forward(duration):
robot.set_left_motors_velocity(max_speed)
robot.set_right_motors_velocity(max_speed)
start_time = robot.experiment_supervisor.getTime()
while robot.experiment_supervisor.step(robot.timestep) != -1:
current_time = robot.experiment_supervisor.getTime()
if current_time - start_time >= duration:
robot.stop()
break
# Main control loop
while robot.experiment_supervisor.step(robot.timestep) != -1:
if robot.experiment_supervisor.getTime()<0.1:
move_forward(1.5)
side_distance = get_side_distance()
# Check for sudden changes in side distance
if side_distance > 1:
print("Edge detected")
rotate_edge()
move_forward(1)
continue
# Calculate velocity using PID controller
pid_control, total_error, error = pid(goal, side_distance, kp, ki, kd,
total_error, last_error, dt)
last_error = error # Update last_error
# Set motor velocities based on wall-following mode
if wall_following_mode == "left":
robot.set_left_motors_velocity(max_speed - pid_control)
robot.set_right_motors_velocity(max_speed)
else:
robot.set_left_motors_velocity(max_speed)
robot.set_right_motors_velocity(max_speed - pid_control)
# Handle obstacles in front
if is_obstacle_in_front():
print("Obstacle in front")
robot.stop()
rotate_90_degrees()
