import numpy as np
import heapq

# Manhattan distance heuristic
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Perform A* search on a 2D grid
def astar(grid, start, goal):
    # Allowed movements: up, right, down, left.
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    # Priority queue for the open set
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    
    came_from = {}  # For path reconstruction
    g_score = {start: 0}  # Cost from start to current node
    f_score = {start: heuristic(start, goal)}  # Estimated total cost
    
    closed_set = set()
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        closed_set.add(current)
        
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check grid bounds
            if (neighbor[0] < 0 or neighbor[0] >= grid.shape[0] or
                neighbor[1] < 0 or neighbor[1] >= grid.shape[1]):
                continue  # Out of bounds
            
            # Skip if an obstacle
            if grid[neighbor[0], neighbor[1]] == 1:
                continue
            
            tentative_g_score = g_score[current] + 1  # Cost is uniform
            
            if neighbor in closed_set and tentative_g_score >= g_score.get(neighbor, float('inf')):
                continue  # Not a better path
            
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return False  # No path found

# Reconstructs the path from start to goal
def reconstruct_path(came_from, current):
    
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path





import time
from picarx import Picarx
from vilib import Vilib

# Constants to calibrate movement (tweak these based on your testing)
TIME_PER_CELL = .75     # Time (in seconds) to move forward the distance of one grid cell
TURN_DURATION = 1.5     # Time (in seconds) to perform a turn maneuver
STEERING_ANGLE = 30     # Example steering angle in degrees (adjust as needed)

def path_to_commands(path):
    """
    Converts a list of grid coordinates into movement commands.
    Here, we assume:
      - Moving from one cell upward (row decreases) means a forward move.
      - Similarly, a move downward corresponds to a backward move.
      - Moves to the left or right imply a turn and then a forward move.
    """
    DIRECTIONS = ["up", "right", "down", "left"]  
    direction = "up"  # Track current facing direction
    commands = []
    grid_directions = []

    for i in range(1, len(path)):
        curr = path[i - 1]
        nxt = path[i]

        # Correct dx and dy based on row-major grid
        dx = nxt[0]- curr[0]   # Negative = moving down, Positive = moving up
        dy = nxt[1] - curr[1]  # Negative = moving left, Positive = moving right


        # Determine the required new direction
        if dx == 1 and dy == 0:
            new_direction = "up"
        elif dx == -1 and dy == 0:
            new_direction = "down"
        elif dx == 0 and dy == -1:
            new_direction = "left"
        elif dx == 0 and dy == 1:
            new_direction = "right"
        else:
            commands.append("unknown")
            continue
        grid_directions.append(new_direction)
        
        # Turn the robot if necessary
        while direction != new_direction:
            # Determine whether to turn left or right
            current_idx = DIRECTIONS.index(direction)
            target_idx = DIRECTIONS.index(new_direction)
            
            #calculate shortest turning direction
            diff = (target_idx - current_idx) % 4
            print(f"Turning from {direction} to {new_direction} (diff={diff})")

            if diff == 1:  
                commands.append("right")  
                direction = DIRECTIONS[(current_idx + 1) % 4]  # Turn right  
            elif diff == 3:  
                commands.append("left")  
                direction = DIRECTIONS[(current_idx - 1) % 4]  # Turn left  
            elif diff == 2:  
                commands.append("right")  
                commands.append("right")  
                direction = DIRECTIONS[(current_idx + 2) % 4]  # Turn around  
            else:  
                print("Unexpected direction change, check input values.")
                break  

        # Move forward after facing the correct direction
        commands.append("up")

    return commands, grid_directions

def detect_stop_sign():
    detected_sign = Vilib.traffic_sign_obj_parameter['t']  
    px.set_cam_pan_angle((-75))
    counter = 0
    #look left and right before continuing  
    while (detected_sign != 'stop' & counter < 20):
        sleep(0.1)
        detected_sign = Vilib.traffic_sign_obj_parameter['t']
        counter += 1
        px.set_cam_pan_angle((-75)+(7.5*counter))
    return detected_sign == 'stop'

def detect_obstacle_in_direction(direction, current):
    """

    Placeholder for obstacle detection logic.
    Replace this function with your actual sensor code.
    The function should return True if an obstacle is detected in the intended cell.
    
    For example, if using an ultrasonic sensor, check if the distance in the 
    direction of movement is below a threshold.
    """
    THRESHOLD_DISTANCE = 50  # cm; adjust based on your sensor and environment
    
    # For forward movement, keep the sensor (camera) centered.
    if direction == "up":
        px.set_cam_pan_angle(0)
        time.sleep(0.1)
        distance = px.ultrasonic.read()
    # For left, pan the sensor to the left.
    elif direction == "left":
        px.set_cam_pan_angle(-STEERING_ANGLE-30)
        time.sleep(0.3)
        distance = px.ultrasonic.read()
        px.set_cam_pan_angle(0)
    # For right, pan the sensor to the right.
    elif direction == "right":
        px.set_cam_pan_angle(STEERING_ANGLE+30)
        time.sleep(0.3)
        distance = px.ultrasonic.read()
        px.set_cam_pan_angle(0)
    # For backward movement, if no rear sensor is available, assume no obstacle.
    elif direction == "down":
        distance = float('inf')
    else:
        distance = float('inf')
    
    print(f"Obstacle detection for direction '{direction}': measured distance = {distance} cm")
    return (distance < THRESHOLD_DISTANCE and distance != -2)

def move_car(direction):

    if direction == "up":
        print("Moving forward")
        px.forward(30)
        time.sleep(TIME_PER_CELL)
        px.stop()
        
    elif direction == "down":
        print("Moving backward")
        px.backward(30)
        time.sleep(TIME_PER_CELL)
        px.stop()
        

    elif direction == "left":
        print("Turning left")

        px.set_dir_servo_angle(-STEERING_ANGLE)
        px.set_cam_pan_angle(-STEERING_ANGLE)
        px.forward(30)
        time.sleep(TURN_DURATION)

        
        time.sleep(TIME_PER_CELL)
        
        px.stop()
        px.set_dir_servo_angle(0)         # Reset steering to straight (typically 0 degrees)
        px.set_cam_pan_angle(0)
        
    elif direction == "right":
        print("Turning right")

        px.set_dir_servo_angle(STEERING_ANGLE)
        px.set_cam_pan_angle(STEERING_ANGLE)
        px.forward(30)
        time.sleep(TURN_DURATION)

        
        time.sleep(TIME_PER_CELL)

        px.stop()
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        
    else:
        print("Unknown command received!")





if __name__ == '__main__':
    px = Picarx()
    Vilib.camera_start()
    Vilib.traffic_detect_switch(True)
    Vilib.display(local=True, web=True)

    # Create a sample 10x10 grid (0 = free, 1 = obstacle)
    grid = np.zeros((10, 10), dtype=int)

    # Define start and goal positions in grid coordinates
    start = (0, 0)
    goal = (9,0)
    current_cell = start

    # Run A* search to compute a path
    while current_cell != goal:
        print(f"\nCurrent position: {current_cell}")
        path = astar(grid, current_cell, goal)
        if not path:
            print("No path found! Goal is unreachable.")
            break
        
        print("Path found:", path)
        commands, grid_directions = path_to_commands(path)
        print("Movement commands:", commands)
        
        # Execute the planned commands step-by-step
        for command, grid_direction in zip(commands, grid_directions):

            # Determine the next cell based on the current cell and the command
            if grid_direction == "up":
                next_cell = (current_cell[0] + 1, current_cell[1])
            elif grid_direction == "down":
                next_cell = (current_cell[0] - 1, current_cell[1])
            elif grid_direction == "left":
                next_cell = (current_cell[0], current_cell[1] - 1)
            elif grid_direction == "right":
                next_cell = (current_cell[0], current_cell[1] + 1)
            else:
                print("Encountered unknown command. Skipping.")
                continue

            # Check for stop sign before moving:
            if detect_stop_sign():
                print("Stop sign detected! Stopping the car.")
                px.stop()
                time.sleep(3)    
            
            # Before executing, check for an obstacle in the intended cell.
            if detect_obstacle_in_direction(command, current_cell):
                print(f"Obstacle detected at {next_cell}. Updating grid and replanning.")
                grid[next_cell] = 1  # Mark the cell as an obstacle.
                break  # Break out to replan the route from the current position.
            
            # No obstacle detected; execute the movement command.
            move_car(command)

            current_cell = next_cell  # Update the current position.
            
            # Check if the goal has been reached.
            if current_cell == goal:
                print("Goal reached!")
                break
        
        # Optional: Pause briefly before re-planning if needed.
        time.sleep(0.5)




"""Notes: 

- TIME PER CELL is how car measures where it is at (if TIME PER CELL is 0.5s, it will take the car 4.5s to go accross the grids x or y axis)

"""


