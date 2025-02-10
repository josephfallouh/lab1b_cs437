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

def path_to_commands(path):
    """
    Converts a list of grid coordinates into movement commands.
    Here, we assume:
      - Moving from one cell upward (row decreases) means a forward move.
      - Similarly, a move downward corresponds to a backward move.
      - Moves to the left or right imply a turn and then a forward move.
    Adjust these assumptions based on your grid's orientation relative to your car.
    """
    commands = []
    for i in range(1, len(path)):
        curr = path[i - 1]
        nxt = path[i]
        dx = nxt[0] - curr[0]
        dy = nxt[1] - curr[1]
        if dx == -1 and dy == 0:
            commands.append("up")
        elif dx == 1 and dy == 0:
            commands.append("down")
        elif dx == 0 and dy == -1:
            commands.append("left")
        elif dx == 0 and dy == 1:
            commands.append("right")
        else:
            commands.append("unknown")
    return commands





import time
from picarx import Picarx 

px = Picarx()

# Constants to calibrate movement (tweak these based on your testing)
TIME_PER_CELL = 0.5     # Time (in seconds) to move forward the distance of one grid cell
TURN_DURATION = 0.3     # Time (in seconds) to perform a turn maneuver
STEERING_ANGLE = 30     # Example steering angle in degrees (adjust as needed)

def move_car(direction):

    if direction == "up":
        print("Moving forward")
        px.forward()
        time.sleep(TIME_PER_CELL)
        px.stop()
        
    elif direction == "down":
        print("Moving backward")
        px.backward()
        time.sleep(TIME_PER_CELL)
        px.stop()
        
    elif direction == "left":
        print("Turning left")

        px.set_dir_angle(STEERING_ANGLE)
        time.sleep(TURN_DURATION)

        px.forward()
        time.sleep(TIME_PER_CELL)
        
        px.stop()
        px.set_dir_angle(0)         # Reset steering to straight (typically 0 degrees)
        
    elif direction == "right":
        print("Turning right")

        px.set_dir_angle(-STEERING_ANGLE)
        time.sleep(TURN_DURATION)

        px.forward()
        time.sleep(TIME_PER_CELL)

        px.stop()
        px.set_dir_angle(0)
        
    else:
        print("Unknown command received!")





if __name__ == '__main__':
    # Create a sample 10x10 grid (0 = free, 1 = obstacle)
    grid = np.zeros((10, 10), dtype=int)
    # Example obstacles:
    grid[3, 1:8] = 1    # Horizontal wall at row 3
    grid[6, 2:10] = 1   # Horizontal wall at row 6

    # Define start and goal positions in grid coordinates
    start = (0, 0)
    goal = (9, 9)
    
    # Run A* search to compute a path
    path = astar(grid, start, goal)
    
    if not path:
        print("No path found!")
    else:
        print("Path found:", path)
        commands = path_to_commands(path)
        print("Movement commands:", commands)
        
        # Execute each command on the PiCar-X
        for cmd in commands:
            move_car(cmd)
            # Optional: Add a short pause between commands
            time.sleep(0.5)



