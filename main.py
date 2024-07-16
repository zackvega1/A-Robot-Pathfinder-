import pygame
import sys
import math
from queue import PriorityQueue
import serial
import time

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 650
GRID_SIZE = 30  # 1 foot in pixels
NUM_ROWS, NUM_COLS = 25, 25  # Grid dimensions
SQUARE_SIZE = GRID_SIZE  # Robot is 1 foot
SQUARE_COLOR = (255, 0, 0)  # Red
DOT_COLOR = (0, 0, 255)  # Blue
GOAL_COLOR = (0, 255, 255)  # Cyan for goal square
BACKGROUND_COLOR = (0, 0, 0)  # Black
GRID_COLOR = (200, 200, 200)  # Light grey
WHITE_COLOR = (255, 255, 255)  # White
PATH_COLOR = (255, 255, 0)  # Yellow for path
MOVE_SPEED = 2  # Adjust movement speed as needed
TURN_SPEED = 5  # Degrees per frame for rotation
DOT_DISTANCE = GRID_SIZE  # Distance of the blue dot from the square's front

# Calculate width of the white area (1/3 of screen width)
WHITE_AREA_WIDTH = SCREEN_WIDTH // 3

# Calculate actual screen dimensions to fit the grid and white area
GRID_SCREEN_WIDTH = NUM_COLS * GRID_SIZE
GRID_SCREEN_HEIGHT = NUM_ROWS * GRID_SIZE

ser = None
try:
    ser = serial.Serial(
        port='COM8',      
        baudrate=115200,    
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    time.sleep(2)
    print("Serial port opened successfully.")
except serial.SerialException as e:
    print(f"Failed to open serial port: {e}")
    ser = None

# Set up the display
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robot Square Control")

# Initial position and orientation of the square (robot)
square_pos = [SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2]
square_angle = 0  # In degrees

# Create a map of the grid
grid_map = [[0 for _ in range(NUM_COLS)] for _ in range(NUM_ROWS)]

# Goal position
goal_pos = None

# Path variable to store the path from A* algorithm
path = None

# Movement states
TURNING_LEFT = "TURNING_LEFT"
TURNING_RIGHT = "TURNING_RIGHT"
MOVING_FORWARD = "MOVING_FORWARD"
MOVING_BACKWARD = "MOVING_BACKWARD"
IDLE = "IDLE"

# Initial state
current_state = IDLE

def send_command(command):
    if ser:
        try:
            ser.write(command.encode())
            print(f"Command '{command}' sent.")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
    else:
        print(f"Simulating command '{command}' as serial is not connected.")

def draw_grid(screen, grid_color, grid_size, num_rows, num_cols):
    for x in range(0, num_cols * grid_size + 1, grid_size):
        pygame.draw.line(screen, grid_color, (x, 0), (x, num_rows * grid_size))
    for y in range(0, num_rows * grid_size + 1, grid_size):
        pygame.draw.line(screen, grid_color, (0, y), (num_cols * grid_size, y))


def draw_squares(screen, grid_map, grid_size):
    for y, row in enumerate(grid_map):
        for x, cell in enumerate(row):
            if cell == 1:
                pygame.draw.rect(screen, WHITE_COLOR, (x * grid_size, y * grid_size, grid_size, grid_size))

def draw_square(screen, color, position, angle):
    center_x = position[0]
    center_y = position[1]
    half_size = SQUARE_SIZE // 2
    
    points = [
        (center_x - half_size, center_y - half_size),  
        (center_x + half_size, center_y - half_size),  
        (center_x + half_size, center_y + half_size),  
        (center_x - half_size, center_y + half_size)  
    ]
    
    angle_rad = math.radians(angle)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    
    rotated_points = []
    for point in points:
        x, y = point
        x_rot = (x - center_x) * cos_angle - (y - center_y) * sin_angle + center_x
        y_rot = (x - center_x) * sin_angle + (y - center_y) * cos_angle + center_y
        rotated_points.append((x_rot, y_rot))

    pygame.draw.polygon(screen, color, rotated_points)
    
    # Draw the blue dot at the front of the square
    front_x = center_x + (half_size + DOT_DISTANCE) * cos_angle
    front_y = center_y + (half_size + DOT_DISTANCE) * sin_angle
    pygame.draw.circle(screen, DOT_COLOR, (int(front_x), int(front_y)), 5)

# Function to check collision with white squares
def check_collision(position, grid_map, grid_size):
    pos_x, pos_y = position
    grid_x = int(pos_x // grid_size)
    grid_y = int(pos_y // grid_size)
    
    if grid_x < 0 or grid_x >= len(grid_map[0]) or grid_y < 0 or grid_y >= len(grid_map):
        return True  # Out of bounds
    
    return grid_map[grid_y][grid_x] == 1

# A* algorithm to find the shortest path
def astar_algorithm(start, goal, grid_map):
    # Directions for moving: right, left, down, up
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    num_rows = len(grid_map)
    num_cols = len(grid_map[0])
    
    # Priority queue for open nodes
    open_queue = PriorityQueue()
    open_queue.put((0, start))  # Start node with priority 0
    
    # Closed set to keep track of visited nodes
    closed_set = set()
    closed_set.add(start)
    
    # Dictionary to store parent nodes for constructing the path
    parent_map = {}
    parent_map[start] = None
    
    # Cost from start to each node
    g_cost = {start: 0}
    
    while not open_queue.empty():
        current_cost, current_node = open_queue.get()
        
        # If goal is reached, construct and return the path
        if current_node == goal:
            path = []
            while current_node in parent_map:
                path.append(current_node)
                current_node = parent_map[current_node]
            return path[::-1]  # Reverse path to start from start node
        
        # Explore neighbors
        for direction in directions:
            neighbor = (current_node[0] + direction[0], current_node[1] + direction[1])
            
            # Check if neighbor is within bounds and not a wall
            if (0 <= neighbor[0] < num_rows and
                0 <= neighbor[1] < num_cols and
                grid_map[int(neighbor[0])][int(neighbor[1])] != 1):  # Ensure indices are integers
                # Calculate tentative g score
                tentative_g = g_cost[current_node] + 1  # Assuming uniform cost of movement
                
                # If neighbor not in closed set or has lower g score, add to open set
                if neighbor not in closed_set or tentative_g < g_cost.get(neighbor, float('inf')):
                    g_cost[neighbor] = tentative_g
                    f_cost = tentative_g + heuristic(neighbor, goal)
                    open_queue.put((f_cost, neighbor))
                    closed_set.add(neighbor)
                    parent_map[neighbor] = current_node
    
    return None  # No path found

# Heuristic function (Manhattan distance for grid)
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Function to draw the path on the grid
def draw_path(screen, path, grid_size):
    for node in path:
        pygame.draw.rect(screen, PATH_COLOR, (node[1] * grid_size, node[0] * grid_size, grid_size, grid_size))

# Function to update the state of the robot
def update_state(keys, current_state):
    if keys[pygame.K_w]:
        return MOVING_FORWARD
    elif keys[pygame.K_s]:
        return MOVING_BACKWARD
    elif keys[pygame.K_a]:
        return TURNING_LEFT
    elif keys[pygame.K_d]:
        return TURNING_RIGHT
    else:
        return IDLE

# Function to move the robot towards the next position in the path
def move_robot_towards(next_pos, square_pos, square_angle):
    dx = next_pos[0] - square_pos[0]
    dy = next_pos[1] - square_pos[1]
    target_angle = math.degrees(math.atan2(dy, dx))
    
    # Normalize angles to be within -180 to 180 degrees
    delta_angle = (target_angle - square_angle + 180) % 360 - 180
    
    if abs(delta_angle) > TURN_SPEED:
        if delta_angle > 0:
            return TURNING_RIGHT, square_angle + TURN_SPEED
        else:
            return TURNING_LEFT, square_angle - TURN_SPEED
    else:
        return MOVING_FORWARD, target_angle

# Main game loop
running = True
moving = False
move_index = 0
while running:
    
        
 
    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:

            # Convert mouse position to grid coordinates

            mouse_x, mouse_y = event.pos

            if 0 <= mouse_x < GRID_SCREEN_WIDTH and 0 <= mouse_y < GRID_SCREEN_HEIGHT:

                grid_x = mouse_x // GRID_SIZE

                grid_y = mouse_y // GRID_SIZE

                # Right click sets goal square

                if event.button == 3:  # Right mouse button

                    goal_pos = (grid_y, grid_x)

                # Toggle the state of the clicked square on left click

                elif event.button == 1:  # Left mouse button

                    grid_map[grid_y][grid_x] = 1 if grid_map[grid_y][grid_x] == 0 else 0
    
    keys = pygame.key.get_pressed()
    current_state = update_state(keys, current_state)
    
    if current_state == MOVING_FORWARD:
        next_pos_x = square_pos[0] + MOVE_SPEED * math.cos(math.radians(square_angle))
        next_pos_y = square_pos[1] + MOVE_SPEED * math.sin(math.radians(square_angle))
        if not check_collision((next_pos_x, next_pos_y), grid_map, GRID_SIZE):
            square_pos[0] = next_pos_x
            square_pos[1] = next_pos_y
            # Serial Foward
            command = "R2"  
            send_command(command)
            print(f"Command '{command}' sent.")
            command = "L2"  
            send_command(command)
            print(f"Command '{command}' sent.")
    elif current_state == MOVING_BACKWARD:
        next_pos_x = square_pos[0] - MOVE_SPEED * math.cos(math.radians(square_angle))
        next_pos_y = square_pos[1] - MOVE_SPEED * math.sin(math.radians(square_angle))
        if not check_collision((next_pos_x, next_pos_y), grid_map, GRID_SIZE):
            square_pos[0] = next_pos_x
            square_pos[1] = next_pos_y
            # Serial Foward
            command = "R-2"  
            send_command(command)
            print(f"Command '{command}' sent.")
            command = "L-2"  
            send_command(command)
            print(f"Command '{command}' sent.")
    elif current_state == TURNING_LEFT:
        square_angle -= TURN_SPEED
        # Serial Turn Left 
        command = "R2"  
        send_command(command)
        print(f"Command '{command}' sent.")

    elif current_state == TURNING_RIGHT:
        square_angle += TURN_SPEED

        # Serial Turn Right 
        command = "L2"  
        send_command(command)
        print(f"Command '{command}' sent.")


    # Clear the screen
    screen.fill(BACKGROUND_COLOR)

    # Draw the grid
    draw_grid(screen, GRID_COLOR, GRID_SIZE, NUM_ROWS, NUM_COLS)
    
    # Draw the squares based on their state
    draw_squares(screen, grid_map, GRID_SIZE)

    # Draw the white area for future buttons or interface elements
    # pygame.draw.rect(screen, WHITE_COLOR, (0, (0 + 2 * SCREEN_HEIGHT // 3), SCREEN_WIDTH, SCREEN_HEIGHT))

    # Draw the goal square if set
    if goal_pos:
        pygame.draw.rect(screen, GOAL_COLOR, (goal_pos[1] * GRID_SIZE, goal_pos[0] * GRID_SIZE, GRID_SIZE, GRID_SIZE))

    

    # Button to execute A* algorithm and draw path
    go_button_rect = pygame.Rect(WHITE_AREA_WIDTH + 50, 150, 100, 50)
    pygame.draw.rect(screen, WHITE_COLOR, go_button_rect)
    go_font = pygame.font.Font(None, 36)
    go_text = go_font.render("Go", True, (0, 0, 0))
    go_text_rect = go_text.get_rect(center=go_button_rect.center)
    screen.blit(go_text, go_text_rect)

    # Check if Go button is clicked
    mouse_x, mouse_y = pygame.mouse.get_pos()
    if go_button_rect.collidepoint(mouse_x, mouse_y):
        if pygame.mouse.get_pressed()[0]:  # Left mouse button clicked
            if goal_pos:
                start_pos = (square_pos[1] // GRID_SIZE, square_pos[0] // GRID_SIZE)
                path = astar_algorithm(start_pos, goal_pos, grid_map)
                if path:
                    moving = True
                    move_index = 0

    # Move along the path if moving is True
    if moving and move_index < len(path):
        # Get the next position in the path
        next_node = path[move_index]
        next_pos = (next_node[1] * GRID_SIZE + GRID_SIZE // 2,
                    next_node[0] * GRID_SIZE + GRID_SIZE // 2)
        
        # Move the robot towards the next position
        current_state, square_angle = move_robot_towards(next_pos, square_pos, square_angle)
        
        # If moving forward, update the robot's position
        if current_state == MOVING_FORWARD:
            delta_x = math.cos(math.radians(square_angle)) * MOVE_SPEED
            delta_y = math.sin(math.radians(square_angle)) * MOVE_SPEED
            square_pos[0] += delta_x
            square_pos[1] += delta_y
        
            # If close enough to the next position, move to the next one
            if abs(square_pos[0] - next_pos[0]) < MOVE_SPEED and abs(square_pos[1] - next_pos[1]) < MOVE_SPEED:
                move_index += 1

    # Draw the path if it exists
    if path:
        draw_path(screen, path, GRID_SIZE)
    # Draw the square (robot)
    draw_square(screen, SQUARE_COLOR, square_pos, square_angle)
    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    pygame.time.Clock().tick(60)

pygame.quit()
sys.exit()


