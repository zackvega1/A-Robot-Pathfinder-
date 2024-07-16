# A-Robot-Pathfinder-
This is a python pygame app that allows you to create custom mazes and have a simulated robot solve the maze in real time. 

# Robot Square Control

## Overview
This project demonstrates robot control in a simulated environment using Python and Pygame. The robot navigates a grid-based environment, executes movements based on user input, and can autonomously find paths using the A* algorithm.

## Features
- **Grid Environment**: Simulates a grid where the robot moves and can interact with obstacles.
- **User Control**: Allows users to control the robot's movements (forward, backward, turn left, turn right) using keyboard inputs.
- **Pathfinding**: Utilizes the A* algorithm to find the shortest path from the robot's current position to a user-defined goal position.
- **Serial Communication**: Provides an interface for serial communication to control a physical robot (optional, requires appropriate setup).

## Requirements
- Python 3.x
- Pygame library
- Optional: Serial port setup for physical robot control

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/robot-square-control.git
```

2. **Install Dependencies**
 ```bash
   pip install pygame
```

## Usage
1. Use the following keys to control the robot:
   - **W**: Move forward
   - **S**: Move backward
   - **A**: Turn left
   - **D**: Turn right

2. Optional: Set a goal position by right-clicking on the grid. Click the "Go" button to execute the A* algorithm and see the robot follow the path.

## Serial Communication Setup (Optional)
1. Connect your serial port device and configure the appropriate settings (baudrate, port).
2. Modify the `Robot` class to implement serial commands suitable for your device.


