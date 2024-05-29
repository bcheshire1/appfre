# File: Astar_Radiation_PathPlanning.py
# Author: Ben Cheshire
# Date: 28/05/2024
# Source: OpenAI ChatGPT
# 
# Description:
# This script is used to implement the A* path planning algorithm with radiation costmap

import numpy as np
import cv2
import pygame
import math
import os

# Define colours
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
CYAN = (0, 255, 255)

# Define scale factors
RADIATION_WEIGHTING = 2.0
GEOMETRIC_WEIGHTING = 1.0
HEURISTIC_WEIGHTING = 1.0

CELL_SIZE = 2

class Cell:
    def __init__(self, x, y):
        self.width = CELL_SIZE
        self.x = x
        self.y = y
        self.reachable = True
        self.g = float('inf')  # Cost from start node
        self.h = float('inf')  # Heuristic (estimated cost to goal)
        self.r = float('inf')  # Radiation cost
        self.f = float('inf')  # Total cost (g + h + r)
        self.parent = None

    def draw(self, win):
        pygame.draw.rect(win, self.colour, (self.x, self.y, self.width, self.width))
        
class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.open_set = []
        self.closed_set = []

    def heuristic(self, cell, goal):
        # Euclidean distance as the heuristic
        h = ((cell.x - goal.x) ** 2 + (cell.y - goal.y) ** 2) ** 0.5
        return h

    def get_neighbours(self, cell):
        neighbours = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:  # Skip the current cell itself
                    continue
                nx = cell.x + dx  
                ny = cell.y + dy

                # Boundary Checks:
                nx = max(0, min(nx, len(self.grid[0]) - 1)) 
                ny = max(0, min(ny, len(self.grid) - 1))        
                
                # print(f"dx: {dx}, dy: {dy}, cell.x: {cell.x}, cell.y: {cell.y} --> Calculated nx: {nx}, ny: {ny}") 

                try:
                    if self.grid[nx][ny].reachable:
                        neighbours.append(self.grid[nx][ny])
                except IndexError: 
                    pass # Ignore out of bounds indices
        return neighbours



    def find_path(self, start, goal, use_radiation=True):
        # print(f"Received start: ({start.x}, {start.y}), end: ({goal.x}, {goal.y})")
        # print(f"Finding path from ({start.x}, {start.y}) to ({goal.x}, {goal.y})...")
        start.g = 0
        start.h = self.heuristic(start, goal)
        start.f = start.g + start.h + start.r
        self.open_set.append(start)

        while self.open_set:
            current = min(self.open_set, key=lambda cell: cell.f)
            if current == goal:
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1]

            self.open_set.remove(current)
            self.closed_set.append(current)

            for neighbour in self.get_neighbours(current):
                if neighbour in self.closed_set:
                    continue

                if neighbour.x == current.x or neighbour.y == current.y:
                    tentative_g = current.g + 1  # Assuming uniform cost
                else:
                    tentative_g = current.g + 1.4
                if tentative_g < neighbour.g:
                    neighbour.g = tentative_g
                    neighbour.h = self.heuristic(neighbour, goal)
                    if use_radiation:
                        neighbour.f = (neighbour.g * GEOMETRIC_WEIGHTING) + (neighbour.h * HEURISTIC_WEIGHTING) + (neighbour.r * RADIATION_WEIGHTING)
                    else:
                        neighbour.f = (neighbour.g * GEOMETRIC_WEIGHTING) + (neighbour.h * HEURISTIC_WEIGHTING)
                    neighbour.parent = current

                    if neighbour not in self.open_set:
                        self.open_set.append(neighbour)
        print("No path found")
        return None

def read_pgm(filename):
    # Read PGM file and return pixel values as a NumPy array
    with open(filename, 'rb') as f:
        header = f.readline().decode().strip()
        if header != 'P5':
            raise ValueError("Invalid PGM format")
        width, height = map(int, f.readline().decode().split())
        max_val = int(f.readline().decode())
        data = np.fromfile(f, dtype=np.uint8, count=width * height)
        return data.reshape((height, width))

def resize_to_match_size(smaller, larger):
    # Resize the smaller image to match the dimensions of the larger image
    resized_smaller =  cv2.resize(smaller, (larger.shape[1], larger.shape[0]), interpolation=cv2.INTER_NEAREST)
    return resized_smaller

def create_grid(occupancy_grid, radiation_costmap, rad_sf = 1.0):
    if rad_sf == 'MAX' or rad_sf * np.max(radiation_costmap) > 255:
        max_rad = np.max(radiation_costmap)
        rad_sf = 255 / max_rad
    
    grid = [[Cell(x, y) for y in range(occupancy_grid.shape[1])] for x in range(occupancy_grid.shape[0])]
    for row in range(occupancy_grid.shape[0]):
        for col in range(occupancy_grid.shape[1]):
            if occupancy_grid[row, col] < 128:
                grid[row][col].reachable = False
            grid[row][col].r = radiation_costmap[row, col] * rad_sf
    return grid

def main():
    pygame.init()
    path = None

    # Get the directory of the current script
    current_dir = os.path.dirname(__file__)

    # Go up one directory level from the current script's directory
    parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))

    # Specify the path to the file in the other directory
    rad_file_path = os.path.join(parent_dir, 'rad_data', 'Obs_rad_Data_5s.pgm')
    obs_file_path = os.path.join(parent_dir, 'maps', 'output.pgm')

    # Read occupancy grid and radiation costmap
    obs_costmap = read_pgm(obs_file_path)
    radiation_costmap = read_pgm(rad_file_path)

    # Ensure both costmaps have the same dimensions
    if obs_costmap.shape != radiation_costmap.shape:
        if np.prod(obs_costmap.shape) < np.prod(radiation_costmap.shape):
            obs_costmap = resize_to_match_size(obs_costmap, radiation_costmap)
        else:
            radiation_costmap = resize_to_match_size(radiation_costmap, obs_costmap)

    ROWS, COLS = obs_costmap.shape

    grid_width = COLS
    grid_height = ROWS
    cell_size = CELL_SIZE

    window_width = grid_width * cell_size
    window_height = grid_height * cell_size

    win = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("A* Path Finding Algorithm")

    costmap = create_grid(obs_costmap, radiation_costmap, 'MAX')

    colour_gradient_1 = np.linspace(BLUE, CYAN, math.ceil(256/3))
    colour_gradient_2 = np.linspace(CYAN, GREEN, math.ceil(256/3))
    colour_gradient_3 = np.linspace(GREEN, RED, 256 - len(colour_gradient_1) - len(colour_gradient_2))
    colour_gradient = np.concatenate((colour_gradient_1, colour_gradient_2, colour_gradient_3))


    # Main loop
    running = True
    started = False
    start_point = None
    end_point = None
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN and not started:
                mouse_pos = pygame.mouse.get_pos()

                grid_pos = (mouse_pos[1] // cell_size, mouse_pos[0] // cell_size)  # Convert screen coordinates to grid coordinates

                # Check if the mouse click is within the grid boundaries
                if 0 <= grid_pos[0] < grid_height and 0 <= grid_pos[1] < grid_width:
                    if start_point is None:
                        start_point = grid_pos
                    elif end_point is None:
                        end_point = grid_pos

        # Draw grid
        win.fill(WHITE)
        for row in range(ROWS):
            for col in range(COLS):
                if not costmap[row][col].reachable:
                    colour = BLACK
                else:
                    colour = colour_gradient[int(costmap[row][col].r)]
                pygame.draw.rect(win, colour, (col * cell_size, row * cell_size, cell_size, cell_size))

        # Draw start point and end point if defined
        if start_point:
            pygame.draw.rect(win, WHITE, (start_point[1] * cell_size, start_point[0] * cell_size, cell_size, cell_size))
        if end_point:
            pygame.draw.rect(win, WHITE, (end_point[1] * cell_size, end_point[0] * cell_size, cell_size, cell_size))
        
        if path:
            for cell in path:
                pygame.draw.rect(win, BLUE, (cell[1] * cell_size, cell[0] * cell_size, cell_size, cell_size))

        if start_point and end_point and not started:
            started = True
            print(f"\nEntering A* algorithm with start point {start_point} and end point {end_point}...")
            path = AStar(costmap).find_path(costmap[start_point[0]][start_point[1]], costmap[end_point[0]][end_point[1]], use_radiation=False)

        pygame.display.update()
        # start_point = (261, 147)
        # end_point = (260, 123)


if __name__ == '__main__':
    main()