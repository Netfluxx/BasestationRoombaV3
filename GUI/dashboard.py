import tkinter as tk
import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import cv2
import numpy as np
from PIL import Image, ImageTk
import random
import time
from scipy.interpolate import splprep, splev
from path_planning import search

# Global Variables
MAP_SIZE_X = 50  # 50 meters
MAP_SIZE_Y = 20  # 20 meters
GRID_SIZE = 0.1  # 0.1 meter precision
OBSTACLE_SIZE = 2  # Size of the obstacle in grid units (20 cm)

# Initialize the main window
root = ctk.CTk()
root.title("Rover Dashboard")
root.geometry("1000x800")

# Create frames for different sections
speed_frame = ctk.CTkFrame(root, corner_radius=10)
speed_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

linear_speed_frame = ctk.CTkFrame(root, corner_radius=10)
linear_speed_frame.grid(row=0, column=1, padx=10, pady=10, sticky="ew")

map_frame = ctk.CTkFrame(root, corner_radius=10)
map_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

battery_frame = ctk.CTkFrame(root, corner_radius=10)
battery_frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")

camera_frame = ctk.CTkFrame(root, corner_radius=10)
camera_frame.grid(row=2, column=1, padx=10, pady=10, sticky="ew")

control_frame = ctk.CTkFrame(root, corner_radius=10)
control_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=10, sticky="ew")

# Ensure the map frame expands with window resizing
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1)
root.rowconfigure(1, weight=1)

# Grid and Matrix Initialization
matrix_size = (int(MAP_SIZE_X // GRID_SIZE), int(MAP_SIZE_Y // GRID_SIZE))
obstacle_matrix = [[0 for _ in range(matrix_size[1])] for _ in range(matrix_size[0])]

# Wheel Speeds
wheel_speeds = {}
for i in range(4):
    label = ctk.CTkLabel(speed_frame, text=f"Wheel {i+1}:", width=100)
    label.grid(row=i, column=0, sticky="w")
    speed_var = tk.StringVar(value="0 RPM")
    speed_display = ctk.CTkLabel(speed_frame, textvariable=speed_var, width=100)
    speed_display.grid(row=i, column=1, sticky="e")
    wheel_speeds[f"wheel_{i+1}"] = speed_var

# Linear Speed of the rover
linear_speed_var = tk.StringVar(value="0 m/s")
linear_speed_label = ctk.CTkLabel(linear_speed_frame, textvariable=linear_speed_var)
linear_speed_label.pack()

# Battery Level
battery_var = tk.StringVar(value="100%")
battery_label = ctk.CTkLabel(battery_frame, textvariable=battery_var)
battery_label.pack()

# 2D Map
fig, ax = plt.subplots()
ax.set_title("Rover Position")
ax.set_xlim(0, MAP_SIZE_X)  # Adjust as needed
ax.set_ylim(0, MAP_SIZE_Y)  # Adjust as needed

# Draw the grid
for x in np.arange(0, MAP_SIZE_X + GRID_SIZE, GRID_SIZE):
    ax.plot([x, x], [0, MAP_SIZE_Y], color='gray', linewidth=0.5)
for y in np.arange(0, MAP_SIZE_Y + GRID_SIZE, GRID_SIZE):
    ax.plot([0, MAP_SIZE_X], [y, y], color='gray', linewidth=0.5)

map_canvas = FigureCanvasTkAgg(fig, master=map_frame)
map_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)
rover_position, = ax.plot([], [], 'bo')  # Rover's position marker
path_plot, = ax.plot([], [], 'g-', label='Path')  # Placeholder for path
smoothed_path_plot, = ax.plot([], [], 'b-', label='Smoothed Path')  # Placeholder for smoothed path
obstacles = []
obstacle_patches = []
adding_obstacle = False
mouse_pressed = False  # Track mouse press state

def update_obstacle_matrix():
    global obstacle_matrix
    obstacle_matrix = [[0 for _ in range(matrix_size[1])] for _ in range(matrix_size[0])]
    for circle in obstacles:
        ox, oy = circle.center
        # Convert and snap to grid coordinates
        grid_x = int(ox // GRID_SIZE)
        grid_y = int(oy // GRID_SIZE)
        for i in range(OBSTACLE_SIZE):  # OBSTACLE_SIZE x OBSTACLE_SIZE cells for each obstacle
            for j in range(OBSTACLE_SIZE):
                if grid_x + i < matrix_size[0] and grid_y + j < matrix_size[1]:
                    obstacle_matrix[grid_x + i][grid_y + j] = 1  # Obstacle cost

def on_map_click(event):
    global adding_obstacle, mouse_pressed
    if adding_obstacle:
        mouse_pressed = True
        # Snap to grid
        obs_x = int(event.xdata // GRID_SIZE) * GRID_SIZE + GRID_SIZE / 2
        obs_y = int(event.ydata // GRID_SIZE) * GRID_SIZE + GRID_SIZE / 2
        if obs_x is not None and obs_y is not None:
            circle = plt.Circle((obs_x, obs_y), GRID_SIZE / 2, edgecolor='orange', facecolor='orange')
            obstacles.append(circle)
            ax.add_patch(circle)
            obstacle_patches.append(circle)
            map_canvas.draw()
            update_obstacle_matrix()

def on_map_drag(event):
    global mouse_pressed
    if adding_obstacle and mouse_pressed and event.xdata is not None and event.ydata is not None:
        obs_x = int(event.xdata // GRID_SIZE) * GRID_SIZE + GRID_SIZE / 2
        obs_y = int(event.ydata // GRID_SIZE) * GRID_SIZE + GRID_SIZE / 2
        circle = plt.Circle((obs_x, obs_y), GRID_SIZE / 2, edgecolor='orange', facecolor='orange')
        obstacles.append(circle)
        ax.add_patch(circle)
        obstacle_patches.append(circle)
        update_obstacle_matrix()

def on_map_release(event):
    global mouse_pressed
    mouse_pressed = False

def add_obstacle():
    global adding_obstacle
    adding_obstacle = True

def clear_obstacles():
    global obstacles, obstacle_patches
    for circle in obstacle_patches:
        circle.remove()
    obstacles.clear()
    obstacle_patches.clear()
    map_canvas.draw()
    update_obstacle_matrix()
    plan_path()  # Recalculate path after clearing obstacles

# Bind the click, drag, and release events to the map
map_canvas.mpl_connect('button_press_event', on_map_click)
map_canvas.mpl_connect('motion_notify_event', on_map_drag)
map_canvas.mpl_connect('button_release_event', on_map_release)

# Control Buttons
def switch_to_autonomous():
    # Code to switch to autonomous mode
    print("Switched to Autonomous")

def switch_to_manual():
    # Code to switch to manual mode
    print("Switched to Manual")

autonomous_button = ctk.CTkButton(control_frame, text="Switch to Autonomous", command=switch_to_autonomous)
autonomous_button.grid(row=0, column=0, padx=5, pady=5)

manual_button = ctk.CTkButton(control_frame, text="Switch to Manual", command=switch_to_manual)
manual_button.grid(row=0, column=1, padx=5, pady=5)

add_obstacle_button = ctk.CTkButton(control_frame, text="Add Obstacle", command=add_obstacle)
add_obstacle_button.grid(row=0, column=2, padx=5, pady=5)

clear_obstacles_button = ctk.CTkButton(control_frame, text="Clear Obstacles", command=clear_obstacles)
clear_obstacles_button.grid(row=0, column=3, padx=5, pady=5)

# Function to apply B-Spline interpolation
def b_spline_interpolation(path, num_points=100, smooth_factor=0):
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    tck, u = splprep([x, y], s=smooth_factor)
    u_new = np.linspace(u.min(), u.max(), num_points)
    x_new, y_new = splev(u_new, tck, der=0)
    
    # Ensure start and end points are included
    x_new[0], y_new[0] = x[0], y[0]
    x_new[-1], y_new[-1] = x[-1], y[-1]
    
    return x_new, y_new

# Function to trigger path planning and update the map
def plan_path():
    init = [0, 0]  # bottom left corner
    goal = [matrix_size[0] - 1, matrix_size[1] - 1]  # top right corner
    cost = 2
    path, action = search(obstacle_matrix, init, goal, cost)
    
    # Plot the path on the map
    path_x, path_y = zip(*path)
    path_x = [x * GRID_SIZE + GRID_SIZE / 2 for x in path_x]  # Convert grid indices to map coordinates
    path_y = [y * GRID_SIZE + GRID_SIZE / 2 for y in path_y]
    path_plot.set_data(path_x, path_y)
    
    # Apply B-Spline Interpolation with a lower smoothing factor
    x_smooth, y_smooth = b_spline_interpolation(path, num_points=200, smooth_factor=0.1)
    smoothed_path_plot.set_data(x_smooth, y_smooth)
    
    map_canvas.draw()

# Add Path Planning Button
plan_path_button = ctk.CTkButton(control_frame, text="Plan Path", command=plan_path)
plan_path_button.grid(row=0, column=4, padx=5, pady=5)

# Updating the Dashboard in Real-Time
def update_dashboard():
    # Simulate data updates
    for i in range(4):
        wheel_speeds[f"wheel_{i+1}"].set(f"{random.randint(0, 100)} RPM")
    linear_speed_var.set(f"{random.uniform(0, 15):.2f} m/s")
    battery_var.set(f"{random.randint(0, 100)}%")
    
    # Update rover position on the map
    rover_x = random.uniform(0, MAP_SIZE_X)
    rover_y = random.uniform(0, MAP_SIZE_Y)
    rover_position.set_data([rover_x], [rover_y])
    map_canvas.draw()
    
    root.after(1000, update_dashboard)  # Update every second

update_dashboard()

root.mainloop()
