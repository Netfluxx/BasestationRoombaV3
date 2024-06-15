import tkinter as tk
import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import cv2
from PIL import Image, ImageTk
import random
import time
from path_planning import search

# Global Variables
MAP_SIZE = 100  # Size of the map
OBSTACLE_SIZE = 2  # Size of the obstacle in grid units

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
grid_size = 1  # Finer grid (1x1 grid cells)
matrix_size = (MAP_SIZE // grid_size, MAP_SIZE // grid_size)
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
ax.set_xlim(0, MAP_SIZE)  # Adjust as needed
ax.set_ylim(0, MAP_SIZE)  # Adjust as needed

# Draw the grid
for x in range(0, MAP_SIZE + 1, grid_size):
    ax.plot([x, x], [0, MAP_SIZE], color='gray', linewidth=0.5)
for y in range(0, MAP_SIZE + 1, grid_size):
    ax.plot([0, MAP_SIZE], [y, y], color='gray', linewidth=0.5)

map_canvas = FigureCanvasTkAgg(fig, master=map_frame)
map_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)
rover_position, = ax.plot([], [], 'bo')  # Rover's position marker
path_plot, = ax.plot([], [], 'g-')  # Placeholder for path
obstacles = []
obstacle_patches = []
adding_obstacle = False
mouse_pressed = False  # Track mouse press state

def update_obstacle_matrix():
    global obstacle_matrix
    obstacle_matrix = [[0 for _ in range(matrix_size[1])] for _ in range(matrix_size[0])]
    for rect in obstacles:
        ox, oy = rect.xy    # Bottom-left corner of the rectangle
        # Convert and snap to grid coordinates
        grid_x = int(ox // grid_size)
        grid_y = int(oy // grid_size)
        for i in range(OBSTACLE_SIZE):  # OBSTACLE_SIZE x OBSTACLE_SIZE cells for each obstacle
            for j in range(OBSTACLE_SIZE):
                if grid_x + i < matrix_size[0] and grid_y + j < matrix_size[1]:
                    obstacle_matrix[grid_x + i][grid_y + j] = 1  # Obstacle cost
    print(obstacle_matrix)

def on_map_click(event):
    global adding_obstacle, mouse_pressed
    if adding_obstacle:
        mouse_pressed = True
        # Snap to grid
        obs_x = int(event.xdata // grid_size) * grid_size
        obs_y = int(event.ydata // grid_size) * grid_size
        if obs_x is not None and obs_y is not None:
            rect = plt.Rectangle((obs_x, obs_y), OBSTACLE_SIZE * grid_size, OBSTACLE_SIZE * grid_size, edgecolor='r', facecolor='r')
            obstacles.append(rect)
            ax.add_patch(rect)
            obstacle_patches.append(rect)
            map_canvas.draw()
            update_obstacle_matrix()
            plan_path()  # Recalculate path when a new obstacle is added

def on_map_drag(event):
    global mouse_pressed
    if adding_obstacle and mouse_pressed and event.xdata is not None and event.ydata is not None:
        obs_x = int(event.xdata // grid_size) * grid_size
        obs_y = int(event.ydata // grid_size) * grid_size
        rect = plt.Rectangle((obs_x, obs_y), OBSTACLE_SIZE * grid_size, OBSTACLE_SIZE * grid_size, edgecolor='r', facecolor='r')
        obstacles.append(rect)
        ax.add_patch(rect)
        obstacle_patches.append(rect)
        update_obstacle_matrix()
        plan_path()  # Recalculate path when a new obstacle is added

def on_map_release(event):
    global mouse_pressed
    mouse_pressed = False

def add_obstacle():
    global adding_obstacle
    adding_obstacle = True

def clear_obstacles():
    global obstacles, obstacle_patches
    for rect in obstacle_patches:
        rect.remove()
    obstacles.clear()
    obstacle_patches.clear()
    map_canvas.draw()
    update_obstacle_matrix()
    plan_path()  # Recalculate path after clearing obstacles

# Bind the click, drag, and release events to the map
map_canvas.mpl_connect('button_press_event', on_map_click)
map_canvas.mpl_connect('motion_notify_event', on_map_drag)
map_canvas.mpl_connect('button_release_event', on_map_release)

# Live Camera Stream
camera_label = ctk.CTkLabel(camera_frame)
camera_label.pack()

def update_camera():
    ret, frame = camera.read()
    if ret:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        ctk_img = ctk.CTkImage(light_image=img)
        camera_label.configure(image=ctk_img)
        camera_label.image = ctk_img  # Keep a reference to avoid garbage collection
    root.after(10, update_camera)

camera = cv2.VideoCapture(0)
update_camera()

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

# Function to trigger path planning and update the map
def plan_path():
    init = [0, 0]  # bottom left corner
    goal = [matrix_size[0] - 1, matrix_size[1] - 1]  # top right corner
    cost = 2
    path, action = search(obstacle_matrix, init, goal, cost)
    
    # Plot the path on the map
    path_x, path_y = zip(*path)
    path_x = [x * grid_size + grid_size / 2 for x in path_x]  # Convert grid indices to map coordinates
    path_y = [y * grid_size + grid_size / 2 for y in path_y]
    path_plot.set_data(path_x, path_y)
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
    rover_x = random.randint(0, MAP_SIZE)
    rover_y = random.randint(0, MAP_SIZE)
    rover_position.set_data([rover_x], [rover_y])
    map_canvas.draw()
    
    root.after(1000, update_dashboard)  # Update every second

update_dashboard()

root.mainloop()
