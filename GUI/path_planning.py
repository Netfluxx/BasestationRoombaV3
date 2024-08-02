import math

ROVER_SIZE = 2 #2x2 square on the grid


DIRECTIONS = [  # [delta_x, delta_y]
    [-1, 0],    # left
    [0, -1],    # down
    [1, 0],     # right
    [0, 1],     # up
    [1, 1],     # top right diag
    [-1, 1],    # top left diag
    [1, -1],    # bottom right diag
    [-1, -1],   # bottom left diag
]

def get_heuristic(x, y, goal_x, goal_y):
    return math.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)

def check_in_bounds(x, y, grid):
    if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
        return True
    return False


def check_cell(x, y, closed, grid):
    if check_in_bounds(x, y, grid):
        if closed[x][y] == 0 and grid[x][y] == 0:
            #check aroudn the current position to see if there is enough spac efor the rover to fit and/or turn
            
            ####IMPlEMENT HERE

            return True
    return False

def search(grid, init, goal, cost):
    #grid : matrix of obstacles, example:
    #   [0, 0, 0, 0, 0, 0],
    #   [0, 0, 1, 0, 0, 0],
    #   [0, 0, 1, 0, 1, 0],
    #   [0, 0, 0, 0, 1, 0],
    #   [0, 0, 1, 0, 0, 0]]

    #closed : 2D array that shows which cells have been visited
    #action : 2D array that shows which action was taken at that cell
    #init : starting point
    #goal : ending point

    heuristic = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            heuristic[i][j] = get_heuristic(i, j, goal[0], goal[1])
            if grid[i][j] == 1:
                # obstacle --> extra penalty in the heuristic map
                heuristic[i][j] += 99

    closed = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    closed[init[0]][init[1]] = 1

    action = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]

    x, y = init
    g = 0                   # Cost to get to the current cell from the start
    h = heuristic[x][y]     # Heuristic: pythagorean distance to goal
    f = g + h               # Total cost
    cell = [[f, g, x, y]]   # Current cell
    
    found = False
    resign = False
    
    while not found and not resign:
        if not cell or len(cell) == 0:
            raise ValueError("A* unable to find solution")
        else:
            cell.sort()     # Sorts in ascending order with respect to f
            cell.reverse()  # Last cell now has the smallest cost
            next_cell = cell.pop()
            x, y, g = next_cell[2], next_cell[3], next_cell[1]
            
            if [x, y] == goal:
                found = True
            else:
                for i in range(len(DIRECTIONS)):
                    new_x, new_y = x + DIRECTIONS[i][0], y + DIRECTIONS[i][1]
                    if check_cell(new_x, new_y, closed, grid):
                        g2 = g + cost
                        h2 = heuristic[new_x][new_y]
                        f2 = g2 + h2
                        cell.append([f2, g2, new_x, new_y])
                        closed[new_x][new_y] = 1   # Mark as visited
                        action[new_x][new_y] = i   # Store the direction taken at this cell
    
    reversed_path = []
    x, y = goal
    reversed_path.append([x, y])
    while [x, y] != init:
        x2 = x - DIRECTIONS[action[x][y]][0]
        y2 = y - DIRECTIONS[action[x][y]][1]
        x, y = x2, y2
        reversed_path.append([x, y])
    
    path = reversed_path[::-1]
    return path, action
