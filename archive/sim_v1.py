import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

N_GRID = 30  # Grid dimensions (NxM)
M_GRID = 30
NUM_ELECTROMAGNETS = 10
NUM_SWARMALATORS = 60
#J = 1  #1 enables agents with similar phases to be attracted, repusulsive if neagtive
J=-0.9
B = 1 #agents do not aggregate at a certain point
dt = 2 # Time step
MIN_THRES = 6

# Arrays to hold electromagnets and swarmalatorsss
arr_ems = []
arr_swarmalators = []

class Swarmalator:
    def __init__(self, position):
        self.internal_phase = 0  # theta
        self.position = position.astype(float)  # Ensure position is float
        self.velocity = np.array([0.0, 0.0], dtype=float)

class Electromagnet:
    def __init__(self, position):
        self.position = position
        self.internal_phase = 1

def set_grid():
    grid = [[None] * N_GRID for _ in range(M_GRID)]
    for _ in range(NUM_ELECTROMAGNETS):
        x = random.randint(N_GRID//2+1, N_GRID-1)
        y = random.randint(M_GRID//2+1, M_GRID-1)
        while grid[x][y] is not None: #do not place on same
            x = random.randint(0, N_GRID - 1)
            y = random.randint(0, M_GRID - 1)
        grid[x][y] = Electromagnet(np.array([x, y]))
        arr_ems.append(grid[x][y])
    return grid

def make_arr_ems(grid):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] is not None:
                arr_ems.append(grid[i][j])

def set_grid_filled_rectangle():
    grid = [[None] * N_GRID for _ in range(M_GRID)]

    # Calculate center
    center_x, center_y = M_GRID // 2, N_GRID // 2

    for i in range(center_x - 5, center_x + 5, 2):  # Adjust range as needed
        for j in range(center_y - 5, center_y + 5, 2):  # Adjust range as needed
            em = Electromagnet(np.array([i, j]))
            grid[i][j] = em
            arr_ems.append(em)

    return grid


def set_grid_square():
    grid = [[None] * N_GRID for _ in range(M_GRID)]

    # Calculate center
    center_x, center_y = M_GRID // 2, N_GRID // 2

    # Place magnets around the center
    grid[center_x - 2][center_y - 2] = Electromagnet(np.array([center_x - 2, center_y - 2]))
    grid[center_x - 2][center_y + 2] = Electromagnet(np.array([center_x - 2, center_y + 2]))
    grid[center_x + 2][center_y - 2] = Electromagnet(np.array([center_x + 2, center_y - 2]))
    grid[center_x + 2][center_y + 2] = Electromagnet(np.array([center_x + 2, center_y + 2]))

    make_arr_ems(grid)
    return grid


def set_grid_triangle():
    grid = [[None] * N_GRID for _ in range(M_GRID)]

    # Calculate center
    center_x, center_y = M_GRID // 2, N_GRID // 2

    # Place magnets forming a triangle near the center
    grid[center_x - 2][center_y - 3] = Electromagnet(np.array([center_x - 2, center_y - 3]))
    grid[center_x - 3][center_y - 1] = Electromagnet(np.array([center_x - 3, center_y - 1]))
    grid[center_x - 3][center_y - 2] = Electromagnet(np.array([center_x - 3, center_y - 2]))
    grid[center_x + 1][center_y + 2] = Electromagnet(np.array([center_x + 1, center_y + 2]))

    make_arr_ems(grid)
    return grid

def set_grid_2_electroMags_vertical():
    grid = [[None] * N_GRID for _ in range(M_GRID)]

    # Calculate center
    center_x, center_y = M_GRID // 2, N_GRID // 2

    # Place magnets vertically near the center
    grid[center_x - 1][center_y - 3] = Electromagnet(np.array([center_x - 1, center_y - 3]))
    grid[center_x - 1][center_y + 3] = Electromagnet(np.array([center_x - 1, center_y + 3]))
    arr_ems.append(Electromagnet(np.array([center_x - 1, center_y - 3])))
    arr_ems.append(Electromagnet(np.array([center_x - 1, center_y + 3])))

    return grid


def set_grid_2_electroMags_horizontal():
    grid = [[None] * N_GRID for _ in range(M_GRID)]

    # Calculate center
    center_x, center_y = M_GRID // 2, N_GRID // 2

    # Place magnets horizontally near the center
    grid[center_x - 2][center_y] = Electromagnet(np.array([center_x - 2, center_y]))
    grid[center_x + 4][center_y] = Electromagnet(np.array([center_x + 4, center_y]))
    arr_ems.append(Electromagnet(np.array([center_x - 2, center_y])))
    arr_ems.append(Electromagnet(np.array([center_x + 4, center_y])))

    return grid



def set_swarmalators(grid):
    for _ in range(NUM_SWARMALATORS):
        x = random.randint(0, N_GRID-1)
        y = random.randint(0, M_GRID-1)
        while grid[x][y] is not None: #do not place on same
            x = random.randint(0, N_GRID - 1)
            y = random.randint(0, M_GRID - 1)
        swarmalator = Swarmalator(np.array([x, y]))
        grid[x][y] = swarmalator
        arr_swarmalators.append(swarmalator)


def unit_vector(vec):
    norm = np.linalg.norm(vec)
    return vec / (norm)  # Avoid division by zero

def simulate_with_thres(curr_swarmalator, dt):
    curr_theta = curr_swarmalator.internal_phase
    curr_pos = curr_swarmalator.position

    change_in_pos = np.array([0.0, 0.0])

    for other_swarmalator in arr_swarmalators:
        if not np.array_equal(curr_pos, other_swarmalator.position):
            diff_pos = other_swarmalator.position - curr_pos
            dist = np.linalg.norm(diff_pos)
            if dist > MIN_THRES:
                spatial_attraction = diff_pos / dist * (1 + J * np.cos(other_swarmalator.internal_phase - curr_theta))
                change_in_pos += spatial_attraction
                repulsion = -B * (diff_pos / (dist ** 2+1e-4))
                change_in_pos += repulsion

    for electromagnet in arr_ems:
        if not np.array_equal(curr_pos, electromagnet.position):
            diff_pos = electromagnet.position - curr_pos
            dist = np.linalg.norm(diff_pos)
            if dist > 1e-3:
                em_force = (diff_pos / (dist ** 2+ 1e-4)) * (1+J * np.cos(curr_swarmalator.internal_phase - electromagnet.internal_phase))
                change_in_pos += em_force

    new_position = curr_swarmalator.position + (1 / NUM_SWARMALATORS) * change_in_pos * dt
    curr_swarmalator.position = new_position

if __name__ == "__main__":
    #grid = set_grid()
    grid = set_grid_triangle()
    set_swarmalators(grid)

    fig, ax = plt.subplots()
    ax.set_xlim(- N_GRID*1.25, N_GRID*1.25)
    ax.set_ylim(-M_GRID*1.25, M_GRID*1.25)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.grid(True)


    electromagnet_scatter = ax.scatter(
        [em.position[0] for em in arr_ems],
        [em.position[1] for em in arr_ems],
        c='blue', marker='x', label='Electromagnets'
    )
    swarmalator_scatter = ax.scatter(
        [swarmalator.position[0] for swarmalator in arr_swarmalators],
        [swarmalator.position[1] for swarmalator in arr_swarmalators],
        c='red', marker='o', label='Swarmalators'
    )
    #ax.legend()

    def update(frame):
        for swarmalator in arr_swarmalators:
            simulate_with_thres(swarmalator, dt)

        swarmalator_scatter.set_offsets(
            [swarmalator.position for swarmalator in arr_swarmalators]
        )


    ani = FuncAnimation(fig, update, frames=100, interval=100)
    plt.show()