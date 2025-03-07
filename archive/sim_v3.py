import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
import random
from beacons import Beacon
from Swarmalator import Swarmalator
from grid import Grid
from math import sqrt

N_GRID = 33
M_GRID = 33
NUM_SWARMALATORS = 30
B = 0.3
dt = 0.1
MIN_THRES = 0.3  # Initial minimum threshold
maxX = N_GRID // 2
minX = - (N_GRID // 2)
maxY = M_GRID // 2
minY = -(M_GRID // 2)

arr_beacons = []
arr_swarmalators = []
grid = [[Grid() for _ in range(N_GRID)] for _ in range(M_GRID)]

def set_grid_beacons():
    for i in range(minX, maxX + 1):
        for j in range(minY, maxY + 1):

            #rectangle in middle

            if j >=3 and j<= 5 and i>=3 and i<=6:
                beac = Beacon(i, j, beacon_j=1)
            else:
                beac = Beacon(i, j, beacon_j=0)

            grid[i][j].beacon = beac
            arr_beacons.append(beac)

def set_swarmalators():
    arr_swarmalators_old = []
    arr_swarmalators_new = []

    for _ in range(NUM_SWARMALATORS):
        x = minX + random.random() * (maxX - minX)
        y = minY + random.random() * (maxY - minY)
        swarmalator = Swarmalator(x, y)
        grid[int(x) + N_GRID // 2][int(y) + M_GRID // 2].swarmalator = swarmalator
        arr_swarmalators_old.append(swarmalator)

    print(len(arr_swarmalators_old))
    for i in range(2, NUM_SWARMALATORS):
        checkPoint = False
        while not checkPoint:
            possible_x = minX + random.random() * (maxX - minX)
            possible_y = minY + random.random() * (maxY - minY)
            checkPoint = True

            for j in range(i):
                dist = sqrt((arr_swarmalators_old[j].x - possible_x) ** 2 + (arr_swarmalators_old[j].y - possible_y) ** 2)

                if dist < 2 * arr_swarmalators_old[j].radius:
                    checkPoint = False
                    break

            if checkPoint:
                swarmalator = Swarmalator(possible_x, possible_y)
                grid[int(possible_x) + N_GRID // 2][int(possible_y) + M_GRID // 2].swarmalator = swarmalator
                arr_swarmalators_new.append(swarmalator)
    return arr_swarmalators_new


def simulate_with_thres(curr_swarmalator):

    dx = 0
    dy = 0
    dx_static = 0
    dy_static = 0
    num_bots_in_thres = 0
    num_static_in_thres = 0

    for swarmalator in arr_swarmalators:
        if not np.array_equal(curr_swarmalator, swarmalator):
            distBetweenBots = sqrt((curr_swarmalator.x-swarmalator.x)**2 + (curr_swarmalator.y-swarmalator.y)**2)
            dist_x = (curr_swarmalator.x-swarmalator.x)
            dist_y = (curr_swarmalator.y-swarmalator.y)
            if distBetweenBots < 1:
                distBetweenBots=1
            if distBetweenBots < 2 * curr_swarmalator.radius:
                dx -= 2.3 * (dist_x / (distBetweenBots * 0.2))
                dy -= 2.3 * (dist_y / (distBetweenBots * 0.2))
            else:
                dx -= B * (dist_x / (distBetweenBots ** 2))
                dy -= B * (dist_y / (distBetweenBots ** 2))
                if distBetweenBots < 3: #TODO
                    num_bots_in_thres +=1

    for beacon in arr_beacons:
        distBetweenBotAndStatic = sqrt((curr_swarmalator.x - beacon.x)**2 +  (curr_swarmalator.y - beacon.y)**2)
        if distBetweenBotAndStatic < 1:
            distBetweenBotAndStatic = 1
        dist_x = (curr_swarmalator.x-beacon.x)
        dist_y = (curr_swarmalator.y-beacon.y)
        curr_phase = curr_swarmalator.internal_freq
        if distBetweenBotAndStatic < 10:
            dx_static += ((dist_x / (distBetweenBotAndStatic**2)) * beacon.beacon_j *
                                            np.cos(beacon.internal_freq - curr_phase))
            dy_static += ((dist_y / (distBetweenBotAndStatic**2)) * beacon.beacon_j *
                                            np.cos(beacon.internal_freq - curr_phase))
            num_static_in_thres +=1

    if num_bots_in_thres == 0:
        num_bots_in_thres = 1
    if num_static_in_thres == 0:
        num_static_in_thres +=1

    curr_swarmalator.x += (dx / num_bots_in_thres) * dt + (dx_static / num_static_in_thres) * dt
    curr_swarmalator.y += (dy / num_bots_in_thres) * dt + (dy_static / num_static_in_thres) * dt

if __name__ == "__main__":
    set_grid_beacons()
    arr_swarmalators = set_swarmalators()

    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)

    ax.set_xlim(minX, maxX)
    ax.set_ylim(minY, maxY)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.grid(True)

    beacon_j_values = [em.beacon_j for em in arr_beacons]

    Beacon_scatter = ax.scatter(
        [em.x for em in arr_beacons],
        [em.y for em in arr_beacons],
        c=beacon_j_values,
        cmap='viridis',
        marker='x',
        label='Beacons'
    )

    cbar = plt.colorbar(Beacon_scatter, ax=ax)
    cbar.set_label('Beacon J Value')

    swarmalator_scatter = ax.scatter(
        [swarmalator.x for swarmalator in arr_swarmalators],
        [swarmalator.y for swarmalator in arr_swarmalators],
        c='red', marker='o', label='Swarmalators'
    )
    ax.legend()

    def update(frame):
        # for swarmalator in arr_swarmalators:
        #     simulate_with_thres(swarmalator)

       swarmalator_scatter.set_offsets(
            np.c_[[swarmalator.x for swarmalator in arr_swarmalators],
                [swarmalator.y for swarmalator in arr_swarmalators]].T
        )


    ani = FuncAnimation(fig, update, frames=100, interval=100)


    ax_thres = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    slider_thres = Slider(ax_thres, 'Min Thres', 0.05, 10.0, valinit=MIN_THRES)


    def update_threshold(val):
        global MIN_THRES
        MIN_THRES = val

    slider_thres.on_changed(update_threshold)
    plt.show()