import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
import random

from beacons import Beacon
from Swarmalator import Swarmalator
from math import sqrt

N_GRID = 10
M_GRID = 10
maxX = N_GRID // 2
minX = - (N_GRID // 2)
maxY = M_GRID // 2
minY = -(M_GRID // 2)

NUM_SWARMALATORS = 30
B = 0.3
dt = 1
MIN_THRES = 0.3

arr_beacons = []
arr_swarmalators_new = []

def set_grid_beacons():
    for i in range(minX, maxX + 1):
        for j in range(minY, maxY + 1):
            if -2 <= j <= 2 and -2 <= i <= 2:
                beac = Beacon(i, j, beacon_j=1)
            else:
                beac = Beacon(i, j, beacon_j=0)

            arr_beacons.append(beac)

def set_swarmalators():
    arr_swarmalators_old = []
    arr_swarmalators_new = []

    for _ in range(NUM_SWARMALATORS):
        x = minX + random.random() * (maxX - minX)
        y = minY + random.random() * (maxY - minY)
        swarmalator = Swarmalator(x, y)
        arr_swarmalators_old.append(swarmalator)

    for i in range(NUM_SWARMALATORS):
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
                arr_swarmalators_new.append(swarmalator)
    return arr_swarmalators_new

def total_movement_and_phase_calcs(arr_swarmalators, dt):

    for i in range(len(arr_swarmalators)):

        curr_swarmalator = arr_swarmalators[i]
        curr_swarmalator.dx = 0
        curr_swarmalator.dy = 0
        curr_swarmalator.dx_static = 0
        curr_swarmalator.dy_static = 0
        curr_swarmalator.num_bots_in_thres = 0
        curr_swarmalator.num_static_in_thres = 0

        for j in range(len(arr_swarmalators)):
            if i != j:
                swarmalator = arr_swarmalators[j]
                distBetweenBots = sqrt((curr_swarmalator.x-swarmalator.x)**2 + (curr_swarmalator.y-swarmalator.y)**2)
                dist_x = swarmalator.x - curr_swarmalator.x
                dist_y = swarmalator.y - curr_swarmalator.y
                if distBetweenBots < 2 * curr_swarmalator.radius:
                    curr_swarmalator.dx = curr_swarmalator.dx - 2.3 * (dist_x / (distBetweenBots * 0.2+0.1))
                    curr_swarmalator.dy = curr_swarmalator.dy - 2.3 * (dist_y / (distBetweenBots * 0.2+0.1))
                else:
                    curr_swarmalator.dx = curr_swarmalator.dx - B * (dist_x / (distBetweenBots ** 2))
                    curr_swarmalator.dy = curr_swarmalator.dy - B * (dist_y / (distBetweenBots ** 2))
                curr_swarmalator.num_bots_in_thres +=1

        for beacon in arr_beacons:
            distBetweenBotAndStatic = sqrt((beacon.x - curr_swarmalator.x)**2 + (beacon.y - curr_swarmalator.y)**2)
            dist_x = beacon.x - curr_swarmalator.x
            dist_y = beacon.y - curr_swarmalator.y
            curr_phase = curr_swarmalator.internal_freq
            curr_swarmalator.dx_static = curr_swarmalator.dx_static + ((dist_x / (distBetweenBotAndStatic**2+0.1)) * beacon.beacon_j *
                                            np.cos(beacon.internal_freq - curr_phase))
            curr_swarmalator.dy_static = curr_swarmalator.dy_static + ((dist_y / (distBetweenBotAndStatic**2+0.1)) * beacon.beacon_j *
                                            np.cos(beacon.internal_freq - curr_phase))
            curr_swarmalator.num_static_in_thres +=1

        if curr_swarmalator.num_bots_in_thres == 0:
            curr_swarmalator.num_bots_in_thres = 1
        if curr_swarmalator.num_static_in_thres == 0:
            curr_swarmalator.num_static_in_thres =1

    for agent in arr_swarmalators:
        agent.x += (agent.dx / agent.num_bots_in_thres) * dt + (agent.dx_static / agent.num_static_in_thres) * dt
        agent.y += (agent.dy / agent.num_bots_in_thres) * dt + (agent.dy_static / agent.num_static_in_thres) * dt


def update(frame):
    total_movement_and_phase_calcs(arr_swarmalators, dt)
    scat.set_offsets([(s.x, s.y) for s in arr_swarmalators])
    return scat,

if __name__ == "__main__":

    set_grid_beacons()
    arr_swarmalators = set_swarmalators()

    fig, ax = plt.subplots()
    ax.set_xlim(2*minX, 2*maxX)
    ax.set_ylim(2*minY, 2*maxY)

    beacon_x = [beacon.x for beacon in arr_beacons]
    beacon_y = [beacon.y for beacon in arr_beacons]
    beacon_colors = plt.cm.viridis([beacon.beacon_j / max(beacon.beacon_j for beacon in arr_beacons) for beacon in arr_beacons])

    sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=min(beacon.beacon_j for beacon in arr_beacons), vmax=max(beacon.beacon_j for beacon in arr_beacons)))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Beacon Strength (beacon_j)')
    ax.scatter(beacon_x, beacon_y, c=beacon_colors, label='Beacons')

    swarmalator_x = [swarmalator.x for swarmalator in arr_swarmalators]
    swarmalator_y = [swarmalator.y for swarmalator in arr_swarmalators]
    scat = ax.scatter(swarmalator_x, swarmalator_y, c='blue', label='Swarmalators')
    ax.legend()

    ani = FuncAnimation(fig, update, frames=100, interval=100, blit=True)

    plt.legend()
    plt.show()
