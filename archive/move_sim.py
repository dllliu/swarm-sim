import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
import random
import time

from beacons import Beacon
from Swarmalator import Swarmalator
from boundary_manager import BoundaryManager
from obstacle import Obstacle_Manager
from math import sqrt

N_GRID = 15
M_GRID = 15
maxX = N_GRID // 2
minX = - (N_GRID // 2)
maxY = M_GRID // 2
minY = -(M_GRID // 2)

NUM_SWARMALATORS = 30
B = 0.3
dt = 1
MIN_THRES = 0.3

arr_beacons = []

def set_grid_beacons():
    for i in range(minX, maxX + 1):
        for j in range(minY, maxY + 1):
            if -2 <= j <= 0 and -2 <= i <= 0:
                beac = Beacon(i, j, beacon_j=20)
            else:
                beac = Beacon(i, j, beacon_j=0)

            arr_beacons.append(beac)

def set_obstacles():
    arr_obs = []
    for i in range(2,7):
        for j in range(-3,2,2):
            arr_obs.append([j,i])
    return arr_obs


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

def total_movement_and_phase_calcs(arr_swarmalators, dt, obstacles):

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

        if obstacles.is_near(agent.x, agent.y):
            obstacle_x, obstacle_y = obstacles.get_nearest_point(agent.x, agent.y)
            dist_x = agent.x - obstacle_x
            dist_y = agent.y - obstacle_y
            dist = max(sqrt(dist_x**2 + dist_y**2), 1e-6)
            repulsion_strength = 30
            agent.dx += repulsion_strength * (dist_x / dist)
            agent.dy += repulsion_strength * (dist_y / dist)

        agent.x += (agent.dx / agent.num_bots_in_thres) * dt + (agent.dx_static / agent.num_static_in_thres) * dt
        agent.y += (agent.dy / agent.num_bots_in_thres) * dt + (agent.dy_static / agent.num_static_in_thres) * dt

if __name__ == "__main__":

    set_grid_beacons()
    arr_swarmalators = set_swarmalators()
    set_time = time.time()

    obstacles = set_obstacles()
    obstacle_x = [obs[0] for obs in obstacles]
    obstacle_y = [obs[1] for obs in obstacles]
    obstacles = Obstacle_Manager(obstacles)

    mean_x = np.mean([swarmalator.x for swarmalator in arr_swarmalators])
    mean_y = np.mean([swarmalator.y for swarmalator in arr_swarmalators])

    boundary_speed = 0.08
    boundary_direction = 7 * (np.pi) / 12

    curr_boundary = BoundaryManager(mean_x, mean_y, boundary_speed, boundary_direction, minX, maxX, minY, maxY, thres_dist=1.5)

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
    beacon_scat = ax.scatter(beacon_x, beacon_y, c=beacon_colors, label='Beacons')

    swarmalator_x = [swarmalator.x for swarmalator in arr_swarmalators]
    swarmalator_y = [swarmalator.y for swarmalator in arr_swarmalators]
    scat = ax.scatter(swarmalator_x, swarmalator_y, c='blue', label='Swarmalators')

    obstacle_scat = ax.scatter(obstacle_x, obstacle_y, c='red', label='Obstacles')

    ax.legend()

    def update(frame):
        total_movement_and_phase_calcs(arr_swarmalators, dt, obstacles)
        if time.time() - set_time > 10:
            curr_boundary.move_boundary(dt)
            curr_boundary.update_beacons(arr_beacons)
        scat.set_offsets([(s.x, s.y) for s in arr_swarmalators])
        beacon_colors = plt.cm.viridis([beacon.beacon_j / max(beacon.beacon_j for beacon in arr_beacons) for beacon in arr_beacons])
        beacon_scat.set_color(beacon_colors)
        return scat, beacon_scat, obstacle_scat

    ani = FuncAnimation(fig, update, frames=100, interval=100, blit=True)

    plt.legend()
    plt.show()
