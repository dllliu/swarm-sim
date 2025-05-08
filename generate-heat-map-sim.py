import numpy as np
import pygame
import random
import time

from beacons import Beacon
from Swarmalator import Swarmalator
from mask_creator import MaskCreator
import matplotlib.pyplot as plt

maxX = 800 #screen width and height
maxY  = 800
SCALE = int(maxX / 33)

NUM_SWARMALATORS = 300
B = 0.4
dt = 2

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

class Simulation():
    def __init__(self, num_agents, agent_speed):
        self.arr_beacons = []
        self.arr_swarmalators = pygame.sprite.Group()
        self.new_beacon_j= 6
        self.thres_dist = 2 #beacon threshold distance from a swarm agent to activate, set_grid_beacons uses this
        self.num_agents = num_agents
        self.agent_speed = agent_speed
        self.set_swarmalators()
        self.obstacles = pygame.sprite.Group()
        self.radius = 1 #for circular paths
        self.screen = pygame.display.set_mode((maxX, maxY))
        self.mask_creator = MaskCreator(maxX, maxY)
        self.y_bottom = 50
        self.y_top = 300

    def set_grid_beacons(self, mask):
        self.arr_beacons.clear()
        for i in range(0, 33):
            for j in range(0, 33):
                if mask.get_at((i*SCALE, j*SCALE)):
                    beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                else:
                    beac = Beacon(i, j, 0, self.thres_dist)
                self.arr_beacons.append(beac)

    def set_swarmalators(self):
        while len(self.arr_swarmalators) < self.num_agents:
            i = random.uniform(3, 12)
            j = random.uniform(3, 12)
            #if all(sqrt((s.x - i) ** 2 + (s.y - j) ** 2) >= 2 * s.radius for s in self.arr_swarmalators):
            swarmalator = Swarmalator(i, j)
            self.arr_swarmalators.add(swarmalator)

    def total_movement_and_phase_calcs(self):
        swarmalators_positions = np.array([[s.x, s.y] for s in self.arr_swarmalators])
        sim_rec_per_dt = []

        for curr_swarmalator in self.arr_swarmalators:
            curr_swarmalator.dx = 0
            curr_swarmalator.dy = 0
            curr_swarmalator.dx_static = 0
            curr_swarmalator.dy_static = 0
            curr_swarmalator.num_bots_in_thres = 0
            curr_swarmalator.num_static_in_thres = 0
            curr_swarmalator.v_x = 0
            curr_swarmalator.v_y = 0

            diffs = swarmalators_positions - np.array([curr_swarmalator.x, curr_swarmalator.y])
            dists = np.linalg.norm(diffs, axis=1)

            mask_near = (dists <= 2 * curr_swarmalator.radius)
            mask_far = (dists > 2 * curr_swarmalator.radius)

            if np.any(mask_near):
                diffs_near = diffs[mask_near]
                dists_near = dists[mask_near]
                influence = 2.3 * (diffs_near.T / (dists_near * 0.2 + 0.1)).T
                curr_swarmalator.dx -= np.sum(influence[:, 0])
                curr_swarmalator.dy -= np.sum(influence[:, 1])

            if np.any(mask_far):
                diffs_far = diffs[mask_far]
                dists_far = dists[mask_far]
                influence_far = B * (diffs_far.T / (dists_far ** 2)).T
                curr_swarmalator.dx -= np.sum(influence_far[:, 0])
                curr_swarmalator.dy -= np.sum(influence_far[:, 1])

            curr_swarmalator.num_bots_in_thres += np.count_nonzero(mask_near) + np.count_nonzero(mask_far)

            for beacon in self.arr_beacons:
                dist_x = beacon.x - curr_swarmalator.x
                dist_y = beacon.y - curr_swarmalator.y
                dist_between_bot_and_static = np.sqrt(dist_x ** 2 + dist_y ** 2)
                curr_phase = curr_swarmalator.internal_freq
                curr_swarmalator.dx_static += (dist_x / (dist_between_bot_and_static ** 2 + 0.1)) * beacon.beacon_j * np.cos(beacon.internal_freq - curr_phase)
                curr_swarmalator.dy_static += (dist_y / (dist_between_bot_and_static ** 2 + 0.1)) * beacon.beacon_j * np.cos(beacon.internal_freq - curr_phase)
                curr_swarmalator.num_static_in_thres += 1

            curr_swarmalator.num_bots_in_thres = max(curr_swarmalator.num_bots_in_thres, 1)
            curr_swarmalator.num_static_in_thres = max(curr_swarmalator.num_static_in_thres, 1)

            curr_swarmalator.v_x = (curr_swarmalator.dx / curr_swarmalator.num_bots_in_thres) * dt + (curr_swarmalator.dx_static / curr_swarmalator.num_static_in_thres) * dt
            curr_swarmalator.v_y = (curr_swarmalator.dy / curr_swarmalator.num_bots_in_thres) * dt + (curr_swarmalator.dy_static / curr_swarmalator.num_static_in_thres) * dt

            curr_swarmalator.x += curr_swarmalator.v_x
            curr_swarmalator.y += curr_swarmalator.v_y
            curr_swarmalator.update(maxX, maxY)
            sim_rec_per_dt.append((curr_swarmalator.x, curr_swarmalator.y))

    def step(self):
        self.polygon_points = [(100, self.y_bottom), (300, self.y_bottom), (300, self.y_top), (100, self.y_top)]
        self.y_top += self.agent_speed
        self.y_bottom += self.agent_speed
        mask = self.mask_creator.create_mask_from_polygon(self.polygon_points)
        self.set_grid_beacons(mask)
        self.total_movement_and_phase_calcs()

    def count_left_behind(self):
        num_left = 0
        for agent in self.arr_swarmalators:
            if agent.x * SCALE < 100 or agent.x* SCALE > 300 or agent.y* SCALE < self.y_bottom or agent.y* SCALE > self.y_top:
                num_left +=1
        return num_left

if __name__ == "__main__":
    velocities = np.arange(0.1, 2.1, 0.1)
    agent_counts = np.arange(10, 201, 10)
    heatmap_data = np.zeros((len(agent_counts), len(velocities)))
    time_steps = 100

    for i, n_agents in enumerate(agent_counts):
        for j, vel in enumerate(velocities):
            print(f"Running for num={n_agents} and vel={vel}")
            sim = Simulation(n_agents, vel)
            left_behind_over_time = []

            for _ in range(time_steps):
                sim.step()
                count = sim.count_left_behind()
                left_behind_over_time.append(count)

            metric = np.mean(left_behind_over_time)
            print(f"mean for num={n_agents} and vel={vel} is {metric}")
            heatmap_data[i, j] = metric

    plt.figure(figsize=(12, 8))
    plt.imshow(heatmap_data, cmap='hot', interpolation='nearest', origin='lower')
    plt.colorbar(label='Agents Left Behind')
    plt.xticks(np.arange(len(velocities)), velocities, rotation=45)
    plt.yticks(np.arange(len(agent_counts)), agent_counts)
    plt.xlabel('Velocity')
    plt.ylabel('Number of Agents')
    plt.title('Agents Left Behind per Velocity and Agent Count')
    plt.tight_layout()
    plt.show()
    plt.savefig("heat-map.png")
