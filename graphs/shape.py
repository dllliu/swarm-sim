import numpy as np
import pygame
import random
import time
from math import sqrt
import json

from beacons import Beacon
from Swarmalator import Swarmalator

maxX = 800 #screen width and height
maxY  = 800
SCALE = int(maxX / 33)

NUM_SWARMALATORS = 300
B = 0.4
dt = 20

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

class Simulation:
    def __init__(self):
        self.arr_beacons = []
        self.arr_swarmalators = pygame.sprite.Group()
        self.new_beacon_j=6
        self.thres_dist = 2 #beacon threshold distance from a swarm agent to activate, set_grid_beacons uses this
        self.set_grid_beacons("m")
        self.set_swarmalators()
        self.set_time = time.time()
        self.boundary_speed = 0.075
        self.boundary_direction = -(np.pi) / 2
        self.sim_record = {}
        # self.boundary_control_points = [BoundaryPoint(33//2-6, 33//2, self.boundary_speed, np.pi/4, 0, 33, 0, 33),
        #                                 BoundaryPoint(33//2+5, 33//2, self.boundary_speed, -np.pi/4, 0, 33, 0, 33)]
        self.radius = 1 #for circular paths
        #self.init_obstacles()

    def set_grid_beacons(self, shape):
        for i in range(0, 33):
            for j in range(0, 33):
                if shape == "triangle":
                    if j<= i:
                        beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                    else:
                        beac = Beacon(i, j, 0, self.thres_dist)
                elif shape == "m":
                    if i <= 2 or i >= 30 or (i == j and i <= 14) or (j == 32 - i and i >= 15) or (i == j - 1 and i <= 14) or (j == 32 - i + 1 and i >= 15):
                        beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                    else:
                        beac = Beacon(i, j, 0, self.thres_dist)
                elif shape == "x":
                    if j == i or j == 32 - i or abs(i - j) == 1 or abs(i - (32 - j)) == 1:
                        beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                    else:
                        beac = Beacon(i, j, 0, self.thres_dist)
                elif shape == "arrow":
                    if j <= 16 and i >= 16 - j and i <= 16 + j:
                        beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                    elif i >= 12 and i <= 18:
                        beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                    else:
                        beac = Beacon(i, j, 0, self.thres_dist)
                self.arr_beacons.append(beac)

    def set_swarmalators(self):
        while len(self.arr_swarmalators) < NUM_SWARMALATORS:
            i = random.uniform(0, 33)
            j = random.uniform(0, 33)
            #if all(sqrt((s.x - i) ** 2 + (s.y - j) ** 2) >= 2 * s.radius for s in self.arr_swarmalators):
            swarmalator = Swarmalator(i, j)
            self.arr_swarmalators.add(swarmalator)



    def total_movement_and_phase_calcs(self, frame_count):
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
            sim_rec_per_dt.append((curr_swarmalator.x, curr_swarmalator.y))

        self.sim_record[frame_count] = sim_rec_per_dt


    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((maxX, maxY))
        clock = pygame.time.Clock()
        running = True

        FRAME_RATE = 60
        frame_count = 0

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill((255, 255, 255))

            if frame_count % 20 == 0: #physics updates happen once every 20 frames
                self.total_movement_and_phase_calcs(frame_count)

            # for beacon in self.arr_beacons:
            #     if beacon.beacon_j > 0:
            #         pygame.draw.circle(screen, (0, 255, 0), (int(beacon.x * SCALE), int(beacon.y * SCALE)), 5)

            for swarmalator in self.arr_swarmalators:
                pygame.draw.circle(screen, (0, 0, 255), (int(swarmalator.x * SCALE), int(swarmalator.y * SCALE)), int(swarmalator.radius * 20))

            pygame.display.flip()
            clock.tick(FRAME_RATE)
            frame_count+=1

        pygame.quit()
        with open("m_shape", "w") as file:
            print(self.sim_record)
            json.dump(self.sim_record, file, indent=4)

if __name__ == "__main__":
    sim = Simulation()
    sim.run()