import numpy as np
import pygame
import random
import time
from math import sqrt

from beacons import Beacon
from Swarmalator import Swarmalator
from boundary_manager import BoundaryPoint
from obstacle import Obstacle

maxX = 800 #screen width and height
maxY  = 800
SCALE = int(maxX / 33)

NUM_SWARMALATORS = 90
B = 1
dt = 0.3

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

class Simulation:
    def __init__(self):
        self.arr_beacons = []
        self.arr_swarmalators = pygame.sprite.Group()
        self.obstacles = []
        self.new_beacon_j=100
        self.thres_dist = 3 #beacon threshold distance from a swarm agent to activate, set_grid_beacons uses this
        self.set_grid_beacons()
        self.set_swarmalators()
        self.set_time = time.time()
        self.boundary_speed = 0.3#0.075
        self.boundary_direction = (np.pi) / 2
        self.boundary_control_points = [BoundaryPoint(33//2-6, 33//2, self.boundary_speed, self.boundary_direction, 0, 33, 0, 33)]
        self.radius = 1
        #self.init_obstacles()

    def set_grid_beacons(self):
        for i in range(0, 33):
            for j in range(0, 33):
                if 33//2-7 <= i <= 33//2-5 and 33//2-2 <= j <= 33//2+2:
                    beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                elif 33//2+5 <= i <= 33//2+7 and 33//2-2 <= j <= 33//2+2:
                    beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                else:
                    beac = Beacon(i, j, 0, self.thres_dist)
                self.arr_beacons.append(beac)

    def set_swarmalators(self):
        while len(self.arr_swarmalators) < NUM_SWARMALATORS:
            i = random.uniform(12, 15)
            j = random.uniform(12, 15)
            #if all(sqrt((s.x - i) ** 2 + (s.y - j) ** 2) >= 2 * s.radius for s in self.arr_swarmalators):
            swarmalator = Swarmalator(i, j)
            self.arr_swarmalators.add(swarmalator)

    def init_obstacles(self):
        #left, top, width, height
        self.obstacles = [
            #pygame.Rect(350, 250, 100, 30),
            Obstacle(200, 250, 150, 30),
            Obstacle(450, 250, 175, 30),

        ]


    def total_movement_and_phase_calcs(self):
        swarmalators_positions = np.array([[s.x, s.y] for s in self.arr_swarmalators])
        beacon_positions = np.array([[b.x, b.y] for b in self.arr_beacons])

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


    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((maxX, maxY))
        clock = pygame.time.Clock()
        running = True

        #PHYSICS_UPDATES_PER_SECOND = 30
        FRAME_RATE = 60
        # physics_dt = 1 / PHYSICS_UPDATES_PER_SECOND
        frame_count = 0
        frame_to_num_outside = []
        time_for_plot = 0

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill((255, 255, 255))

            if frame_count % 2 == 0:
                self.total_movement_and_phase_calcs()
                if time.time() - self.set_time > 10:
                    time_for_plot +=1
                    for boundary_point in self.boundary_control_points:
                        pygame.draw.circle(screen, RED, (int(boundary_point.center_x* SCALE), int(boundary_point.center_y * SCALE)), int(0.1 * 50))
                        boundary_point.move_boundary(dt)
                        count_outside = 0
                        for swarmalator in self.arr_swarmalators:
                            if ((boundary_point.center_x - swarmalator.x)**2 + (boundary_point.center_y - swarmalator.y)**2)**0.5 > self.thres_dist:
                                count_outside+=1
                        frame_to_num_outside.append((time_for_plot, count_outside))

                        #boundary_point.move_in_a_circle(dt, self.radius*1.5)
                        #boundary_point.move_in_a_cycloid(dt, self.radius/2)

            # for boundary_point in self.boundary_control_points:
            #     pygame.draw.circle(screen, RED, (int(boundary_point.center_x* SCALE), int(boundary_point.center_y * SCALE)), int(0.1 * 50))

            for beacon in self.arr_beacons:
                if beacon.is_near(self.boundary_control_points, self.new_beacon_j):
                    pygame.draw.circle(screen, (0, 255, 0), (int(beacon.x * SCALE), int(beacon.y * SCALE)), 5)

            for swarmalator in self.arr_swarmalators:
                pygame.draw.circle(screen, (0, 0, 255), (int(swarmalator.x * SCALE), int(swarmalator.y * SCALE)), int(swarmalator.radius * 20))

            for obstacle in self.obstacles:
                pygame.draw.rect(screen, RED, obstacle)

            pygame.display.flip()
            clock.tick(FRAME_RATE)
            frame_count+=1

        print(frame_to_num_outside)
        pygame.quit()

if __name__ == "__main__":
    sim = Simulation()
    sim.run()