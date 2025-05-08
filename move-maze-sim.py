import numpy as np
import pygame
import random
import time
from math import sqrt

from beacons import Beacon
from Swarmalator import Swarmalator
from mask_creator import MaskCreator
from obstacle import Obstacle
import json

maxX = 800 #screen width and height
maxY  = 800
SCALE = int(maxX / 33)

NUM_SWARMALATORS = 30
B = 0.4
dt = 0.5

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

class Simulation:
    def __init__(self):
        self.arr_beacons = []
        self.arr_swarmalators = pygame.sprite.Group()
        self.new_beacon_j= 100 #10 #6
        self.thres_dist = 2 #beacon threshold distance from a swarm agent to activate, set_grid_beacons uses this
        self.set_swarmalators()
        self.obstacles = pygame.sprite.Group()
        self.set_time = time.time()
        self.radius = 1 #for circular paths
        self.screen = pygame.display.set_mode((maxX, maxY))
        self.mask_creator = MaskCreator(maxX, maxY)
        self.sim_record = {}
        self.start_time = time.time()
        self.init_obstacles()

    def select_mask(self, mask_arg):
        if mask_arg == "polygon":
            mask = self.mask_creator.create_mask_from_polygon(self.polygon_points)
        elif mask_arg == "donut":
            mask = self.mask_creator.create_mask_from_donut()

        elif mask_arg == "circle":
            mask = self.mask_creator.create_mask_from_circle()

        elif mask_arg == "ellipse":
            mask = self.mask_creator.create_mask_from_ellipse([225, 300, 300, 400])

        elif mask_arg == "hollow_ellipse":
            mask = self.mask_creator.create_mask_from_hollow_ellipse([225, 300, 300, 400], 20)

        elif mask_arg == "lines":
            mask = self.mask_creator.create_mask_from_lines([(100, 500), (200, 200), (500, 200), (600, 500)])
        else:
            print("Available options are: polygon, donut, circle, ellipse, hollow_ellipse, lines")
            return None
        return mask

    def set_grid_beacons(self, mask_arr, lookup_mask_type):
        self.arr_beacons.clear()

        for i in range(0, 33):
            for j in range(0, 33):
                sett = False
                for mask in mask_arr:
                    if mask.get_at((i*SCALE, j*SCALE)):
                        if lookup_mask_type[mask] == "attr":
                            beac = Beacon(i, j, self.new_beacon_j, self.thres_dist)
                        else: #repl
                            beac = Beacon(i, j, -self.new_beacon_j, self.thres_dist)
                        sett = True
                if not sett:
                    beac = Beacon(i, j, 0, self.thres_dist)
                self.arr_beacons.append(beac)

    def set_swarmalators(self):
        while len(self.arr_swarmalators) < NUM_SWARMALATORS:
            i = random.uniform(5, 8)
            j = random.uniform(14, 17)
            if all(sqrt((s.x - i) ** 2 + (s.y - j) ** 2) >= 2 * s.radius for s in self.arr_swarmalators):
                swarmalator = Swarmalator(i, j)
                self.arr_swarmalators.add(swarmalator)

    def init_obstacles(self):
        #left, top, width, height
        color = RED
        ob_type = "maze"

        self.obstacles.add(Obstacle(color=color, size=(20, 300), position=(100, 600), angle=0, ob_type=ob_type))
        self.obstacles.add(Obstacle(color=color, size=(20, 200), position=(250, 570), angle=0, ob_type=ob_type))
        self.obstacles.add(Obstacle(color=color, size=(20, 330), position=(405, 670), angle=90, ob_type=ob_type))
        self.obstacles.add(Obstacle(color=color, size=(20, 500), position=(340, 760), angle=90, ob_type=ob_type))
        self.obstacles.add(Obstacle(color=GREEN, size=(30, 30), position=(210, 600), angle=90, ob_type="obstacle"))

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

            self.handle_collisions(curr_swarmalator)
            curr_swarmalator.x += curr_swarmalator.v_x
            curr_swarmalator.y += curr_swarmalator.v_y
            curr_swarmalator.update(maxX, maxY)
            sim_rec_per_dt.append((curr_swarmalator.x, curr_swarmalator.y))

        self.sim_record[frame_count] = sim_rec_per_dt

    def handle_collisions(self, swarmalator):
        """Handle collisions between a swarmalator and obstacles realistically"""
        for obstacle in self.obstacles:
            if pygame.sprite.collide_mask(obstacle, swarmalator):
                print(obstacle.ob_type)
                # Direction vector from obstacle to swarmalator
                dx = swarmalator.rect.centerx - obstacle.rect.centerx
                dy = swarmalator.rect.centery - obstacle.rect.centery

                half_w = obstacle.rect.width / 2
                half_h = obstacle.rect.height / 2

                # Penetration depth along each axis
                overlap_x = half_w - abs(dx)
                overlap_y = half_h - abs(dy)

                if overlap_y < overlap_x: # collision along y
                    correction = overlap_y
                    if dy > 0:
                        #print("below")
                        swarmalator.y += swarmalator.v_y
                        if obstacle.ob_type != "maze":
                            obstacle.rect.y += correction
                    else:
                        #print("above")
                        swarmalator.y -= swarmalator.v_y
                        if obstacle.ob_type != "maze":
                            obstacle.rect.y -= correction
                else:
                    correction = overlap_x
                    if dx > 0: # from right
                        #print("right")
                        swarmalator.x += swarmalator.v_x
                        if obstacle.ob_type != "maze": # movable obstacle
                            obstacle.rect.x += correction
                    else:
                        #print("left")
                        swarmalator.x -= swarmalator.v_x
                        if obstacle.ob_type != "maze":
                            obstacle.rect.x -= correction
                # break

    def run(self):
        pygame.init()
        # screen = pygame.display.set_mode((maxX, maxY))
        clock = pygame.time.Clock()
        running = True

        FRAME_RATE = 60
        frame_count = 0

        center_x = 200
        center_y = 400
        start_time = time.time()
        move_right = False

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.screen.fill((255, 255, 255))

            if frame_count % 20 == 0:
                self.total_movement_and_phase_calcs(frame_count)

            mask = self.mask_creator.create_mask_from_circle(center=(center_x, center_y), radius=70)
            if center_y < 640:
                center_y+= dt* 0.1
            else:
                if center_x > 130 and not move_right:
                    center_y+= dt* 0.1
                    center_x -= dt* 0.1
                else:
                    move_right = True

            if move_right:
                mask = self.mask_creator.create_mask_from_circle(center=(center_x, center_y), radius=40)
                center_x += dt*0.1

            lookup_mask_type ={mask: "attr"}
            mask_arr = [mask]
            self.set_grid_beacons(mask_arr, lookup_mask_type)

            # NOTE: comment to not draw beacons
            for beacon in self.arr_beacons:
                if beacon.beacon_j > 0:
                    pygame.draw.circle(self.screen, GREEN, (int(beacon.x * SCALE), int(beacon.y * SCALE)), 5)
                elif beacon.beacon_j < 0:
                    pygame.draw.circle(self.screen, RED, (int(beacon.x * SCALE), int(beacon.y * SCALE)), 5)

            self.arr_swarmalators.draw(self.screen)
            self.obstacles.draw(self.screen)

            pygame.display.flip()
            clock.tick(FRAME_RATE)
            frame_count+=1

        pygame.quit()

if __name__ == "__main__":
    sim = Simulation()
    sim.run()
