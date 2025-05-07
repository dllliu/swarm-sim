import numpy as np
import pygame
import random
import time

from beacons import Beacon
from Swarmalator import Swarmalator
from mask_creator import MaskCreator
from obstacle import Obstacle
import json
import string

maxX = 800 #screen width and height
maxY  = 800
SCALE = int(maxX / 33)

NUM_SWARMALATORS = 300
B = 0.4
dt = 2

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

class Simulation:
    def __init__(self):
        self.arr_beacons = []
        self.arr_swarmalators = pygame.sprite.Group()
        self.new_beacon_j=6 #6
        self.thres_dist = 2 #beacon threshold distance from a swarm agent to activate, set_grid_beacons uses this
        self.set_swarmalators()
        self.obstacles = pygame.sprite.Group()
        self.set_time = time.time()
        self.radius = 1 #for circular paths
        self.screen = pygame.display.set_mode((maxX, maxY))
        self.polygon_points = [(50, 50), (350, 50), (125, 100), (350, 150), (50, 350), (75, 100)]
        self.mask_creator = MaskCreator(maxX, maxY)
        self.sim_record = {}
        #self.init_obstacles()


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
        elif mask_arg == "letter":
            mask = self.mask_creator.create_mask_from_letter("a", 900)
        else:
            print("Available options are: polygon, donut, circle, ellipse, hollow_ellipse, lines")
            return None
        return mask


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
        while len(self.arr_swarmalators) < NUM_SWARMALATORS:
            i = random.uniform(0, 33)
            j = random.uniform(0, 33)
            #if all(sqrt((s.x - i) ** 2 + (s.y - j) ** 2) >= 2 * s.radius for s in self.arr_swarmalators):
            swarmalator = Swarmalator(i, j)
            self.arr_swarmalators.add(swarmalator)

    def init_obstacles(self):
        #left, top, width, height
        self.obstacles.add(Obstacle(color=RED, size=(80, 50), position=(400, 550)))

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
            curr_swarmalator.update(maxX, maxY)
            sim_rec_per_dt.append((curr_swarmalator.x, curr_swarmalator.y))

        self.sim_record[frame_count] = sim_rec_per_dt

    def run(self):
        pygame.init()
        # screen = pygame.display.set_mode((maxX, maxY))
        clock = pygame.time.Clock()
        running = True
        dt = 0

        FRAME_RATE = 60
        frame_count = 0

        letter_index = 19
        all_letters = list(string.ascii_uppercase)
        mask = self.mask_creator.create_mask_from_letter(all_letters[letter_index], font_size=900)
        self.set_grid_beacons(mask)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            if dt % 100 == 0 and dt != 0:
                if letter_index <= 24:
                    with open(f"letters/{all_letters[letter_index]}", "w") as file:
                        #print(self.sim_record)
                        json.dump(self.sim_record, file, indent=4)
                    self.sim_record.clear()
                    frame_count = 0
                    letter_index += 1
                    # RESET SWARMALATORS AND SIM STATE
                    self.arr_swarmalators.empty()
                    self.set_swarmalators()
                else:
                    with open(f"letters/{all_letters[letter_index]}", "w") as file: #last letter
                        #print(self.sim_record)
                        json.dump(self.sim_record, file, indent=4)
                    running = False

                mask = self.mask_creator.create_mask_from_letter(all_letters[letter_index], font_size=900)
                self.set_grid_beacons(mask)


            self.screen.fill((255, 255, 255))

            if frame_count % 20 == 0:
                self.total_movement_and_phase_calcs(frame_count)
                dt+=1

            # for boundary_point in self.boundary_control_points:
            #     pygame.draw.circle(screen, RED, (int(boundary_point.center_x* SCALE), int(boundary_point.center_y * SCALE)), int(0.1 * 50))


            for beacon in self.arr_beacons:
                if beacon.beacon_j >0:
                    pygame.draw.circle(self.screen, (0, 255, 0), (int(beacon.x * SCALE), int(beacon.y * SCALE)), 5)

            self.arr_swarmalators.draw(self.screen)
            #self.obstacles.draw(self.screen)

            pygame.display.flip()
            clock.tick(FRAME_RATE)
            frame_count+=1

        pygame.quit()

if __name__ == "__main__":
    sim = Simulation()
    sim.run()
