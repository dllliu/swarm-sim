import numpy as np
import pygame
import random
import time
from math import sqrt

from beacons import Beacon
from Swarmalator import Swarmalator
from boundary_manager import BoundaryManager

maxX = 800
maxY  = 800
SCALE = int(maxX / 33)

NUM_SWARMALATORS = 30
B = 0.4
dt = 1

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

class Simulation:
    def __init__(self):
        self.arr_beacons = []
        self.arr_swarmalators = pygame.sprite.Group()
        self.obstacles = []
        self.new_beacon_j=32
        self.set_grid_beacons()
        self.set_swarmalators()
        self.set_time = time.time()
        self.boundary_speed = 0.05
        self.boundary_direction = (np.pi) / 2
        self.curr_boundary = BoundaryManager(33//2, 33//2, self.boundary_speed, self.boundary_direction, 0, 33, 0, 33, thres_dist=3)
        self.init_obstacles()

    def set_grid_beacons(self):
        for i in range(0, 33):
            for j in range(0, 33):
                if 33//2-2 <= j <= 33//2+2 and 33//2-2 <= i <= 33//2+2:
                    beac = Beacon(i, j, beacon_j=self.new_beacon_j)
                else:
                    beac = Beacon(i, j, beacon_j=0)
                self.arr_beacons.append(beac)

    def set_swarmalators(self):
        while len(self.arr_swarmalators) < NUM_SWARMALATORS:
            i = random.randint(0, 33)
            j = random.randint(0, 33)
            if all(sqrt((s.x - i) ** 2 + (s.y - j) ** 2) >= 2 * s.radius for s in self.arr_swarmalators):
                swarmalator = Swarmalator(i, j)
                self.arr_swarmalators.add(swarmalator)

    def init_obstacles(self):
        #left, top, width, height
        self.obstacles = [
            pygame.Rect(350, 200, 100, 30),
            pygame.Rect(350, 50, 100, 30),
            pygame.Rect(325, 475, 100, 30),
            pygame.Rect(325, 650, 100, 30)
        ]


    def handle_collisions(self, swarmalator):
        swarmalator_rect = pygame.Rect(int(swarmalator.x * SCALE - swarmalator.radius * SCALE), int(swarmalator.y * SCALE - swarmalator.radius * SCALE), swarmalator.radius * SCALE, swarmalator.radius * SCALE)

        offset = abs(swarmalator.v_y)

        for obstacle in self.obstacles:
            if swarmalator_rect.colliderect(obstacle):
                if abs(swarmalator_rect.bottom - obstacle.top) < 10 and swarmalator.v_y > 0:  # Hitting from top
                    swarmalator.v_y *= -1
                    swarmalator.y -= 2 * offset
                    if swarmalator_rect.centerx < obstacle.centerx:
                        swarmalator.x -= 3 * offset
                    else:
                        swarmalator.x += 3 * offset
                elif abs(swarmalator_rect.top - obstacle.bottom) < 10 and swarmalator.v_y < 0:  # Hitting from bottom
                    swarmalator.v_y *= -1
                    swarmalator.y += 2 * offset
                    if swarmalator_rect.centerx < obstacle.centerx:
                        swarmalator.x -= 3 * offset
                    else:
                        swarmalator.x += 3 * offset
                elif abs(swarmalator_rect.right - obstacle.left) < 10 and swarmalator.v_x > 0:  # Hitting from left
                    swarmalator.v_x *= -1
                    swarmalator.x -= 2 * offset
                    if swarmalator_rect.centery < obstacle.centery:
                        swarmalator.y -= 3 * offset
                    else:
                        swarmalator.y += 3 * offset
                elif abs(swarmalator_rect.left - obstacle.right) < 10 and swarmalator.v_x < 0:  # Hitting from right
                    swarmalator.v_x *= -1
                    swarmalator.x += 2 * offset
                    if swarmalator_rect.centery < obstacle.centery:
                        swarmalator.y -= 3 * offset
                    else:
                        swarmalator.y += 3 * offset


    def total_movement_and_phase_calcs(self):
        for curr_swarmalator in self.arr_swarmalators:
            curr_swarmalator.dx = 0
            curr_swarmalator.dy = 0
            curr_swarmalator.dx_static = 0
            curr_swarmalator.dy_static = 0
            curr_swarmalator.num_bots_in_thres = 0
            curr_swarmalator.num_static_in_thres = 0
            curr_swarmalator.v_x=0
            curr_swarmalator.v_y=0

            for swarmalator in self.arr_swarmalators:
                if curr_swarmalator != swarmalator:
                    distBetweenBots = sqrt((curr_swarmalator.x - swarmalator.x) ** 2 + (curr_swarmalator.y - swarmalator.y) ** 2)
                    dist_x = swarmalator.x - curr_swarmalator.x
                    dist_y = swarmalator.y - curr_swarmalator.y
                    if distBetweenBots < 2 * curr_swarmalator.radius:
                        curr_swarmalator.dx -= 2.3 * (dist_x / (distBetweenBots * 0.2 + 0.1))
                        curr_swarmalator.dy -= 2.3 * (dist_y / (distBetweenBots * 0.2 + 0.1))
                    else:
                        curr_swarmalator.dx -= B * (dist_x / (distBetweenBots ** 2))
                        curr_swarmalator.dy -= B * (dist_y / (distBetweenBots ** 2))
                    curr_swarmalator.num_bots_in_thres += 1

            for beacon in self.arr_beacons:
                distBetweenBotAndStatic = sqrt((beacon.x - curr_swarmalator.x) ** 2 + (beacon.y - curr_swarmalator.y) ** 2)
                dist_x = beacon.x - curr_swarmalator.x
                dist_y = beacon.y - curr_swarmalator.y
                curr_phase = curr_swarmalator.internal_freq
                curr_swarmalator.dx_static += (dist_x / (distBetweenBotAndStatic ** 2 + 0.1)) * beacon.beacon_j * np.cos(beacon.internal_freq - curr_phase)
                curr_swarmalator.dy_static += (dist_y / (distBetweenBotAndStatic ** 2 + 0.1)) * beacon.beacon_j * np.cos(beacon.internal_freq - curr_phase)
                curr_swarmalator.num_static_in_thres += 1

            if curr_swarmalator.num_bots_in_thres == 0:
                curr_swarmalator.num_bots_in_thres = 1
            if curr_swarmalator.num_static_in_thres == 0:
                curr_swarmalator.num_static_in_thres = 1

            curr_swarmalator.v_x = (curr_swarmalator.dx / curr_swarmalator.num_bots_in_thres) * dt + (curr_swarmalator.dx_static / curr_swarmalator.num_static_in_thres) * dt
            curr_swarmalator.v_y = (curr_swarmalator.dy / curr_swarmalator.num_bots_in_thres) * dt + (curr_swarmalator.dy_static / curr_swarmalator.num_static_in_thres) * dt
            self.handle_collisions(curr_swarmalator)
            curr_swarmalator.x += curr_swarmalator.v_x
            curr_swarmalator.y += curr_swarmalator.v_y

    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((maxX, maxY))
        clock = pygame.time.Clock()
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill((255, 255, 255))
            self.total_movement_and_phase_calcs()

            for beacon in self.arr_beacons:
                if beacon.active:
                    pygame.draw.circle(screen, (0, 255, 0), (int(beacon.x * SCALE), int(beacon.y * SCALE)), 5)

            for swarmalator in self.arr_swarmalators:
                pygame.draw.circle(screen, (0, 0, 255), (int(swarmalator.x * SCALE), int(swarmalator.y * SCALE)), int(swarmalator.radius * 20))

            for obstacle in self.obstacles:
                pygame.draw.rect(screen, RED, obstacle)

            if time.time() - self.set_time > 20:
                self.curr_boundary.move_boundary(dt)
                self.curr_boundary.update_beacons(self.arr_beacons, self.new_beacon_j)

            pygame.display.flip()
            clock.tick(30)

        pygame.quit()

if __name__ == "__main__":
    sim = Simulation()
    sim.run()