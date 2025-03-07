import numpy as np
import pygame
import random
import time
from math import sqrt

from beacons import Beacon
from Swarmalator import Swarmalator
from boundary_manager import BoundaryManager
from obstacle import Obstacle_Manager

N_GRID = 15
M_GRID = 15
maxX = N_GRID // 2
minX = - (N_GRID // 2)
maxY = M_GRID // 2
minY = -(M_GRID // 2)

NUM_SWARMALATORS = 30
B = 0.4
dt = 0.2

class Simulation:
    def __init__(self):
        self.arr_beacons = []
        self.arr_swarmalators = pygame.sprite.Group()
        self.obstacles = pygame.sprite.Group()
        self.new_beacon_j=20
        self.set_grid_beacons()
        self.set_swarmalators()
        self.set_time = time.time()
        self.median_x = np.mean([swarmalator.x for swarmalator in self.arr_swarmalators])
        self.median_y = np.mean([swarmalator.y for swarmalator in self.arr_swarmalators])
        self.boundary_speed = 0.05
        self.boundary_direction = (np.pi) / 2
        self.curr_boundary = BoundaryManager(self.median_x, self.median_y, self.boundary_speed, self.boundary_direction, minX, maxX, minY, maxY, thres_dist=1.5)
        self.init_obstacles()

    def set_grid_beacons(self):
        for i in range(minX, maxX + 1):
            for j in range(minY, maxY + 1):
                if -2 <= j <= 0 and -2 <= i <= 0:
                    beac = Beacon(i, j, beacon_j=self.new_beacon_j)
                else:
                    beac = Beacon(i, j, beacon_j=0)
                self.arr_beacons.append(beac)

    def set_swarmalators(self):
        for _ in range(NUM_SWARMALATORS):
            while True:
                x = minX + random.random() * (maxX - minX)
                y = minY + random.random() * (maxY - minY)
                swarmalator = Swarmalator(x, y)
                if all(sqrt((s.x - x) ** 2 + (s.y - y) ** 2) >= 2 * s.radius for s in self.arr_swarmalators):
                    self.arr_swarmalators.add(swarmalator)
                    break

    def init_obstacles(self):
        self.obstacles.add(Obstacle_Manager("rect", -0.75, 5, size=60, color=(255, 0, 0)))
        self.obstacles.add(Obstacle_Manager("rect", -0.75, 3, size=60, color=(255, 0, 0)))
        self.obstacles.add(Obstacle_Manager("rect", -0.75, 1, size=60, color=(255, 0, 0)))


    def handle_collisions(self, swarmalator):
        swarmalator.update()

        for obstacle in self.obstacles:
            dist_x = swarmalator.rect.centerx - obstacle.rect.centerx
            dist_y = swarmalator.rect.centery - obstacle.rect.centery
            distance = sqrt(dist_x ** 2 + dist_y ** 2)
            threshold = obstacle.size / 40

            if obstacle.shape == "rect" and pygame.sprite.collide_rect(swarmalator, obstacle) and distance <= max(1, threshold):
                repulsion_strength = 1.5 * threshold
                swarmalator.dx = repulsion_strength * (dist_x / (distance * 0.2 + 0.1))
                swarmalator.dy = repulsion_strength * (dist_y / (distance * 0.2 + 0.1))


    def total_movement_and_phase_calcs(self):
        for curr_swarmalator in self.arr_swarmalators:
            curr_swarmalator.dx = 0
            curr_swarmalator.dy = 0
            curr_swarmalator.dx_static = 0
            curr_swarmalator.dy_static = 0
            curr_swarmalator.num_bots_in_thres = 0
            curr_swarmalator.num_static_in_thres = 0

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

            self.handle_collisions(curr_swarmalator)

            curr_swarmalator.x += (curr_swarmalator.dx / curr_swarmalator.num_bots_in_thres) * dt + (curr_swarmalator.dx_static / curr_swarmalator.num_static_in_thres) * dt
            curr_swarmalator.y += (curr_swarmalator.dy / curr_swarmalator.num_bots_in_thres) * dt + (curr_swarmalator.dy_static / curr_swarmalator.num_static_in_thres) * dt

    def transform_coordinates(self, x, y, screen_width=800, screen_height=800):
        px = ((x - minX) / (maxX - minX)) * screen_width
        py = screen_height - ((y - minY) / (maxY - minY)) * screen_height
        return px, py

    def run(self):
        self.init_obstacles()
        pygame.init()
        screen = pygame.display.set_mode((800, 800))
        clock = pygame.time.Clock()
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill((255, 255, 255))
            self.total_movement_and_phase_calcs()

            if time.time() - self.set_time > 10:
                self.curr_boundary.move_boundary(dt)
                self.curr_boundary.update_beacons(self.arr_beacons, self.new_beacon_j)
                for beacon in self.arr_beacons:
                    if beacon.active:
                        beacon_x, beacon_y = self.transform_coordinates(beacon.x, beacon.y)
                        pygame.draw.circle(screen, (0, 255, 0), (beacon_x, beacon_y), 5)

            for swarmalator in self.arr_swarmalators:
                swarmalator_x, swarmalator_y = self.transform_coordinates(swarmalator.x, swarmalator.y)
                pygame.draw.circle(screen, (0, 0, 255), (swarmalator_x, swarmalator_y), swarmalator.radius * 20)

            for obstacle in self.obstacles:
                obstacle_x, obstacle_y = self.transform_coordinates(obstacle.x, obstacle.y)
                if obstacle.shape == "rect":
                    pygame.draw.rect(screen, obstacle.color, (obstacle_x, obstacle_y, obstacle.size, obstacle.size))
                elif obstacle.shape == "circle":
                    pygame.draw.circle(screen, obstacle.color, (obstacle_x, obstacle_y), obstacle.size)

            pygame.display.flip()
            clock.tick(30)

        pygame.quit()

if __name__ == "__main__":
    sim = Simulation()
    sim.run()