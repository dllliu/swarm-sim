import pygame
import math

class Obstacle(pygame.Rect):
    def __init__(self, x, y, width, height):
        super().__init__(x, y, width, height)

        self.angle = 0
        self.angular_velocity = 0

    # def update(self):
    #     self.topleft = (new_x, new_y)
