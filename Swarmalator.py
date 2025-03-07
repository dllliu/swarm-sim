import numpy as np
import pygame

class Swarmalator(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.x = x
        self.y = y
        self.internal_freq = np.pi
        self.radius = 0.2
        self.dx = 0
        self.dy = 0
        self.dx_static = 0
        self.dy_static = 0
        self.num_bots_in_thres = 0
        self.num_static_in_thres = 0

        self.image = pygame.Surface((20, 20))
        self.image.fill((255, 0, 0))
        self.rect = self.image.get_rect(center=(self.x, self.y))

    def update(self):
        self.rect = self.image.get_rect(center=(self.x, self.y))

