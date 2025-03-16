import numpy as np
import pygame

class Swarmalator(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.radius = 0.2
        self.x = x
        self.y = y
        self.internal_freq = np.pi
        self.radius = 0.2
        self.dx = 0
        self.dy = 0
        self.dx_static = 0
        self.dy_static = 0
        self.v_x = 0
        self.v_y= 0
        self.num_bots_in_thres = 0
        self.num_static_in_thres = 0