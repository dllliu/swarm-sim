import numpy as np
import pygame

maxX = 800 #screen width and height
maxY  = 800
SCALE = int(maxX / 33)

class Swarmalator(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.radius = 0.2
        self.x = x
        self.y = y
        self.internal_freq = np.pi
        self.dx = 0
        self.dy = 0
        self.dx_static = 0
        self.dy_static = 0
        self.v_x = 0
        self.v_y= 0
        self.num_bots_in_thres = 0
        self.num_static_in_thres = 0
        self.image = pygame.Surface((self.radius * SCALE * 2, self.radius * SCALE  * 2), pygame.SRCALPHA)
        pygame.draw.circle(self.image, (0, 0, 255), (self.radius * SCALE , self.radius * SCALE ), self.radius * SCALE )
        self.rect = self.image.get_rect(center=(x * SCALE,y * SCALE))
        self.mask = pygame.mask.from_surface(self.image)

    def update(self, screen_width, screen_height):
        self.rect.x = self.x * SCALE
        self.rect.y = self.y* SCALE
