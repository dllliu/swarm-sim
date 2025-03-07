import pygame

class Obstacle_Manager(pygame.sprite.Sprite):
    def __init__(self, shape, x, y, size=20, color=(255, 0, 0)):
        super().__init__()
        self.x = x
        self.y = y
        self.shape = shape
        self.color = color
        self.size = size
        self.image = pygame.Surface((size, size))
        self.rect = self.image.get_rect(center=(self.x, self.y))