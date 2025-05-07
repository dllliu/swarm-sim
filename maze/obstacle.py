import pygame

class Obstacle(pygame.sprite.Sprite):
    def __init__(self, color, size, position, ob_type = "move", angle=0):
        super().__init__()
        self.color = color
        self.size = size
        self.angle = angle
        self.ob_type = ob_type

        # Create a transparent surface with a rectangle
        self.original_image = pygame.Surface(size, pygame.SRCALPHA)
        pygame.draw.rect(self.original_image, color, (0, 0, *size))

        self.image = self.original_image
        self.rect = self.image.get_rect(center=position)
        self.mask = pygame.mask.from_surface(self.image)
        if self.angle != 0:
            self.image = pygame.transform.rotate(self.original_image, self.angle)
            self.rect = self.image.get_rect(center=self.rect.center)
            self.mask = pygame.mask.from_surface(self.image)


    def rotate(self, angle_change):
        self.angle += angle_change
        self.image = pygame.transform.rotate(self.original_image, self.angle)
        self.rect = self.image.get_rect(center=self.rect.center)
        self.mask = pygame.mask.from_surface(self.image)

    def update(self):
        self.mask = pygame.mask.from_surface(self.image)
