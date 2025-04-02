import pygame

class RotatableSprite(pygame.sprite.Sprite):
    def __init__(self, color, size, position, angle=0):
        super().__init__()
        self.color = color
        self.size = size
        self.angle = angle

        # Create a transparent surface with a rectangle
        self.original_image = pygame.Surface(size, pygame.SRCALPHA)
        pygame.draw.rect(self.original_image, color, (0, 0, *size))

        self.image = self.original_image
        self.rect = self.image.get_rect(center=position)
        self.mask = pygame.mask.from_surface(self.image)

    def rotate(self, angle_change):
        self.angle += angle_change
        self.image = pygame.transform.rotate(self.original_image, self.angle)
        self.rect = self.image.get_rect(center=self.rect.center)
        self.mask = pygame.mask.from_surface(self.image)

    def update(self):
        self.mask = pygame.mask.from_surface(self.image)


class BouncingDot(pygame.sprite.Sprite):
    def __init__(self, color, radius, position, speed):
        super().__init__()
        self.color = color
        self.radius = radius
        self.image = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(self.image, color, (radius, radius), radius)
        self.rect = self.image.get_rect(center=position)
        self.speed_x, self.speed_y = speed
        self.mask = pygame.mask.from_surface(self.image)

    def update(self, screen_width, screen_height):
        # Move the dot
        self.rect.x += self.speed_x
        self.rect.y += self.speed_y

        # Bounce off the walls
        if self.rect.left <= 0 or self.rect.right >= screen_width:
            self.speed_x = -self.speed_x
        if self.rect.top <= 0 or self.rect.bottom >= screen_height:
            self.speed_y = -self.speed_y


# Initialize pygame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Create a rotating rectangle
rectangle = RotatableSprite(color=(0, 0, 255), size=(200, 50), position=(400, 300))

# Create a bouncing dot
dot = BouncingDot(color=(255, 0, 0), radius=10, position=(200, 200), speed=(5, 5))

# Sprite group
all_sprites = pygame.sprite.Group(rectangle, dot)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                rectangle.rotate(5)  # Rotate 5 degrees left
            if event.key == pygame.K_RIGHT:
                rectangle.rotate(-5)  # Rotate 5 degrees right

    # Update bouncing dot
    dot.update(WIDTH, HEIGHT)

    # Collision detection using masks
    if pygame.sprite.collide_mask(rectangle, dot):
        dot.speed_x = -dot.speed_x
        dot.speed_y = -dot.speed_y  # Reverse direction on collision

    # Drawing
    screen.fill((255, 255, 255))  # Clear screen
    all_sprites.draw(screen)  # Draw sprites

    pygame.display.flip()  # Update display
    clock.tick(60)  # Maintain 60 FPS

pygame.quit()
