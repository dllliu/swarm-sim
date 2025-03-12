import pygame
import random

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Bouncing Circles with Movable Obstacles")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Circle properties
NUM_CIRCLES = 5
CIRCLE_RADIUS = 10

# Generate circles with random positions and velocities
circles = [
    {
        "pos": pygame.Vector2(random.randint(CIRCLE_RADIUS, WIDTH - CIRCLE_RADIUS),
                              random.randint(CIRCLE_RADIUS, HEIGHT - CIRCLE_RADIUS)),
        "vel": pygame.Vector2(random.choice([-4, 4]), random.choice([-4, 4]))
    }
    for _ in range(NUM_CIRCLES)
]

# Obstacle properties (now movable)
obstacles = [
    {
        "rect": pygame.Rect(300, 200, 200, 20),
        "vel": pygame.Vector2(0, 0)  # Initial velocity is zero
    },
    {
        "rect": pygame.Rect(150, 400, 250, 20),
        "vel": pygame.Vector2(0, 0)
    },
    {
        "rect": pygame.Rect(500, 300, 150, 20),
        "vel": pygame.Vector2(0, 0)
    }
]

# Clock for frame rate control
clock = pygame.time.Clock()
FPS = 60

def check_circle_collision(circle1, circle2):
    """Check if two circles are colliding and update their velocities if they do."""
    distance = circle1["pos"].distance_to(circle2["pos"])
    if distance < CIRCLE_RADIUS * 2:  # Collision detected
        normal = (circle2["pos"] - circle1["pos"]).normalize()
        tangent = pygame.Vector2(-normal.y, normal.x)

        v1n = normal.dot(circle1["vel"])
        v1t = tangent.dot(circle1["vel"])
        v2n = normal.dot(circle2["vel"])
        v2t = tangent.dot(circle2["vel"])

        # Swap normal components
        v1n, v2n = v2n, v1n

        circle1["vel"] = normal * v1n + tangent * v1t
        circle2["vel"] = normal * v2n + tangent * v2t


# Main loop
running = True
while running:
    screen.fill(WHITE)

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update each circle
    for i, circle in enumerate(circles):
        circle["pos"] += circle["vel"]

        # Wall collision
        if circle["pos"].x - CIRCLE_RADIUS <= 0 or circle["pos"].x + CIRCLE_RADIUS >= WIDTH:
            circle["vel"].x *= -1
        if circle["pos"].y - CIRCLE_RADIUS <= 0 or circle["pos"].y + CIRCLE_RADIUS >= HEIGHT:
            circle["vel"].y *= -1

        circle_rect = pygame.Rect(int(circle["pos"].x - CIRCLE_RADIUS), int(circle["pos"].y - CIRCLE_RADIUS),
                                  CIRCLE_RADIUS * 2, CIRCLE_RADIUS * 2)

        # Obstacle collision detection
        for obstacle in obstacles:
            if circle_rect.colliderect(obstacle["rect"]):
                # Apply a small force to the obstacle
                obstacle["vel"] += circle["vel"] * 0.2  # Transfer some velocity

                # Reverse circle direction to simulate bounce
                if abs(circle_rect.bottom - obstacle["rect"].top) < 10 and circle["vel"].y > 0:
                    circle["vel"].y *= -1
                elif abs(circle_rect.top - obstacle["rect"].bottom) < 10 and circle["vel"].y < 0:
                    circle["vel"].y *= -1
                elif abs(circle_rect.right - obstacle["rect"].left) < 10 and circle["vel"].x > 0:
                    circle["vel"].x *= -1
                elif abs(circle_rect.left - obstacle["rect"].right) < 10 and circle["vel"].x < 0:
                    circle["vel"].x *= -1

        # Check collision with other circles
        for j in range(i + 1, NUM_CIRCLES):
            check_circle_collision(circle, circles[j])

        pygame.draw.circle(screen, RED, (int(circle["pos"].x), int(circle["pos"].y)), CIRCLE_RADIUS)

    # Update and draw obstacles
    for obstacle in obstacles:
        obstacle["rect"].x += int(obstacle["vel"].x)
        obstacle["rect"].y += int(obstacle["vel"].y)

        # Slow down obstacles over time (friction effect)
        obstacle["vel"] *= 0.9

        # Ensure obstacles stay within bounds
        if obstacle["rect"].left < 0:
            obstacle["rect"].left = 0
            obstacle["vel"].x = 0
        if obstacle["rect"].right > WIDTH:
            obstacle["rect"].right = WIDTH
            obstacle["vel"].x = 0
        if obstacle["rect"].top < 0:
            obstacle["rect"].top = 0
            obstacle["vel"].y = 0
        if obstacle["rect"].bottom > HEIGHT:
            obstacle["rect"].bottom = HEIGHT
            obstacle["vel"].y = 0

        pygame.draw.rect(screen, BLUE, obstacle["rect"])

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
