import pygame
import random
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Multiple Bouncing Circles with Collision")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Circle properties
NUM_CIRCLES = 5  # Number of bouncing circles
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

# Obstacle properties
obstacles = [
    pygame.Rect(300, 200, 200, 20),
    pygame.Rect(150, 400, 250, 20),
    pygame.Rect(500, 300, 150, 20)
]

# Clock for frame rate control
clock = pygame.time.Clock()
FPS = 60


def check_circle_collision(circle1, circle2):
    """Check if two circles are colliding and update their velocities if they do."""
    distance = circle1["pos"].distance_to(circle2["pos"])
    if distance < CIRCLE_RADIUS * 2:  # Collision detected
        # Calculate unit normal and unit tangent vectors
        normal = (circle2["pos"] - circle1["pos"]).normalize()
        tangent = pygame.Vector2(-normal.y, normal.x)

        # Project velocities onto normal and tangent directions
        v1n = normal.dot(circle1["vel"])
        v1t = tangent.dot(circle1["vel"])
        v2n = normal.dot(circle2["vel"])
        v2t = tangent.dot(circle2["vel"])

        # Swap normal components (simulate elastic collision)
        v1n, v2n = v2n, v1n

        # Convert scalar normal and tangential velocities back to vectors
        circle1["vel"] = normal * v1n + tangent * v1t
        circle2["vel"] = normal * v2n + tangent * v2t


# Main loop
running = True
while running:
    screen.fill(WHITE)  # Clear screen

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update each circle
    for i, circle in enumerate(circles):
        circle["pos"] += circle["vel"]

        # Wall collision detection (bounce off walls)
        if circle["pos"].x - CIRCLE_RADIUS <= 0 or circle["pos"].x + CIRCLE_RADIUS >= WIDTH:
            circle["vel"].x *= -1  # Reverse X direction
        if circle["pos"].y - CIRCLE_RADIUS <= 0 or circle["pos"].y + CIRCLE_RADIUS >= HEIGHT:
            circle["vel"].y *= -1  # Reverse Y direction

        # Convert circle position to a pygame.Rect for collision detection
        circle_rect = pygame.Rect(int(circle["pos"].x - CIRCLE_RADIUS), int(circle["pos"].y - CIRCLE_RADIUS),
                                  CIRCLE_RADIUS * 2, CIRCLE_RADIUS * 2)

        # Obstacle collision detection
        for obstacle in obstacles:
            if circle_rect.colliderect(obstacle):
                # Determine collision side by checking overlap
                if abs(circle_rect.bottom - obstacle.top) < 10 and circle["vel"].y > 0:  # Hitting from top
                    circle["vel"].y *= -1
                elif abs(circle_rect.top - obstacle.bottom) < 10 and circle["vel"].y < 0:  # Hitting from bottom
                    circle["vel"].y *= -1
                elif abs(circle_rect.right - obstacle.left) < 10 and circle["vel"].x > 0:  # Hitting from left
                    circle["vel"].x *= -1
                elif abs(circle_rect.left - obstacle.right) < 10 and circle["vel"].x < 0:  # Hitting from right
                    circle["vel"].x *= -1

        # Check collision with other circles
        for j in range(i + 1, NUM_CIRCLES):
            check_circle_collision(circle, circles[j])

        # Draw bouncing circle
        pygame.draw.circle(screen, RED, (int(circle["pos"].x), int(circle["pos"].y)), CIRCLE_RADIUS)

    # Draw obstacles
    for obstacle in obstacles:
        pygame.draw.rect(screen, BLUE, obstacle)

    pygame.display.flip()  # Update display
    clock.tick(FPS)  # Maintain frame rate

pygame.quit()
