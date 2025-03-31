import pygame
import math
import random

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Bouncing Circles with Rotating Rod and Inertia")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

# Circle class
class Circle:
    def __init__(self, x, y, radius, color, velocity_x, velocity_y):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.velocity_x = velocity_x
        self.velocity_y = velocity_y

    def move(self):
        self.x += self.velocity_x
        self.y += self.velocity_y

        # Bounce off the walls
        if self.x - self.radius <= 0 or self.x + self.radius >= WIDTH:
            self.velocity_x *= -1
        if self.y - self.radius <= 0 or self.y + self.radius >= HEIGHT:
            self.velocity_y *= -1

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

# Rotating Rod class with Moment of Inertia
class RotatingRod:
    def __init__(self, center_x, center_y, length, thickness, mass):
        self.center_x = center_x
        self.center_y = center_y
        self.length = length
        self.thickness = thickness
        self.angle = 0  # Angle of rotation for the rod
        self.angular_velocity = 0  # Angular velocity of the rod
        self.mass = mass  # Mass of the rod
        self.inertia = (1 / 3) * mass * (length ** 2)  # Moment of inertia for a rod rotating around the center

    def apply_torque(self, torque):
        # Apply torque to the rod, affecting angular velocity
        # Torque = Inertia * Angular acceleration, so angular acceleration = Torque / Inertia
        angular_acceleration = torque / self.inertia
        self.angular_velocity += angular_acceleration  # Update angular velocity based on torque

    def rotate(self):
        # Update the angle based on angular velocity (simple physics)
        self.angle += self.angular_velocity
        if self.angle >= 360:
            self.angle -= 360
        elif self.angle < 0:
            self.angle += 360

    def draw(self, screen):
        # Calculate the rod's endpoints based on the angle
        x1 = self.center_x + math.cos(math.radians(self.angle)) * self.length / 2
        y1 = self.center_y + math.sin(math.radians(self.angle)) * self.length / 2
        x2 = self.center_x - math.cos(math.radians(self.angle)) * self.length / 2
        y2 = self.center_y - math.sin(math.radians(self.angle)) * self.length / 2

        # Draw the rod as a line
        pygame.draw.line(screen, BLACK, (x1, y1), (x2, y2), self.thickness)

# Create circles
circles = [Circle(random.randint(50, WIDTH-50), random.randint(50, HEIGHT-50), 20, RED, random.randint(-5, 5), random.randint(-5, 5)) for _ in range(5)]

# Create a rotating rod (with mass for inertia calculation)
rod = RotatingRod(WIDTH // 2, HEIGHT // 2, 150, 10, mass=10)

# Main game loop
running = True
while running:
    screen.fill(WHITE)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update and draw the bouncing circles
    for circle in circles:
        circle.move()
        circle.draw(screen)

        # Collision detection with the rod
        dx = circle.x - rod.center_x
        dy = circle.y - rod.center_y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance <= circle.radius + rod.thickness:
            # Apply a torque to the rod based on the circle's velocity and position
            # The force on the rod is proportional to the speed and the perpendicular distance from the center of the rod
            torque = (circle.velocity_x * dy - circle.velocity_y * dx) * circle.radius  # Torque = F * r
            rod.apply_torque(torque)

            # Make the circle bounce off the rod
            normal_angle = math.atan2(dy, dx)
            velocity_magnitude = math.sqrt(circle.velocity_x ** 2 + circle.velocity_y ** 2)
            angle_of_reflection = 2 * normal_angle - math.atan2(circle.velocity_y, circle.velocity_x)

            # Reflect the velocity based on the angle of collision
            circle.velocity_x = velocity_magnitude * math.cos(angle_of_reflection)
            circle.velocity_y = velocity_magnitude * math.sin(angle_of_reflection)

    # Update and rotate the rod based on the applied torque
    rod.rotate()
    rod.draw(screen)

    # Update the screen
    pygame.display.flip()

    # Set frame rate
    pygame.time.Clock().tick(60)

# Quit Pygame
pygame.quit()
