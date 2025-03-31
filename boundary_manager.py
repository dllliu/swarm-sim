import numpy as np

class BoundaryPoint:
    def __init__(self, center_x, center_y, speed, direction, minX, maxX, minY, maxY):
        self.center_x = center_x
        self.center_y = center_y
        self.speed = speed
        self.direction = direction
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY

        self.V_bx = speed * np.cos(direction)
        self.V_by = speed * np.sin(direction)

        self.initial_x = center_x
        self.initial_y = center_y
        self.time_elapsed = 0

    def move_boundary(self, dt):
        self.center_x += self.V_bx * dt
        self.center_y += self.V_by * dt

        if self.center_x < self.minX or self.center_x > self.maxX:
            self.V_bx = -self.V_bx
        if self.center_y < self.minY or self.center_y > self.maxY:
            self.V_by = -self.V_by

    def move_in_a_circle(self, dt, radius):
        self.time_elapsed += dt

        # Reverse direction if out of bounds
        if self.center_x < self.minX or self.center_x > self.maxX or self.center_y < self.minY or self.center_y > self.maxY:
            self.speed = -self.speed  # Reverse motion in parametric equations

        self.center_x = self.initial_x + radius * np.cos(self.time_elapsed * self.speed / radius)
        self.center_y = self.initial_y + radius * np.sin(self.time_elapsed * self.speed / radius)

    def move_in_a_cycloid(self, dt, radius):
        self.time_elapsed += dt

        # Reverse direction if out of bounds
        if self.center_x < self.minX or self.center_x > self.maxX or self.center_y < self.minY or self.center_y > self.maxY:
            self.speed = -self.speed  # Reverse motion in parametric equations

        self.center_x = self.initial_x + radius * (self.time_elapsed * self.speed / radius - np.sin(self.time_elapsed * self.speed / radius))
        self.center_y = self.initial_y + radius * (1 - np.cos(self.time_elapsed * self.speed / radius))
