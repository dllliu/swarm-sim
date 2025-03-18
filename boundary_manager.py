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

    def move_boundary(self, dt):
        self.center_x += self.V_bx * dt
        self.center_y += self.V_by * dt

        if self.center_x < self.minX or self.center_x > self.maxX:
            self.V_bx = -self.V_bx
        if self.center_y < self.minY or self.center_y > self.maxY:
            self.V_by = -self.V_by
