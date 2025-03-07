from matplotlib.path import Path
import numpy as np

class BoundaryManager:
    def __init__(self, center_x, center_y, speed, direction, minX, maxX, minY, maxY, thres_dist):
        self.center_x = center_x
        self.center_y = center_y
        self.speed = speed
        self.direction = direction
        self.thres_dist = thres_dist
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY
        self.thres_dist = thres_dist

        self.V_bx = speed * np.cos(direction)
        self.V_by = speed * np.sin(direction)

    def move_boundary(self, dt):
        self.center_x += self.V_bx * dt
        self.center_y += self.V_by * dt

        if self.center_x < self.minX or self.center_x > self.maxX:
            self.V_bx = -self.V_bx
        if self.center_y < self.minY or self.center_y > self.maxY:
            self.V_by = -self.V_by

    def is_inside_boundary(self, x, y):
        dist = np.sqrt((x- self.center_x)**2 + (y-self.center_y)**2)
        if dist < self.thres_dist:
            return True
        return False

    def update_beacons(self, beacons, new_beacon_j):
        for beacon in beacons:
            if self.is_inside_boundary(beacon.x, beacon.y):
                beacon.set_beacon_J(new_beacon_j)
                beacon.active = True
            else:
                beacon.set_beacon_J(0)
                beacon.active = False