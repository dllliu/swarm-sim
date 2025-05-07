import numpy as np

class Beacon:
    def __init__(self, x, y, beacon_j, thres_dist):
        self.x = x
        self.y = y
        self.internal_freq = np.pi
        self.beacon_j = beacon_j
        self.thres_dist = thres_dist

    def set_beacon_J(self, new_beacon_j):
        self.beacon_j = new_beacon_j

    def is_near(self, beacon_control_points, new_beacon_j):
        for point in beacon_control_points:
            dist = np.sqrt((self.x- point.center_x)**2 + (self.y-point.center_y)**2)
            if dist < self.thres_dist:
                self.beacon_j = new_beacon_j
                return True
        self.beacon_j = 0
        return False