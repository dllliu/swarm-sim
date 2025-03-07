import numpy as np

class Beacon:
    def __init__(self, x, y, beacon_j=0):
        self.x = x
        self.y = y
        self.internal_freq = np.pi
        self.beacon_j = beacon_j
        self.active = True if beacon_j > 0 else False

    def set_beacon_J(self, new_beacon_j):
        self.beacon_j = new_beacon_j