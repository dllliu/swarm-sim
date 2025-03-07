import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
import random
from beacons import Beacon
from Swarmalator import Swarmalator
from grid import Grid

N_GRID = 10
M_GRID = 10
NUM_SWARMALATORS = 30
B = 1
dt = 0.1
MIN_THRES = 0.3  # Initial minimum threshold

arr_beacons = []
arr_swarmalators = []
grid = [[Grid()] * N_GRID for _ in range(M_GRID)]

def set_grid_beacons():
    for i in range(N_GRID):
        for j in range(M_GRID):

            #rectangle in middle
            '''
            if j >=3 and j<= 5 and i>=3 and i<=6:
                beac = Beacon(np.array([i,j]), beacon_j=1)
            else:
                beac = Beacon(np.array([i,j]), beacon_j=0)
            '''

            #square in middle
            '''
            if j >=3 and j<= 5 and i>=3 and i<=5:
                beac = Beacon(np.array([i,j]), beacon_j=1)
            else:
                beac = Beacon(np.array([i,j]), beacon_j=0)
            '''

            #cocentric squares in middle
            '''
            if i==0 or i== N_GRID-1 or j==0 or j== M_GRID-1:
                beac = Beacon(np.array([i,j]), beacon_j=1)

            elif j >=3 and j<= 5 and i>=3 and i<=5:
                beac = Beacon(np.array([i,j]), beacon_j=1)

            else:
                beac = Beacon(np.array([i,j]), beacon_j=0)
            '''

            #right triangle
            '''
            if j <= i:
                beac = Beacon(np.array([i,j]), beacon_j=1)
            else:
                beac = Beacon(np.array([i,j]))
            '''


            if (j==1 or j==2) and i==0:
                beac = Beacon(np.array([i,j]), beacon_j=1)
            elif j==1 and i==1:
                beac = Beacon(np.array([i,j]), beacon_j=1)
            elif (i==0 and j<= 4 and j>=1) or (i==2 and j<= 4 and j>=1):
                beac = Beacon(np.array([i,j]), beacon_j=1)
            elif (i==5 and j<= 4 and j>=1) or (i==9 and j<= 4 and j>=1):
                beac = Beacon(np.array([i,j]), beacon_j=1)
            elif (j== 3) and (i ==6 or i==8):
                beac = Beacon(np.array([i,j]), beacon_j=1)
            elif (j== 1 or j==2) and i==7:
                beac = Beacon(np.array([i,j]), beacon_j=1)
            elif i==4 and j>=8:
                beac = Beacon(np.array([i,j]), beacon_j=-1)
            elif j >= 8:
                beac = Beacon(np.array([i,j]), beacon_j=-1)
            else:
                beac = Beacon(np.array([i,j]), beacon_j=0)


            grid[i][j].beacon = beac
            arr_beacons.append(beac)

def set_swarmalators():
    for _ in range(NUM_SWARMALATORS):
        x = random.randint(0, N_GRID - 1)
        y = random.randint(0, M_GRID - 1)
        swarmalator = Swarmalator(np.array([x, y]))
        grid[x][y].swarmalator = swarmalator
        arr_swarmalators.append(swarmalator)

def simulate_with_thres(curr_swarmalator, dt):
    curr_theta = curr_swarmalator.internal_freq
    curr_pos = curr_swarmalator.position

    change_in_pos = np.array([0.0, 0.0])

    num_swarmulators_within_thres = 0
    num_beacons_within_thres = 0

    for beacon in arr_beacons:
        if not np.array_equal(curr_pos, beacon.position):
            distbetweenBeaconAndBot = beacon.position - curr_pos
            magdist_BeaconAndBot = np.linalg.norm(distbetweenBeaconAndBot)
            if magdist_BeaconAndBot < MIN_THRES:
                num_beacons_within_thres +=1

    for other_swarmalator in arr_swarmalators:
         if not np.array_equal(curr_pos, other_swarmalator.position):
            distbetweenBotsAndBot = other_swarmalator.position - curr_pos
            magdist_BotsAndBot = np.linalg.norm(distbetweenBotsAndBot)
            if magdist_BotsAndBot < MIN_THRES:
                num_swarmulators_within_thres +=1

    for beacon in arr_beacons:
        if not np.array_equal(curr_pos, beacon.position):
            distbetweenBeaconAndBot = beacon.position - curr_pos
            magdist_BeaconAndBot = np.linalg.norm(distbetweenBeaconAndBot)
            beacon_force = 0
            if (num_beacons_within_thres == 0):
                beacon_force = (distbetweenBeaconAndBot / (magdist_BeaconAndBot ** 2)) * beacon.beacon_j * np.cos(beacon.internal_freq - curr_swarmalator.internal_freq)
            else:
                beacon_force = ((distbetweenBeaconAndBot / (magdist_BeaconAndBot ** 2)) * beacon.beacon_j * np.cos(beacon.internal_freq - curr_swarmalator.internal_freq)) / np.sqrt(num_beacons_within_thres)
            change_in_pos += beacon_force

    for other_swarmalator in arr_swarmalators:
        if not np.array_equal(curr_pos, other_swarmalator.position):
            distbetweenBotsAndBot = other_swarmalator.position - curr_pos
            magdist_BotsAndBot = np.linalg.norm(distbetweenBotsAndBot)
            repulsion = 0

            if magdist_BotsAndBot < MIN_THRES:
                if (num_swarmulators_within_thres == 0):
                    repulsion = -B * (distbetweenBotsAndBot / (magdist_BotsAndBot ** 2))
                else:
                    repulsion =  (-B * (distbetweenBotsAndBot / (magdist_BotsAndBot ** 2))) / np.sqrt(num_swarmulators_within_thres)
                change_in_pos += repulsion

    new_position = curr_swarmalator.position + change_in_pos * dt
    curr_swarmalator.position = new_position

if __name__ == "__main__":
    set_grid_beacons()
    set_swarmalators()

    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)

    ax.set_xlim(-N_GRID * 1.25, N_GRID * 1.25)
    ax.set_ylim(-M_GRID * 1.25, M_GRID * 1.25)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.grid(True)

    beacon_j_values = [em.beacon_j for em in arr_beacons]

    Beacon_scatter = ax.scatter(
        [em.position[0] for em in arr_beacons],
        [em.position[1] for em in arr_beacons],
        c=beacon_j_values,
        cmap='viridis',
        marker='x',
        label='Beacons'
    )

    cbar = plt.colorbar(Beacon_scatter, ax=ax)
    cbar.set_label('Beacon J Value')

    swarmalator_scatter = ax.scatter(
        [swarmalator.position[0] for swarmalator in arr_swarmalators],
        [swarmalator.position[1] for swarmalator in arr_swarmalators],
        c='red', marker='o', label='Swarmalators'
    )
    ax.legend()

    def update(frame):
        for swarmalator in arr_swarmalators:
            simulate_with_thres(swarmalator, dt)

        #print(MIN_THRES)

        swarmalator_scatter.set_offsets(
            [swarmalator.position for swarmalator in arr_swarmalators]
        )


    ani = FuncAnimation(fig, update, frames=100, interval=100)


    ax_thres = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    slider_thres = Slider(ax_thres, 'Min Thres', 0.05, 10.0, valinit=MIN_THRES)


    def update_threshold(val):
        global MIN_THRES
        MIN_THRES = val

    slider_thres.on_changed(update_threshold)
    plt.show()