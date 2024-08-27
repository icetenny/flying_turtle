#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# IDS = [212, 56, 987, 785]
IDS = [117, 314, 454, 856, 1014]
MAP_NAME = 'map.txt'
MAP_PLOT_NAME = 'map_gt.png'

# Define the grid with step 0.2 in the range [0.2, 3.8]
grid_x = np.arange(0.2, 4.0, 0.2)
grid_y = np.arange(0.2, 4.0, 0.2)

# Generate 4 unique random points
points = []

while len(points) < len(IDS):
    # Randomly select an x and y value from the grid
    x = np.random.choice(grid_x)
    y = np.random.choice(grid_y)
    # Add the point to the set if it's unique
    if (x, y) not in points:
        points.append((x, y))

# Convert the set of points back to a list
random_points = list(points)
wline = [f'{id} {round(p[0], 3)} {round(p[1],3)}\n' for id,
         p in zip(IDS, points)]
with open(MAP_NAME, 'w') as file:
    file.writelines(wline)


plt.figure(figsize=(8, 8))
plt.xlim(0, 4)
plt.ylim(0, 4)

plt.title('MAP')
plt.xlabel('X')
plt.ylabel('Y')

plt.grid(True)

color_rank = ['black'] + ['C' + str(i) for i in range(10)]


map_data = dict()
# Read the content of the file
with open(MAP_NAME, 'r') as file:
    for n, md in enumerate(file.readlines()):
        mid, mx, my = [float(i) for i in md.strip().split()]
        # map_data[int(mid)] = ((mx, my), color_rank[n])
        map_data[int(mid)] = {'x': mx, 'y': my,
                              'plot_color': color_rank[n]}

        plt.plot(mx, my, 'o', color=color_rank[n], label=str(int(mid)))

    plt.legend()
    plt.savefig(MAP_PLOT_NAME)

# Print the content of the file
print(map_data)
