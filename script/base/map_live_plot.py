import matplotlib.pyplot as plt
import numpy as np

# Create the figure and axis
fig, ax = plt.subplots()
plt.figure(figsize=(6, 6))
plt.xlim(0, 4)
plt.ylim(0, 4)

# Initialize a point that will be updated
point, = ax.plot([], [], 'bo')  # 'bo' means blue circle

# Initial position
x, y = [0], [0]

# Function to update the point position


def update_point(new_x, new_y):
    point.set_data(new_x, new_y)
    plt.draw()


# Simulate live updates
for i in range(100):
    new_x = np.random.uniform(0, 4)
    new_y = np.random.uniform(0, 4)

    # Update the point on the plot
    update_point(new_x, new_y)

    # Pause to simulate real-time data updates
    plt.pause(0.1)

# Keep the plot open
plt.show()
