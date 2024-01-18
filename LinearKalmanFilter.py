#
#  Copyright (C) 2024, SToFU Systems S.L.
#  All rights reserved.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import sys
import random

# Read noise_percentage, noise_probability, total_points from the command line
noise_percentage = float(sys.argv[1]) / 100.0 if len(sys.argv) > 1 else 0.1
noise_probability = float(sys.argv[2]) / 100.0 if len(sys.argv) > 2 else 0.2
total_points = int(sys.argv[3]) if len(sys.argv) > 3 else 2000
interval = 10 / total_points

# Initialize true positions and velocity
true_positions_linear = [0]
true_velocity = 2

for t in range(1, total_points):
    true_positions_linear.append(true_positions_linear[-1] + true_velocity * interval)

# Function to add noise to true positions
def add_noise(true_positions, probability):
    return [p + np.random.normal(0, scale=noise_percentage * np.mean(true_positions))
            if random.uniform(0, 1) <= probability else p for p in true_positions]

sensor_positions_with_noise = add_noise(true_positions_linear, noise_probability)

# Initialize Kalman Filter (KF)
kf = KalmanFilter(dim_x=2, dim_z=1)
kf.F = np.array([[1, interval],
                 [0, 1]])
kf.H = np.array([[1, 0]])
kf.P *= 1000
kf.R = noise_percentage * np.mean(true_positions_linear)
kf.Q = Q_discrete_white_noise(dim=2, dt=interval, var=0.001)

x = np.array([[sensor_positions_with_noise[0]],
              [0]])
kf_positions = [x[0, 0]]

# Function to update Kalman Filter with sensor data
def update_kalman_filter(z):
    kf.predict()
    kf.update(z)
    return kf.x[0, 0]

# Function to create an animation with Kalman Filter (KF) estimation
def animate_with_kalman(i):
    plt.cla()
    plt.plot(true_positions_linear[:i], color='green', alpha=0.5, label='True Position')
    plt.plot(sensor_positions_with_noise[:i], color='red', alpha=0.5, linewidth=0.5, label='Sensor Data')

    if i < total_points:
        kf_estimate = update_kalman_filter(sensor_positions_with_noise[i])
        kf_positions.append(kf_estimate)

    plt.plot(kf_positions[:i + 1], color='blue', alpha=0.5, label='Kalman Filter (KF) Estimate')

    plt.xlim(0, total_points)
    plt.ylim(min(true_positions_linear + sensor_positions_with_noise),
             max(true_positions_linear + sensor_positions_with_noise))
    plt.xlabel('Time (Index)')
    plt.ylabel('Position: X-Axis Coordinate')
    plt.title('Simulation of Linear One-Dimensional Motion with Kalman Filter (KF)')
    plt.legend()

# Main entry point
if __name__ == "__main__":
    # Create a figure and animation
    fig, ax = plt.subplots(figsize=(10, 6))
    ani = FuncAnimation(fig, animate_with_kalman, frames=total_points, interval=interval * 1000, repeat=False)

    # Display the animation
    plt.show()
