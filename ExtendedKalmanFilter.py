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

# Named constants
AMPLITUDE_FACTOR = 5.0  # Factor for amplitude in the nonlinear acceleration function
WAVES = 4  # Number of waves in the nonlinear motion model
INITIAL_POSITION = 0  # Initial position
TRUE_VELOCITY_INITIAL = 2.0  # Initial true velocity (changed to float)
TRUE_VELOCITY_INCREMENT = 0.2  # Increment for true velocity (added as a constant)
PREDICTION_INTERVAL = 10  # Interval for predictions
DIM_X = 2  # Dimension of state vector in Kalman filter
DIM_Z = 1  # Dimension of measurement vector in Kalman filter
INITIAL_P_COVARIANCE = 1000  # Initial state covariance for Kalman filter

# Initialize random number generator with the current time
random.seed()

# Read noise_percentage, noise_probability, and total_points from command line arguments
noise_percentage = float(sys.argv[1]) / 100.0 if len(sys.argv) > 1 else 0.1
noise_probability = float(sys.argv[2]) / 100.0 if len(sys.argv) > 2 else 0.2
total_points = int(sys.argv[3]) if len(sys.argv) > 3 else 2000
interval = PREDICTION_INTERVAL / total_points  # Time interval between data points

# Define a nonlinear acceleration function that varies with time
def nonlinear_acceleration(t):
    # Calculate the amplitude of acceleration based on time
    amplitude = AMPLITUDE_FACTOR * 200.0 * (1 - abs((t % (total_points / WAVES) - (total_points / (2 * WAVES))) / (total_points / (2 * WAVES))))
    # Calculate the nonlinear acceleration based on time and amplitude
    return amplitude * np.sin(2.0 * np.pi * t / (total_points / WAVES))

# Function to add noise to true positions with a given probability
def add_noise(true_positions, probability):
    # Add Gaussian noise to true positions with specified probability
    return [p + np.random.normal(0, scale=noise_percentage * np.mean(true_positions)) if random.uniform(0, 1) <= probability else p for p in true_positions]

# Function to update the Extended Kalman Filter (EKF) with sensor measurements
def update_extended_kalman_filter(z):
    ekf.predict()  # Predict the next state
    ekf.update(z)  # Update the state based on sensor measurements
    return ekf.x[0, 0]  # Return the estimated position

# Функция для закрытия анимации при нажатии клавиши ESC
def close_animation(event):
    if event.key == 'escape':
        plt.close()

# Function for creating an animation with EKF estimates
def animate_with_ekf(i):
    plt.cla()
    # Plot the true position and sensor data up to the current time
    plt.plot(true_positions_nonlinear[:i], color='green', alpha=0.5, label='True Position')
    plt.plot(sensor_positions_with_noise[:i], color='red', alpha=0.5, label='Sensor Data', linewidth=0.5)

    if i < total_points:
        # Update the EKF with the current sensor measurement and obtain the estimate
        ekf_estimate = update_extended_kalman_filter(sensor_positions_with_noise[i])
        ekf_positions.append(ekf_estimate)

    # Plot the EKF estimate up to the current time
    plt.plot(ekf_positions[:i + 1], color='purple', alpha=0.5, label='Extended Kalman Filter (EKF) Estimate')

    # Set plot limits and labels
    plt.xlim(0, total_points)
    plt.ylim(min(true_positions_nonlinear + sensor_positions_with_noise),
             max(true_positions_nonlinear + sensor_positions_with_noise))
    plt.xlabel('Time (Index)')
    plt.ylabel('Position: X-Axis Coordinate')
    plt.title('Simulation of Nonlinear One-Dimensional Motion with Extended Kalman Filter (EKF)')
    plt.legend()

# Main entry point of the code
if __name__ == "__main__":
    # Create a figure for the animation
    fig, ax = plt.subplots(figsize=(10, 6))

    # Используйте connect для добавления обработчика событий
    fig.canvas.mpl_connect('key_press_event', close_animation)

    # Initialize true_positions_nonlinear with the initial position and true_velocity
    true_positions_nonlinear = [INITIAL_POSITION]
    true_velocity = TRUE_VELOCITY_INITIAL

    # Generate true positions based on the nonlinear motion model
    for t in range(1, total_points):
        acc = nonlinear_acceleration(t)
        true_velocity += acc * interval
        true_positions_nonlinear.append(true_positions_nonlinear[-1] + true_velocity * interval)

    # Add noise to true positions to simulate sensor data
    sensor_positions_with_noise = add_noise(true_positions_nonlinear, noise_probability)

    # Initialize an Extended Kalman Filter (EKF)
    ekf = KalmanFilter(dim_x=DIM_X, dim_z=DIM_Z)
    ekf.F = np.array([[1, interval],
                    [0, 1]])
    ekf.H = np.array([[1, 0]])
    ekf.P *= INITIAL_P_COVARIANCE
    ekf.R = noise_percentage * np.mean(true_positions_nonlinear)

    ekf_x = np.array([[sensor_positions_with_noise[0]],
                    [0]])
    ekf_positions = [ekf_x[0, 0]]

    # Create the animation using FuncAnimation
    ani = FuncAnimation(fig, animate_with_ekf, frames=total_points, interval=interval * 1000, repeat=False)

    # Display the animation
    plt.show()
