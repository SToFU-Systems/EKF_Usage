# Extended Kalman Filter (EKF) for Nonlinear Motion Estimation

This repository contains a Python code example demonstrating the use of the Kalman Filter and Extended Kalman Filter (EKF).
The code includes a nonlinear motion model and simulates sensor data with noise. 
The EKF is used to estimate the object's position based on noisy sensor measurements.

The script generates true positions based on a nonlinear acceleration function. 
It then simulates sensor readings by adding noise to these true positions. 
The Extended Kalman Filter is used to estimate the actual position of the object 
by processing these noisy sensor readings.

Visualisation script provides a real-time animated plot showing:
	1. The true position of the object.
	2. Noisy sensor readings.
	3. EKF-based position estimates.
	
For more information please visit our site and read detailed arcticle: 
https://stofu.io/blog/view_post.php?id=18

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Code Explanation](#code-explanation)
- [License](#license)

## Introduction

The Extended Kalman Filter (EKF) is a recursive Bayesian filter that can estimate the state of a dynamic system in the presence of noise. This code example demonstrates how to use the EKF for motion estimation in a one-dimensional space. The motion model is nonlinear, and sensor measurements are simulated with noise.

## Installation

To run this code on your local machine, follow these steps:

1. Clone the repository to your local machine:

    ```
	git clone https://github.com/SToFU-Systems/EKF_Usage.git
    ```

2. Ensure you have Python installed (Python 3.6 or later is recommended).

3. Install the required libraries by running:

    ```
    pip install numpy matplotlib filterpy
    ```

## Usage

To run the code and visualize the EKF motion estimation animation, use the following command:

For Visualisation script:

    
    python Visualisation.py [noise_percentage] [noise_probability] [total_points]
    

For Linear Kalman Filter:	

    
    python LinearKalmanFilter.py [noise_percentage] [noise_probability] [total_points]
    

For Extended Kalman Filter:	

    
    python ExtendedKalmanFilter.py [noise_percentage] [noise_probability] [total_points]
    

This will execute the code and display an animation of the true motion, sensor measurements with noise, and EKF estimates.

## Code Explanation

The code consists of the following key components:

    1. Importing necessary libraries, including NumPy, Matplotlib, and FilterPy (for Kalman filtering).
    2. Defining constants and initial values such as amplitude factors, wave counts, initial position, and more.
    3. Implementation of a nonlinear acceleration function that varies with time.
    4. Functions for adding noise to true positions and updating the Extended Kalman Filter (EKF) with sensor measurements.
    5. Creating an animation using Matplotlib and FuncAnimation to visualize the EKF estimates.
    6. The main entry point of the code, where true positions, sensor data, and the EKF are initialized, and the animation is displayed.

You can customize the code by adjusting parameters like noise levels, prediction intervals, and initial conditions to see how the EKF performs under different conditions.

## License

This code is distributed under the GNU General Public License (GPL) v.3. 