# Rocket Kalman Filter

This repository contains an implementation of a Kalman filter for rocket state estimation. The Kalman filter is a mathematical algorithm used to estimate the state of a dynamic system given noisy measurements. In the context of rocketry, it can be used to estimate parameters such as position, velocity, and acceleration.

## Overview

The Kalman filter implemented here takes measurements from various sensors onboard the rocket, such as accelerometers, gyroscopes, and GPS receivers, and fuses them together to produce an optimal estimate of the rocket's state. The filter operates in a recursive manner, continually updating its estimate as new sensor measurements become available.

## Features

- **State Estimation**: Estimates the rocket's position, velocity, and acceleration over time.
- **Sensor Fusion**: Fuses measurements from multiple sensors to improve estimation accuracy.
- **Dynamic Model**: Incorporates a dynamic model of the rocket's motion to predict its future state.
- **Adaptive Filtering**: Capable of adapting to changes in the system dynamics or sensor characteristics.

## Installation

To use this Kalman filter implementation, simply clone the repository to your local machine:

```bash
git clone https://github.com/Arbalest-Rocketry/kalmanfilter.git
