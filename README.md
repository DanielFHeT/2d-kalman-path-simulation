# 2d-kalman-path-simulation
MATLAB simulation of a 2D vehicle path using GPS, accelerometer, and gyroscope data fused with a Kalman filter for improved position and heading estimation.

## Requirements
- MATLAB R2021a or later
- Signal Processing Toolbox (if required by the plotting functions)

## How to Run
1. Download or clone this repository to your local machine.
2. Open MATLAB and set the current folder to the project directory.
3. Run `main.m`.
4. The script will generate plots showing:
   - True path
   - Noisy sensor measurements
   - Kalman filter estimates

## Files
- **main.m** — Runs the full simulation: path generation, noise addition, Kalman filtering, and plotting.
- **kalmanFilter.m** — Implements the Kalman filter algorithm for position estimation.
- **generatePath.m** — Creates the object's true path over time.
- **addSensorNoise.m** — Adds measurement noise to the true path.
- **plotResults.m** — Plots true path, noisy measurements, and Kalman filter estimates.

## License
This project is licensed under the MIT License — see the LICENSE file for details.
