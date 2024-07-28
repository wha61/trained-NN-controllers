# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

#### Project Overview
This repository aims to develop and test custom neural network controllers for the Crazyflie 2.1 drone, using Hamilton-Jacobi Reachability Analysis combined with reinforcement learning methods. The goal is to enhance the robustness of the trained algorithms and validate their performance in real-world flight tests.

#### To Do List
1. **Develop Custom Neural Network Controller in C**:
    - Accept 17 input parameters: 
      - Positions (3): `xyz`
      - Quaternions (4): `quat`
      - Velocities (3): `vel`
      - Angular Velocities (3): `ang_v`
      - Last Clipped Actions (4): `last_clipped_action`
    - Simulate the neural network processing using the extracted weights and biases.
    - Generate 4 output parameters:
      - PWM for each motor: `pwm = 30000 + np.clip(action, -1, +1) * 30000`
      
2. **Integrate Custom Controller into Firmware**:
    - Compile the custom controller into the Crazyflie firmware.
    - Flash the firmware onto the Crazyflie 2.1 drone.

3. **Conduct Flight Tests**:
    - Use CrazySwarm to script flight operations such as takeoff, hovering, and landing.
    - Validate the stability and effectiveness of the custom controller under various disturbance conditions.

#### Environment Details
- **quadrotor_null**: The quadrotor environment with no disturbance (Baseline for RAP and RARL).
- **quadrotor_boltz**: The quadrotor environment with Boltzmann distributed disturbance (Combined with PPO).
- **quadrotor_fixed**: The quadrotor environment with constant HJ disturbance (Needs manual tuning of disturbance level, combined with PPO).
- **quadrotor_random**: The quadrotor environment with bounded random disturbance (Needs manual setting of bounds, combined with PPO).

#### Input Parameters
- **Observation (17)**: `[pos (3: xyz), quat (4), vel (3), ang_v (3), last_clipped_action (4)]`

#### Output Parameters
- **Action (4)**: `pwm for each motor (pwm = 30000 + np.clip(action, -1, +1) * 30000)`

#### Reference

[How our neural network works](https://github.com/Hu-Hanyang/safe-control-gym/blob/main/WorkingLogs.md)
