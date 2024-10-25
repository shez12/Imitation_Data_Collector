# Robot Imitation Learning Project

## Table of Contents

- [Robot Imitation Learning Project](#robot-imitation-learning-project)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [File Descriptions](#file-descriptions)
    - [`controller.py`](#controllerpy)
    - [`record_episode.py`](#record_episodepy)
    - [`real_env.py`](#real_envpy)
    - [`eval_model.py`](#eval_modelpy)



## Overview

The **Robot Imitation Learning Project** is a suite of Python scripts designed to facilitate the control and management of robotic systems using Xbox controllers. This project enables real-time input capture, environment management for robots operating in physical settings, and data recording for analysis and training purposes. The primary components of the project include:

- **`controller.py`**: Interfaces with Xbox controllers to capture user inputs and translate them into commands that control the robot's movements and actions.
- **`record_episode.py`**: Manages the recording of robot interactions, capturing data such as joint positions, velocities, and sensor inputs over the duration of an episode.
- **`real_env.py`**: Establishes and manages the real-world environment in which the robots operate, handling robot states, sensor data, and interactions with various hardware components.
- **`eval_model.py`**: Evaluates trained robotic policies by running them in the real environment, measuring performance metrics such as success rates and average returns.



## File Descriptions

### `controller.py`

**Purpose**:  
Handles interaction with Xbox controllers to capture user inputs such as button presses, joystick movements, and D-Pad actions. These inputs are then translated into actionable commands that dictate the robot's movements and operations.

**Key Features**:
- Initializes and manages multiple Xbox controllers.
- Captures and processes various input types (buttons, axes, hats).
- Translates raw inputs into movement and manipulation commands for the robot.
- Supports control of additional components like grippers and IO interfaces.

### `record_episode.py`

**Purpose**:  
Facilitates the recording of robot operations by capturing and storing data over a series of timesteps. This data can be used for analysis, training machine learning models, or debugging purposes.

**Key Features**:
- Initializes the robot environment and specifies parameters such as robot names, end-effectors, and camera setups.
- Executes a predefined number of timesteps, during which it captures data on robot states and actions.
- Saves the recorded data in a structured format (HDF5) for easy access and analysis.
- Supports data overwriting based on user preference to prevent accidental loss of existing datasets.

### `real_env.py`

**Purpose**:  
Manages the real-world environment in which the robots operate. It interfaces with robotic hardware, handles sensor data, and provides a structured framework for executing and monitoring robot actions.

**Key Features**:
- Initializes multiple robots, controllers, grippers, and IO controls within the environment.
- Retrieves and compiles robot states, including joint positions and velocities.
- Captures sensor data from cameras and other peripherals.
- Provides an observation space compatible with reinforcement learning frameworks.
- Manages the execution of commands, whether from user inputs or predefined scripts, ensuring synchronized control across multiple robots.

### `eval_model.py`

**Purpose**:  
Evaluates trained robotic policies by running them in the real environment. This script measures the performance of these policies through metrics such as success rates and average returns, facilitating the assessment of model effectiveness and reliability.

**Key Features**:
- **Policy Loading**: Loads trained policies from specified checkpoints for evaluation.
- **Environment Setup**: Initializes the real-world environment with designated robots and end-effectors.
- **Evaluation Loop**: Runs multiple rollouts where the policy interacts with the environment, executing actions based on observations.
- **Performance Metrics**: Calculates success rates, average returns, and other relevant metrics to assess policy performance.
- **Data Visualization**: Optionally saves videos of episodes for visual inspection and analysis.
- **Collision Checking**: Integrates collision detection to ensure safe and valid robot movements during evaluation.
- **Command-Line Interface**: Provides flexibility through various command-line arguments for different evaluation scenarios and configurations.
