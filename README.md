# Model-Free Optimal Control Design Using Policy Iteration for LQR Problems - MATLAB

This repository contains MATLAB codes for solving Linear Quadratic Regulator (LQR) problems using model-free Reinforcement Learning (RL) techniques. It includes methods such as Policy Iteration, Value Iteration, and Integral Reinforcement Learning (IRL) for continuous state-space systems. These approaches allow us to compute the optimal control gains without needing the system's dynamics explicitly.

## Table of Contents
- [Overview](#overview)
- [Requirements](#requirements)
- [Installation](#installation)
- [Policy and Value Iteration](#policy-and-value-iteration)
- [LQR Model-Free RL](#lqr-model-free-rl)
- [Integral Reinforcement Learning (IRL)](#integral-reinforcement-learning-irl)
- [Comparison with Riccati Solution](#comparison-with-riccati-solution)


## Overview
This repository aims to provide a set of MATLAB codes to solve LQR control problems using model-free RL techniques. The primary focus is on:
1. **Policy and Value Iteration**: Finding the optimal gain through generalized policy and value iteration.
2. **LQR Model-Free RL**: Using RL to determine optimal gains and comparing them with the traditional Riccati solution.
3. **Integral RL (IRL)**: Applying IRL for continuous state-space systems to compute optimal gains.

These methods allow optimal control to be designed without explicitly knowing the system model, making them ideal for scenarios where system dynamics are unknown or difficult to model.

## Requirements
- MATLAB (tested on version XX.XX)
- Control System Toolbox
- Reinforcement Learning Toolbox (optional, if required for additional functionalities)

## Installation
1. Open MATLAB and navigate to the directory where the repository is located.
2. Add the repository to your MATLAB path:
    ```matlab
    addpath(genpath('path_to_repository'));
    ```

## Policy and Value Iteration
This section implements **Generalized Policy Iteration (GPI)** and **Value Iteration (VI)** methods for solving the optimal LQR gain in a model-free manner. The codes perform iterations over the policy and value functions to find the optimal feedback gain for the system.

### Approach:
- Initialize with an arbitrary stabilizing feedback policy.
- Iterate between policy evaluation and policy improvement until the optimal gain is found.
- This is done without any explicit model of the system dynamics.

### Key Concepts:
- **Policy Iteration**: Alternates between policy evaluation (estimating value function) and policy improvement.
- **Value Iteration**: Directly iterates on the value function to converge to the optimal solution.

## LQR Model-Free RL
This module focuses on using model-free RL to solve the LQR problem and find optimal gains, comparing them to the analytical solution obtained via the Riccati equation. This demonstrates how RL can perform control tasks without knowledge of the system's underlying dynamics.

### Approach:
- Implement the RL framework where the agent interacts with the system, collects data, and updates its policy.
- Compare the RL-derived optimal gains with the Riccati solution to assess performance.

### Key Concepts:
- **Model-Free RL**: Control design without needing a system model, relying on real-time data.
- **Optimal Gain Comparison**: Evaluating the performance of RL-based gains vs. traditional Riccati solutions.

## Integral Reinforcement Learning (IRL)
Integral RL (IRL) is a special form of reinforcement learning applied to continuous state-space systems. It is particularly useful for systems with continuous-time dynamics where the goal is to find optimal gains using real-time data.

### Approach:
- Use an integral reinforcement learning formulation to iteratively compute optimal gains.
- This method adapts to the system's continuous nature and works without needing to discretize the problem.

### Key Concepts:
- **Integral RL (IRL)**: A model-free RL approach for continuous-time systems.
- **Continuous State-Space**: Systems where states evolve continuously over time.

## Comparison with Riccati Solution
In all approaches, the final step compares the RL-based optimal control gains with the analytical solution obtained via the **Algebraic Riccati Equation (ARE)**. This comparison helps to validate the performance of model-free RL methods against the traditional model-based solution.



