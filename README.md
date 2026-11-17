# Gantry Crane Plant Modeling - Simscape Multibody vs Nonlinear ODE

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=simorxb/gantry-crane)

## Summary
This project builds an open-loop planar gantry crane plant in Simscape Multibody and cross-checks it against the same dynamics written as nonlinear ODEs in MATLAB. A cart force pulse (constant force, then drop to zero) drives the motion. The Multibody animation and the ODE trajectories are compared on the same plot. No controller is closed yet: this repository starts from a verified plant that later control studies can reuse.

## Project Overview
A gantry crane is a cart that moves horizontally while a suspended load swings. The swing couples into the cart motion, so any force step on the cart leaves residual sway after the force is removed. That underdamped load angle is the plant feature that anti-sway control has to handle.

Here the plant is built twice. First as a Simscape Multibody mechanism: prismatic cart on a rail, revolute pivot, light visualization rod, and a point-mass load, with viscous friction on the cart and at the pivot. Second as the classical nonlinear cart-pendulum ODEs, integrated with `ode45`, neglecting the visualization rod mass. Matching the two responses on cart velocity and load angle is the check that the Multibody frames, joint senses, and friction terms are oriented correctly.

The force profile is a rectangular pulse: applied force $F_{\mathrm{step}}$ until $t_{\mathrm{step}}$, then zero. While the force is on, the cart accelerates and the load lags. When the force drops, friction decelerates the cart and the load swings ahead. Steady cart velocity after the pulse is zero.

### Key Features
- **Dual plant**: Simscape Multibody model plus independent nonlinear ODE reference
- **Open-loop pulse**: force on, then force off; no feedback
- **Friction on both joints**: cart viscous damping $b$, pivot rotational damping $c$
- **Mechanics Explorer**: Multibody animation kept enabled for qualitative checks
- **Shared parameters**: `crane_params.m` drives both the `.slx` model and the ODE

## Gantry Crane Modeling

### Plant equations
The ODE plant (rod mass neglected) is:

$$
(M+m)~\ddot{x} + m L (\ddot{\theta}\cos\theta - \dot{\theta}^{2}\sin\theta) = F - b~\dot{x}
$$

$$
L~\ddot{\theta} + \ddot{x}\cos\theta + g\sin\theta + \frac{c}{m L}~\dot{\theta} = 0
$$

Where:
- $M$: cart mass
- $m$: load mass (point mass)
- $L$: rope length
- $b$: cart viscous friction
- $c$: pivot rotational viscous friction
- $g$: gravity
- $x$: cart position along the rail
- $\theta$: load angle from vertical ($\theta = 0$ hangs straight down; $\theta > 0$ means the load is on the $+x$ side of the pivot)
- $F$: horizontal force on the cart

### Nominal parameters
- **Cart mass $M$**: 10 kg
- **Load mass $m$**: 2 kg
- **Rope length $L$**: 2 m
- **Cart friction $b$**: 10 N·s/m
- **Pivot friction $c$**: 0.1 N·m·s/rad
- **Gravity $g$**: 9.81 m/s²

## Simulation Scenario
- **Initial state**: cart and load at rest, $\theta = 0$
- **Input**: $F = F_{\mathrm{step}}$ for $t < t_{\mathrm{step}}$, then $F = 0$
- **Default pulse**: $F_{\mathrm{step}} = 10$ N, $t_{\mathrm{step}} = 3$ s
- **Stop time**: 40 s
- **Outputs logged**: $t$, $F$, $x$, $\dot{x}$, $\theta$ (deg), written to `crane_step.csv`

## Simulink and Simscape Implementation
The Multibody model `crane_open_loop.slx` moves in the world $XY$ plane with gravity $[0,~-g,~0]$. Both joints act along or about their own $Z$ axis. `RT_Rail_Align` rotates $+90°$ about $Y$, so the prismatic joint moves the cart along world $+X$. `RT_Revolute_Align` rotates $-90°$ about $Y$, undoing that rotation, so the revolute joint swings about world $+Z$. Positive $x$ and positive $\theta$ in the model therefore match the sign convention above. Cart force enters through a Simulink-PS converter; position, velocity, and angle leave through PS-Simulink converters. A very light cylindrical rod is included only for animation; the ODE neglects it.

Run `run_step_response.m` to simulate both plants, export the Multibody time series, and plot force, cart velocity, and load angle with Simscape as a solid line and the ODE as a dashed line.

## Files
- **`crane_params.m`**: plant, pulse, and visualization parameters
- **`crane_reference_ode.m`**: nonlinear ODE plant solved with `ode45`
- **`crane_open_loop.slx`**: Simscape Multibody open-loop plant
- **`run_step_response.m`**: runs Multibody + ODE, writes `crane_step.csv`, plots the overlay

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
