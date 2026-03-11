# Active-Airbrake-Control-Apogee-Prediction-System
Overview

This repository contains a closed-loop 1-Degree of Freedom (1-DOF) flight simulation and control system built in MATLAB/Simulink. It is designed to model the coast phase of a high-power rocket and autonomously control aerodynamic drag (airbrakes) to hit a precise target apogee of 10,000 ft (3048m).

Rather than relying on a pre-calculated reference trajectory, the flight computer utilizes a real-time apogee predictor based on the closed-form drag equation. It continuously calculates the vehicle's kinetic energy, predicts the final altitude, and commands the airbrake servos via a tuned PID controller to minimize apogee error.
Key Features

    Real-Time Apogee Prediction: Implements the closed-form kinematic drag equation (Altitude+k2​⋅ln(1+k1​⋅Velocity2)) to dynamically predict the rocket's final altitude at any given millisecond, accounting for air density and shifting drag coefficients.

    Dynamic Aerodynamic Plant: Replaces static drag approximations with 1-D Lookup Tables populated by OpenRocket Component Analysis wind-tunnel data. The physics engine interpolates between "Clean Cd​" and "Max Cd​" states based on the active brake deployment percentage.

    Altitude-Conditioned Deployment: Includes a hard-coded safety threshold that locks airbrakes at 0% deployment during the high-velocity lower atmosphere coast, safely handing over control to the PID loop only after crossing 2000 meters.

    Actuator Saturation Protection: Actuator commands are clamped between 0.0 (0%) and 1.0 (100%) to perfectly mimic physical servo limits and prevent mathematical integrator windup.

System Architecture

The Simulink model is divided into two primary subsystems:

    The Physics Plant: Takes the brake command (u) from the controller, calculates the blended drag coefficient, computes dynamic pressure and total drag force, and integrates acceleration to output true real-time Velocity and Altitude state variables.

    The Flight Controller: Reads the simulated sensor data. The Apogee Predictor calculates the estimated apogee, subtracts the 3048m target to find the error, and feeds it into a tuned PID controller. The control effort is bounded and passed through the 2000m logic switch before commanding the plant.

Prerequisites

    MATLAB & Simulink

    OpenRocket (to generate custom Cd​ vs. Mach lookup tables for your specific airframe)

Tuning and Configuration

To adapt this model for a different rocket, users must update:

    The Initial Conditions in the Integrator blocks (Burnout Velocity and Burnout Altitude).

    The 1-D Lookup Tables with specific Mach vs. Cd​ arrays.

    The k1​ and k2​ ballistic constants in the Apogee Predictor, calculated using the vehicle's specific mass, reference area, and average active-control air density.
