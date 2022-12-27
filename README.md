# Robust Adaptive Control #Inverted Pendulum Control

Used Adaptive Control to estimate parameters of a simple inverted pendulum. Those parameters are assumed to be constant but unknown.

Designed an adaptive controller to estimate unknown parameters of the system and stabilize it to track a time-varying command.

Random disturbance of the system dynamic is also included, and only the maximum magnitude of the disturbance is assumed to be known for the designed controller to reach robustness.

In RobustAdaptive_Control.m file, the simulation is implemented in m-file; while in RobustAdaptive_Control.slx file, the simulation is implemented in simulink environment. 
