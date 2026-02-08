# Live NetworkTables Control Plotter

A Python utility to * visualize measured states vs desired setpoints * in real time, designed for robotics and control systems. Useful for * controller tuning, monitoring closed-loop responses, and system diagnostics*.

## Features
 * Live plotting of multiple state variables with their setpoints.
 * Designed for control theory applications: visualize overshoot, steady-state error, and response speed.
 * Modular: define any system’s measurements and setpoints.
 * Works with drivetrain, arm, elevator, shooter, or custom subsystems.

## Requirements
 * Python 3.8+
 * `pynetworktables`
 * `matplotlib`

Install dependencies and load python virtual environment
```
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip3 install -r requirements.txt
```

## Boilerplate Script for using `nt_live_plot`
```
from nt_live_plot import LiveNTPlotter

robot_ip = "10.TE.AM.2"

# Define states to track with corresponding measured and setpoint keys
entries = {
    "X Position": {"measured": "Pose X", "setpoint": "Desired X"},
    "Y Position": {"measured": "Pose Y", "setpoint": "Desired Y"},
    "Heading": {"measured": "Heading", "setpoint": "Desired Heading"}
}

plotter = LiveNTPlotter(robot_ip, entries)
plotter.run()
```