from nt_live_plot import LiveNTPlotter

robot_ip = "10.TE.AM.2"

entries = {
    "X Position": {
        "measured": "Pose X",
        "setpoint": "Desired X"
    },
    "Y Position": {
        "measured": "Pose Y",
        "setpoint": "Desired Y"
    },
    "Heading": {
        "measured": "Heading",
        "setpoint": "Desired Heading"
    }
}

plotter = LiveNTPlotter(robot_ip, entries)
plotter.run()