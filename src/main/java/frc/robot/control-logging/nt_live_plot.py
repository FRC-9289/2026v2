from networktables import NetworkTables
import matplotlib.pyplot as plt
import time


class LiveNTPlotter:
    def __init__(self, robot_ip, entries, update_period=0.05):
        NetworkTables.initialize(server=robot_ip)
        self.table = NetworkTables.getTable("SmartDashboard")

        self.entries = entries
        self.update_period = update_period

        self.start_time = time.time()
        self.times = []

        self.data = {}
        for name in entries:
            self.data[name] = []

        plt.ion()
        self.fig, self.axes = plt.subplots(len(entries), 1)


    def read_values(self):
        t = time.time() - self.start_time
        self.times.append(t)

        for name, keys in self.entries.items():
            value = self.table.getNumber(keys["measured"], 0.0)
            setpoint = self.table.getNumber(keys["setpoint"], 0.0)
            self.data[name].append((value, setpoint))


    def update_plot(self):
        for ax in self.axes:
            ax.clear()

        for i, name in enumerate(self.entries):
            values = [v for v, s in self.data[name]]
            setpoints = [s for v, s in self.data[name]]

            self.axes[i].plot(self.times, values)
            self.axes[i].plot(self.times, setpoints)
            self.axes[i].set_title(name)

        plt.pause(self.update_period)


    def run(self):
        while True:
            self.read_values()
            self.update_plot()
            time.sleep(self.update_period)
