from networktables import NetworkTables
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# -------------------------
# NetworkTables Setup
# -------------------------
# Replace with your robot's IP address if not running locally
ROBOT_IP = "10.6.56.2"  # Example: FRC team 656 → 10.TE.AM.2

NetworkTables.initialize(server=ROBOT_IP)
sd = NetworkTables.getTable("SmartDashboard")

# -------------------------
# Data storage
# -------------------------
time_data = []
current_velocity_data = []
desired_velocity_data = []

start_time = time.time()

# -------------------------
# Plot setup
# -------------------------
plt.style.use('seaborn-darkgrid')
fig, ax = plt.subplots()
line_current, = ax.plot([], [], label='Current Velocity (rad/s)', color='blue')
line_desired, = ax.plot([], [], label='Desired Velocity (rad/s)', color='red')
ax.set_xlim(0, 10)  # initial x-axis 10 seconds
ax.set_ylim(-10, 10)  # adjust based on expected velocities
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (rad/s)")
ax.set_title("Turret Velocity Real-Time")
ax.legend()
plt.tight_layout()

# -------------------------
# Update function for animation
# -------------------------
def update(frame):
    current_time = time.time() - start_time
    try:
        current_vel = sd.getNumber("Turret-omega", 0.0)
        desired_vel = sd.getNumber("Turret-desired-omega", 0.0)
    except Exception as e:
        print("Error reading NetworkTables:", e)
        current_vel, desired_vel = 0.0, 0.0

    time_data.append(current_time)
    current_velocity_data.append(current_vel)
    desired_velocity_data.append(desired_vel)

    # Keep last 10 seconds of data
    while time_data and time_data[0] < current_time - 10:
        time_data.pop(0)
        current_velocity_data.pop(0)
        desired_velocity_data.pop(0)

    # Update lines
    line_current.set_data(time_data, current_velocity_data)
    line_desired.set_data(time_data, desired_velocity_data)

    ax.set_xlim(max(0, current_time-10), current_time)
    ax.set_ylim(min(min(current_velocity_data+desired_velocity_data)-1, -1),
                max(max(current_velocity_data+desired_velocity_data)+1, 1))
    return line_current, line_desired

# -------------------------
# Animate
# -------------------------
ani = animation.FuncAnimation(fig, update, interval=50)  # 20 Hz
plt.show()

