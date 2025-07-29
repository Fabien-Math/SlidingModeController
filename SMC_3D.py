import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # ensures 3D plotting works
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.animation import FuncAnimation

# plt.style.use('ggplot')
# plt.style.use('dark_background')
plt.style.use('seaborn-v0_8')

np.random.seed(42)  # for reproducibility

# Constants and robot parameters
m = 10.0  # mass [kg]
d = 10.0   # damping coefficient [Ns/m]
k = 0.0   # restoring coefficient [N/m]

g = 9.81  # gravity acceleration [m/s^2]
rho_water = 1000.0  # density [kg/m^3]
volume = 0.0101  # m^3
F_gravity = m * g
F_buoyancy = rho_water * g * volume
net_buoyancy_force = F_buoyancy - F_gravity  # positive means upward force

# Desired position in 3D (x, y, z)
pos_desired = np.array([5.0, 3.0, -10.0])  # for example 10 meters underwater

dt = 0.001
T = 20
time = np.arange(0, T, dt)


# Define thruster system
class ThrusterSystem:
    def __init__(self, T, n_thrusters, dt, thruster_limits=(-20, 20), wn=30, zeta=0.7):
        self.T = T
        self.n_thrusters = n_thrusters
        self.dt = dt
        self.min_force, self.max_force = thruster_limits

        # 2nd-order actuator model for each thruster: x'' + 2ζωx' + ω²x = ω²u
        self.wn = wn  # Natural frequency
        self.zeta = zeta  # Damping ratio

        # States: force and velocity for each thruster
        self.force = np.zeros(n_thrusters)
        self.velocity = np.zeros(n_thrusters)

    def update(self, tau_desired):
        # Compute desired thruster forces using pseudo-inverse control allocation
        thruster_cmd = np.linalg.pinv(self.T) @ tau_desired

        # Clip desired thruster commands to actuator physical limits
        thruster_cmd = np.clip(thruster_cmd, self.min_force, self.max_force)

        # Update thruster forces using second-order propeller dynamics
        # Discretize using explicit Euler (or better with RK4 if you want more accuracy)
        accel = self.wn**2 * (thruster_cmd - self.force) - 2 * self.zeta * self.wn * self.velocity
        self.velocity += accel * self.dt
        self.force += self.velocity * self.dt

        # Clip again to be sure the physical force doesn't exceed limits
        self.force = np.clip(self.force, self.min_force, self.max_force)

        # Return the actual force/torque applied on the robot
        tau_actual = self.T @ self.force
        return tau_actual

# Sliding Mode Controller class for 3D
class SlidingModeController3D:
    def __init__(self, k_smc, lambda_smc, phi):
        self.k_smc = k_smc
        self.lambda_smc = lambda_smc
        self.phi = phi

    def compute(self, pos, vel, pos_des):
        s = self.lambda_smc * (pos - pos_des) + vel
        u = - self.k_smc * np.array([np.tanh(si / self.phi) for si in s])
        return s, u

# Disturbance force model for 3D ocean currents
def ocean_current_disturbance_3d(t):
    # Reuse your previous complex disturbance but for x,y,z components
    # For simplicity, add independent components with noise and bursts

    def base_disturbance(freq, amp):
        return amp * np.sin(2 * np.pi * freq * t)

    # x disturbance
    dx = base_disturbance(0.05, 5) + 2 * np.random.normal()
    # y disturbance
    dy = base_disturbance(0.03, 4) + 2 * np.random.normal()
    # z disturbance
    dz = base_disturbance(0.02, 3) + 1.5 * np.random.normal()

    # Occasional bursts (simulate eddies)
    burst = 15.0 if (t % 20) < 2 else 0.0
    dx += burst * np.sin(2 * np.pi * 0.5 * t)
    dy += burst * np.cos(2 * np.pi * 0.5 * t)

    return np.array([dx, dy, dz])

# Initialize state arrays
pos = np.zeros((len(time), 3))     # x, y, z positions
s_log = np.zeros((len(time)-1, 3))     # sliding surface log
u_cmd_log = np.zeros((len(time)-1, 3))     # control command log
vel = np.zeros((len(time), 3))     # velocities
control_forces = np.zeros((len(time), 3))
disturbances = np.zeros((len(time), 3))

# Propeller states (force output, velocity, acceleration) per axis
u_actual = np.zeros(3)
u_dot = np.zeros(3)

# Instantiate the 3D sliding mode controller
smc = SlidingModeController3D(k_smc=50.0, lambda_smc=5.0, phi=0.5)

# Thruster allocation matrix
T = np.array([
    [ 0.707,  0.707, -0.707, -0.707,  0.0,    0.0,    0.0,    0.0],
    [-0.707,  0.707, -0.707,  0.707,  0.0,    0.0,    0.0,    0.0],
    [ 0.0,    0.0,    0.0,    0.0,   1.0,   1.0,   1.0,   1.0],
])

# Create thruster system
thrusters = ThrusterSystem(T=T, n_thrusters=8, dt=dt, thruster_limits=(-20, 20), wn=30, zeta=1.0)


# Simulation loop
for i, t in enumerate(time[:-1]):
    # Calculate control input using SMC
    s, u_cmd = smc.compute(pos[i], vel[i], pos_desired)
    s_log[i] = s  # Log sliding surface for analysis
    u_cmd_log[i] = u_cmd  # Log control commands

    u_actual = thrusters.update(u_cmd)  # Apply thruster dynamics

    # External disturbances
    dist = ocean_current_disturbance_3d(t)
    disturbances[i] = dist

    # Net forces on robot: control + disturbance + buoyancy/gravity on z-axis
    total_force = u_actual + dist
    total_force[2] += net_buoyancy_force  # buoyancy + gravity compensation

    # Dynamics update (Newton's 2nd law)
    acc = (total_force - d * vel[i] - k * pos[i]) / m

    # Euler integration
    vel[i+1] = vel[i] + acc * dt
    pos[i+1] = pos[i] + vel[i+1] * dt
    control_forces[i] = u_actual

# Plot results
fig, axs = plt.subplots(4, 1, figsize=(16, 8), sharex=True)

# Positions
axs[0].plot(time, pos[:, 0], label='x')
axs[0].plot(time, pos[:, 1], label='y')
axs[0].plot(time, pos[:, 2], label='z')
axs[0].axhline(pos_desired[0], color='r', linestyle='--', label='x desired')
axs[0].axhline(pos_desired[1], color='g', linestyle='--', label='y desired')
axs[0].axhline(pos_desired[2], color='b', linestyle='--', label='z desired')
axs[0].set_ylabel('Position [m]')
axs[0].legend()
axs[0].grid(True)
axs[0].set_title('3D Position Tracking')

# Velocities
axs[1].plot(time, vel[:, 0], label='x_dot')
axs[1].plot(time, vel[:, 1], label='y_dot')
axs[1].plot(time, vel[:, 2], label='z_dot')
axs[1].set_ylabel('Velocity [m/s]')
axs[1].legend()
axs[1].grid(True)
axs[1].set_title('Velocities')

# Control forces
axs[2].plot(time, control_forces[:, 0], label='u_x')
axs[2].plot(time, control_forces[:, 1], label='u_y')
axs[2].plot(time, control_forces[:, 2], label='u_z')
axs[2].set_ylabel('Control Forces [N]')
axs[2].legend()
axs[2].grid(True)
axs[2].set_title('Control Forces')

# Disturbances
axs[3].plot(time, disturbances[:, 0], label='dist_x')
axs[3].plot(time, disturbances[:, 1], label='dist_y')
axs[3].plot(time, disturbances[:, 2], label='dist_z')
axs[3].set_xlabel('Time [s]')
axs[3].set_ylabel('Disturbance Forces [N]')
axs[3].legend()
axs[3].grid(True)
axs[3].set_title('Disturbances')

plt.tight_layout()
plt.show()

# Compute distance error
errors = np.linalg.norm(pos - pos_desired, axis=1)
error_min = errors.min()
error_max = errors.max()

# Create segments for the line
points = pos.reshape(-1, 1, 3)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

# Define a custom colormap: green → yellow → red
cmap = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])

# Build the figure
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Create a Line3DCollection colored by error
lc = Line3DCollection(segments, cmap=cmap, norm=plt.Normalize(error_min, error_max))
lc.set_array(errors[:-1])
lc.set_linewidth(2)
ax.add_collection(lc)

# Plot desired position as a red star
ax.scatter(pos_desired[0], pos_desired[1], pos_desired[2],
           color='red', s=150, marker='*', label='Desired Position', zorder=5)

# Axis labels
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('3D Trajectory Colored by Absolute Distance Error')

# Center view on the desired position
margin = 5
ax.set_xlim(pos_desired[0] - margin, pos_desired[0] + margin)
ax.set_ylim(pos_desired[1] - margin, pos_desired[1] + margin)
ax.set_zlim(pos_desired[2] - margin, pos_desired[2] + margin)

# Colorbar with min/max ticks
cbar = fig.colorbar(lc, ax=ax, shrink=0.6, pad=0.1)
cbar.set_label('Distance Error [m]')
cbar.set_ticks([error_min, error_max])
cbar.set_ticklabels([f'{error_min:.2f} m', f'{error_max:.2f} m'])

ax.legend()
plt.show()


# 1. Plot Sliding Surface (s)
fig, axs = plt.subplots(2, 1, figsize=(12, 6), layout="constrained")
for i in range(s_log.shape[1]):
    axs[0].plot(time[:-1], s_log[:, i], label=f's[{i}]')
axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('Sliding Surface s')
axs[0].set_title('Sliding Mode Controller: Sliding Surface Evolution')
axs[0].legend()
axs[0].grid(True)

# 2. Plot Desired Control Wrench (tau_desired)
for i in range(u_cmd_log.shape[1]):
    axs[1].plot(time[:-1], u_cmd_log[:, i], label=f'u_cmd[{i}]')
axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('Desired Control Force/Torque')
axs[1].set_title('Sliding Mode Controller: Desired Control Output')
axs[1].legend()
axs[1].grid(True)
plt.show()


# 1. Plot Phase Diagram
fig, axs = plt.subplots(1, 3, figsize=(12, 6), layout="constrained")
fig.suptitle('Sliding Mode Controller: Phase diagrams')
axis = ['X', 'Y', 'Z']
for i in range(s_log.shape[1]):
    axs[i].scatter(pos_desired[i], 0, color='red', marker='*', s=70, label='Desired Position', zorder=5)
    axs[i].scatter(0, 0, color='C0', marker='o', s=40, label='Initial Position', zorder=5)
    axs[i].plot(pos[:, i], vel[:, i])
    axs[i].set_xlabel(f'{axis[i]} Position [m]')
    axs[i].set_ylabel(f'{axis[i]} Velocity [m/s]')
plt.show()

# 3D Animation of the underwater robot trajectory
positions = pos[::50] 
target = pos_desired

# Create the figure and 3D axis
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Set fixed view centered around the target
ax.set_xlim(target[0] - 5, target[0] + 5)
ax.set_ylim(target[1] - 5, target[1] + 5)
ax.set_zlim(target[2] - 5, target[2] + 5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Underwater Robot Trajectory')

# Desired position as a static point
ax.scatter(*target, color='red', marker='*', s=200, label='Target Position', zorder=5)

# Initialize the robot marker
robot_marker, = ax.plot([], [], [], marker='o', color='blue', markersize=5, label='Robot')

# Initialize the trajectory trace
trace_line, = ax.plot([], [], [], color='blue', alpha=0.5)

# Initialize the trajectory storage
trajectory_x, trajectory_y, trajectory_z = [], [], []

# Animation update function
def update(frame):
    # Current robot position
    x, y, z = positions[frame]
    trajectory_x.append(x)
    trajectory_y.append(y)
    trajectory_z.append(z)

    # Update robot marker and trajectory
    robot_marker.set_data([x], [y])
    robot_marker.set_3d_properties([z])

    trace_line.set_data(trajectory_x, trajectory_y)
    trace_line.set_3d_properties(trajectory_z)

    return robot_marker, trace_line

# Create the animation
ani = FuncAnimation(fig, update, frames=len(positions), interval=20, blit=True, repeat=False)

# Show the legend and plot
ax.legend()
plt.show()
