import numpy as np
import matplotlib.pyplot as plt

# Plant parameters
m = 10.0
d = 5.0
k = 1.0

# Constants
g = 9.81  # gravity
volume = 0.011  # cubic meters
rho_water = 1000.0  # kg/m^3

# Forces
F_gravity = m * g
F_buoyancy = rho_water * g * volume

# Net buoyant force (positive = upwards)
net_buoyancy_force = F_buoyancy - F_gravity  # Should be ~small positive number for positive buoyancy


x_desired = 5.0
MAX_FORCE = 50.0

# Propeller parameters (2nd order)
omega_n = 20.0
zeta = 0.8

# PID Controller
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Sliding Mode Controller
class SlidingModeController:
    def __init__(self, k_smc, lambda_smc, phi):
        self.k_smc = k_smc
        self.lambda_smc = lambda_smc
        self.phi = phi  # boundary layer thickness
    
    def compute(self, x, x_dot):
        s = self.lambda_smc * (x - x_desired) + x_dot
        return -self.k_smc * np.tanh(s / self.phi)

# Simplified L1 Adaptive Controller
class L1AdaptiveController:
    def __init__(self, k_nominal=20.0, filter_alpha=0.7):
        self.k_nominal = k_nominal
        self.disturbance_estimate = 0.0
        self.filter_alpha = filter_alpha

    def compute(self, x, x_dot, dt):
        error = x_desired - x
        # Nominal control (PD)
        u_nom = self.k_nominal * error - d * x_dot

        # Adaptive element estimates external force as the difference between expected and actual acceleration
        # Here we simulate it simply as a disturbance estimate adjustment.
        # Low-pass filter applied to avoid aggressive responses.
        self.disturbance_estimate = (1 - self.filter_alpha) * self.disturbance_estimate + self.filter_alpha * 0  # Simplified, could be improved with state observer
        # Subtract estimated disturbance from nominal control
        u_total = u_nom - self.disturbance_estimate

        return u_total

# External disturbance
def disturbance(t):
    sin_disturbance = 5.0 * np.sin(2 * np.pi * 0.1 * t) if 10 < t < 30 else 0.0
    impulse = 30.0 if 4.9 < t < 5.1 else 0.0
    return sin_disturbance + impulse

def ocean_current_disturbance(t):
    # Low frequency: tidal oscillation ~ period of 12 hours (~0.000023 Hz)
    tidal_freq = 1/(12*3600)  
    tidal_force = 5.0 * np.sin(2 * np.pi * tidal_freq * t)
    
    # Mid frequency: waves/current gusts ~ period of 10 seconds
    gust_freq = 1/10
    gust_force = 10.0 * np.sin(2 * np.pi * gust_freq * t + np.pi/4)
    
    # High frequency noise: turbulence (white noise)
    noise = 2.0 * np.random.normal(0, 1)
    
    # Random bursts: simulate sudden surges every ~30 seconds lasting 1-3 sec
    burst_force = 0
    if ((t + 15) % 30) < 3:
        burst_force = 20.0 * np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz burst oscillation
    
    # Total disturbance force
    total_force = tidal_force + gust_force + noise + burst_force
    return total_force


# Simulation parameters
dt = 0.001
T = 50
time = np.arange(0, T, dt)

controllers = {
    'PID': PIDController(kp=20.0, ki=5.0, kd=10.0),
    'SMC': SlidingModeController(k_smc=50.0, lambda_smc=5.0, phi=0.5),
    'L1 Adaptive': L1AdaptiveController(k_nominal=25.0, filter_alpha=0.6)
}

results = {}
disturbances = {}
control_forces = {}

# Simulation loop
for ctrl_name, controller in controllers.items():
    state = np.array([0.0, 0.0])  # [x, x_dot]
    u_actual = 0.0
    u_dot = 0.0
    trajectory = []
    disturbance_values = []
    control_values = []

    for t in time:
        error = x_desired - state[0]

        # Controller command
        if ctrl_name == 'PID':
            u_cmd = controller.compute(error, dt)
        elif ctrl_name == 'SMC':
            u_cmd = controller.compute(state[0], state[1])
        elif ctrl_name == 'L1 Adaptive':
            u_cmd = controller.compute(state[0], state[1], dt)

        # Apply saturation
        u_cmd = np.clip(u_cmd, -MAX_FORCE, MAX_FORCE)

        # Propeller second-order dynamics
        u_ddot = omega_n**2 * (u_cmd - u_actual) - 2 * zeta * omega_n * u_dot
        u_dot += u_ddot * dt
        u_actual += u_dot * dt
        u_actual = np.clip(u_actual, -MAX_FORCE, MAX_FORCE)

        # External disturbance
        dist = ocean_current_disturbance(t)

        # Robot dynamics
        total_force = u_actual + dist + net_buoyancy_force
        x_dot_dot = (total_force - d * state[1] - k * state[0]) / m
        state[1] += x_dot_dot * dt
        state[0] += state[1] * dt

        trajectory.append(state[0])
        disturbance_values.append(dist)
        control_values.append(u_actual)

    results[ctrl_name] = trajectory
    disturbances[ctrl_name] = disturbance_values
    control_forces[ctrl_name] = control_values


# Plot depth & disturbance
for ctrl_name in controllers.keys():
    fig, ax1 = plt.subplots(2, 1, figsize=(10, 9), layout="constrained")
    fig.suptitle(f'{ctrl_name} Controller: Depth and Disturbance (Actuator Lag & Saturation)')

    ax1[0].plot(time, results[ctrl_name], color='tab:blue', label='Depth')
    ax1[0].axhline(y=x_desired, color='k', linestyle='--', label='Desired Depth')
    ax1[0].set_xlabel('Time [s]')
    ax1[0].set_ylabel('Depth [m]', color='tab:blue')
    ax1[0].tick_params(axis='y', labelcolor='tab:blue')
    ax1[0].legend(loc='upper right', bbox_to_anchor=(1, 0.85))
    ax1[0].grid()

    ax2 = ax1[0].twinx()
    ax2.plot(time[::50], disturbances[ctrl_name][::50], color='tab:red', linestyle=':', label='Disturbance Force')
    ax2.set_ylabel('Disturbance Force [N]', color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')

    
    ax1[1].plot(time, control_forces[ctrl_name], color='tab:green', label='Actual Actuator Force')
    ax1[1].axhline(y=MAX_FORCE, color='r', linestyle='--', label='Force Limits')
    ax1[1].axhline(y=-MAX_FORCE, color='r', linestyle='--')
    ax1[1].set_xlabel('Time [s]')
    ax1[1].set_ylabel('Force [N]')
    ax1[1].set_title(f'{ctrl_name} Controller: Actuator Output (With Lag & Saturation)')
    ax1[1].legend()
    ax1[1].grid()
plt.show()
