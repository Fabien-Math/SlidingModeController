import matplotlib.pyplot as plt
import numpy as np

# Define thruster system
class ThrusterSystem:
	def __init__(self, n_thrusters=8, thruster_limits=(-35, 45), wn=30, zeta=0.7):
		self.T = np.array([
			[  0.866,   0.866,  -0.839,  -0.839,   0.000,   0.000,   0.000,   0.000],
			[ -0.500,   0.500,  -0.545,   0.545,   0.000,   0.000,   0.000,   0.000],
			[  0.000,   0.000,   0.000,   0.000,   1.000,   1.000,   1.000,   1.000],
			[ -0.033,   0.033,   0.0359, -0.0359,  0.122,   0.122,  -0.117,  -0.117],
			[  0.0654,  0.0654,  0.0653,  0.0653,  0.218,  -0.218,   0.218,  -0.218],
			[ -0.158,   0.158,   0.166,  -0.166,   0.000,   0.000,   0.000,   0.000]
		])
		self.n_thrusters = n_thrusters
		self.min_force, self.max_force = thruster_limits

		# 2nd-order actuator model for each thruster: x'' + 2ζωx' + ω²x = ω²u
		self.wn = wn  # Natural frequency
		self.zeta = zeta  # Damping ratio

		# States: force and velocity for each thruster
		self.force = np.zeros(n_thrusters)
		self.rpm = np.zeros(n_thrusters)

	def update(self, dt):
		# Compute desired thruster forces using pseudo-inverse control allocation
		thruster_cmd = self.T.T @ np.array([1000.0, 0.0, 0.0, 0.0, 0.0, 0.0])

		# Clip desired thruster commands to actuator physical limits
		thruster_cmd = np.clip(thruster_cmd, self.min_force, self.max_force)

		# Update thruster forces using second-order propeller dynamics
		# Discretize using explicit Euler (or better with RK4 if you want more accuracy)
		accel = self.wn**2 * (thruster_cmd - self.force) - 2 * self.zeta * self.wn * self.rpm
		self.rpm += accel * dt
		self.force += self.rpm * dt

		# Clip again to be sure the physical force doesn't exceed limits
		self.force = np.clip(self.force, self.min_force, self.max_force)


thrusters = ThrusterSystem(wn = 8, zeta=0.9)

dt = 0.001
N = 1000
forces = np.zeros((N, 8))
times = np.linspace(dt, N*dt, N)
for i in range(N):
	thrusters.update(dt)
	forces[i] = thrusters.force

plt.style.use("seaborn-v0_8")
plt.plot(times, forces)
plt.plot([times[0], times[-1]], [0.95*45, 0.95*45])
plt.plot([times[0], times[-1]], [0.95*-35, 0.95*-35])
plt.show()