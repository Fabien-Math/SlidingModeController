import numpy as np
import tqdm
from bluerov_sim import Simulator
from scipy.spatial.transform import Rotation as R
import bluerov_plot as bplt

class BlueROV:
	def __init__(self, k_smc, lambda_smc, phi):
		self.dt = None
		self.eta = np.zeros(6)
		self.eta_prev = np.zeros(6)
		self.nu = np.zeros(6)

		# MASS
		self.m = 11.5								# kg
		self.rg = np.array([0.0, 0.0, 0.02])			# m
		self.I0 = np.array([[0.16, 0.00, 0.00],
					[0.00, 0.16, 0.00],
					[0.00, 0.00, 0.16]])		# k.m2
		self.Ma = np.diag([5.5, 12.7, 14.57, 0.12, 0.12, 0.12])
		
		# DAMPING
		self.Dl = np.diag([4.03, 6.22, 5.18, 0.07, 0.07, 0.07])
		self.Dq = np.diag([18.18, 21.66, 36.99, 1.55, 1.55, 1.55])


		self.controller = Controller(k_smc, lambda_smc, phi)
		self.control_force = np.zeros(6)
	
	def update(self, dt, gamma):
		self.dt = dt
		self.controller.update(self.dt, self.eta, self.nu)
		self.control_force = self.controller.f_thrust
		self.gamma = gamma
		# Velocity calculation
		self.compute_nu()
		# Position calculation
		self.compute_eta()
	
	def compute_nu(self):
		self.nu = (self.eta - self.eta_prev) / self.dt
	
	def compute_eta(self):
		# Verlet integration for position
		new_eta = 2*self.eta - self.eta_prev + self.dt**2 * self.gamma
		self.eta_prev = self.eta
		self.eta = new_eta


		


# Define thruster system
class ThrusterSystem:
	def __init__(self, T, n_thrusters, thruster_limits=(-20, 20), wn=30, zeta=0.7):
		self.T = T
		self.n_thrusters = n_thrusters
		self.min_force, self.max_force = thruster_limits

		# 2nd-order actuator model for each thruster: x'' + 2ζωx' + ω²x = ω²u
		self.wn = wn  # Natural frequency
		self.zeta = zeta  # Damping ratio

		# States: force and velocity for each thruster
		self.force = np.zeros(n_thrusters)
		self.velocity = np.zeros(n_thrusters)

	def update(self, dt, tau_desired):
		# Compute desired thruster forces using pseudo-inverse control allocation
		thruster_cmd = self.T.T @ tau_desired

		# Clip desired thruster commands to actuator physical limits
		thruster_cmd = np.clip(thruster_cmd, self.min_force, self.max_force)

		# Update thruster forces using second-order propeller dynamics
		# Discretize using explicit Euler (or better with RK4 if you want more accuracy)
		accel = self.wn**2 * (thruster_cmd - self.force) - 2 * self.zeta * self.wn * self.velocity
		self.velocity += accel * dt
		self.force += self.velocity * dt

		# Clip again to be sure the physical force doesn't exceed limits
		self.force = np.clip(self.force, self.min_force, self.max_force)



# Sliding Mode Controller class for 3D
class SlidingModeController3D:
	def __init__(self, k_smc, lambda_smc, phi):
		self.k_smc = k_smc
		self.lambda_smc = lambda_smc
		self.phi = phi
		
		self.s = np.zeros(6)
		self.u = np.zeros(6)

	def compute(self, eta_err, nu_err):
		self.s = self.lambda_smc * (- eta_err) + nu_err
		self.u = - self.k_smc * np.array([np.tanh(si / self.phi) for si in self.s])

	
class Controller:
	def __init__(self, k_smc, lambda_smc, phi):
		self.desired_tfs = []
		self.desired_tf = None
		self.last_desired_tf = None
		self.mission_finished = False

		self.eta_err = np.zeros(6)
		self.eta_tol = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
		self.nu_err = np.zeros(6)
		self.nu_tol = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

		self.T = np.array([
			[  0.866,   0.866,  -0.839,  -0.839,   0.000,   0.000,   0.000,   0.000],
			[ -0.500,   0.500,  -0.545,   0.545,   0.000,   0.000,   0.000,   0.000],
			[  0.000,   0.000,   0.000,   0.000,   1.000,   1.000,   1.000,   1.000],
			[ -0.033,   0.033,   0.0359, -0.0359,  0.122,   0.122,  -0.117,  -0.117],
			[  0.0654,  0.0654,  0.0653,  0.0653,  0.218,  -0.218,   0.218,  -0.218],
			[ -0.158,   0.158,   0.166,  -0.166,   0.000,   0.000,   0.000,   0.000]
		])
		self.thrusters = ThrusterSystem(T=self.T, n_thrusters=8, thruster_limits=(-35, 45), wn=30, zeta=1.0)
		self.f_thrust = np.zeros(6)

		self.SMC = SlidingModeController3D(k_smc=k_smc, lambda_smc=lambda_smc, phi=phi)

	def add_waypoint(self, wps, overwrite = False):
		if overwrite:
			if len(np.shape(wps) > 1):
				self.desired_tfs = wps
			else:
				self.desired_tfs = [wps]
		else:
			if len(np.shape(wps)) > 1:
				for wp in wps:
					self.desired_tfs.append(wp)
			else:
				self.desired_tfs.append(wps)
			
	def manage_waypoint(self, eta, nu):
		if self.desired_tf is not None:
			if np.all(np.abs(self.eta_err) < self.eta_tol):
				self.last_desired_tf = self.desired_tf
				# print('Arrived')
				self.desired_tf = None
			else:
				return
		
		if self.desired_tf is None:
			if len(self.desired_tfs):
				self.desired_tf = self.desired_tfs.pop(0)
			else:
				self.mission_finished = True
				if self.last_desired_tf is not None:
					self.desired_tf = self.last_desired_tf
				else:
					self.desired_tf = eta
			
	def compute_error(self, eta, nu):
		eta_diff = self.desired_tf - eta
		R_world_to_bluerov = R.from_euler('xyz', eta[3:6]).as_matrix().T  # transpose = inverse

		pos_error_world = eta_diff[:3]
		oriented_pos_error = np.matmul(R_world_to_bluerov, pos_error_world)[:2]
		eta_diff[:2] = oriented_pos_error
		if np.linalg.norm(eta_diff[:3]) > 1.0:
			eta_diff[:3] /= np.linalg.norm(eta_diff[:3])

		if np.linalg.norm(eta_diff[3:]) > 0.3:
			eta_diff[3:] /= np.linalg.norm(eta_diff[3:])

		self.eta_err = eta_diff
		self.nu_err = nu
		
	def update(self, dt, eta, nu):
		self.manage_waypoint(eta, nu)

		self.compute_error(eta, nu)


		self.SMC.compute(self.eta_err, self.nu_err)
		self.thrusters.update(dt, self.SMC.u)  # Apply thruster dynamics
		self.f_thrust = self.T @ self.thrusters.force



def compute_total_distance(etas):
	d = 0
	for i in range(len(etas) - 1):
		d += np.linalg.norm(etas[i+1] - etas[i])
	print(f"Total distance: {d:.3f} m")
	return d



def simulate(k_smc, lambda_smc, phi, current_params):
	dt = 0.01
	
	bl = BlueROV(k_smc, lambda_smc, phi)
	bl.controller.add_waypoint(np.array([[0.5, 5.0, 0, 0, 0, 0], [-4.0, 0.0, -3.0, 0, 0, 0], [0, 0, 20, 0, 0, 0]]))
	sim = Simulator(dt, bl.eta, bl.nu, bl.m, bl.rg, bl.I0, bl.Ma, bl.Dl, bl.Dq, current_params)

	N = 2000

	# LOGGING
	gammas = np.zeros((N, 6))
	time = np.zeros(N)
	nus = np.zeros((N, 6))
	etas = np.zeros((N, 6))
	eta_errs = np.zeros((N, 6))
	cmds = np.zeros((N, 6))
	forces = np.zeros((N, 8))

	for i in range(N):
		sim.update(bl.control_force)
		bl.update(dt, sim.gamma)


		gammas[i] = bl.gamma
		eta_errs[i] = bl.controller.eta_err
		nus[i] = bl.nu
		etas[i] = bl.eta
		cmds[i] = bl.controller.SMC.u
		forces[i] = bl.controller.thrusters.force
		time[i] = (i+1)*dt

		if bl.controller.mission_finished:
			print(f"Total simulation time: {i*dt:.3f} s")
			# return i * dt
			break
	
	compute_total_distance(etas)

	bplt.plot(time[:i], eta_errs[:i], 'eta errors')
	bplt.plot(time[:i], cmds[:i], 'U cmd')
	bplt.plot(time[:i], nus[:i], 'nu')
	bplt.plot_nm(time[:i], forces[:i], 4, 2, 'C force cmd')

	bplt.show()

	bplt.plot_3D(etas[:i])

def main_2():
	current_params = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
	k_smc = np.array([82.19050697, 63.3188027, 27.44381288, 26.92402467, 19.05816569, 87.67570092])
	lambda_smc = np.array([4.55458967, 2.22107203, 5.06772488, 4.16064917, 4.26646995, 4.44595175])
	phi = 0.8
	t = simulate(k_smc, lambda_smc, phi, current_params)
	print(t)


def main_1():
	current_params = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
	# current_params = [(0.5, -0.5, 0), (0.05, 0.05, 0)]
	refining_step = 10
	nb_pool = 20
	pool_size = 20

	phi = 0.8
	most_probable_k_values = [80.28652322, 63.13938178, 31.62434937, 39.31356554, 17.15406625, 82.97375505]
	most_probable_l_values = [3.37883657, 1.64074502, 5.678491, 4.97961957, 1.78616331, 1.12095108]
	k_disparity = 10
	l_disparity = 3

	min_t = 1e9
	for i in tqdm.tqdm(range(refining_step)):
		params = {'k':None, 'l':None, 't':None}
		for _ in tqdm.tqdm(range(nb_pool)):
			pool_most_probable_k_values = np.abs(np.random.normal(most_probable_k_values, k_disparity))
			pool_most_probable_l_values = np.abs(np.random.normal(most_probable_l_values, l_disparity))
			pool_k_disparity = 3
			pool_l_disparity = 0.5

			pool_params = {'k':None, 'l':None, 't':None}
			pool_min_t = 1e9
			for _ in tqdm.tqdm(range(pool_size)):
				k_smc = np.abs(np.random.normal(pool_most_probable_k_values, pool_k_disparity))
				lambda_smc = np.abs(np.random.normal(pool_most_probable_l_values, pool_l_disparity))
				
				t = simulate(k_smc, lambda_smc, phi, current_params)
				if t is None:
					continue

				if t < pool_min_t:
					pool_params['k'] = k_smc
					pool_params['l'] = lambda_smc
					pool_params['t'] = t
					pool_min_t = t
					print(f"Min time: {t:.3f}")
			
			if pool_params['t'] is None:
				continue
			
			if pool_params['t'] < min_t:
				params['k'] = pool_params['k']
				params['l'] = pool_params['l']
				params['t'] = pool_params['t']
				min_t = pool_params['t']

		if params['t'] is not None:
			most_probable_k_values = params['k']
			most_probable_l_values = params['l']
			print(f"Min time at iteration {i}: {params['t']:.3f}s")
	
	print("k_smc:", most_probable_k_values)
	print("lambda_smc:", most_probable_l_values)
	print("min time:", min_t)
	# bplt.scatter1V(xs, times)
	# bplt.show()


if __name__ == '__main__':
	main_2()

### No current
# k_smc = [80.28652322 63.13938178 31.62434937 39.31356554 17.15406625 82.97375505]
# lambda_smc = [3.37883657 1.64074502 5.678491   4.97961957 1.78616331 1.12095108]
# min_t = 14.480 s

### No current
# k_smc: [82.19050697 63.3188027  27.44381288 26.92402467 19.05816569 87.67570092]
# lambda_smc: [4.55458967 2.22107203 5.06772488 4.16064917 4.26646995 4.44595175]
# min time: 14.32

### with current
# k_smc: [64.21895646 57.94866604 34.89806393 26.78179783 28.69251328 94.58876751]
# lambda_smc: [ 5.44071796  6.07685165  6.881452   12.16265169  7.32699484  3.50207908]
# min time: 13.73