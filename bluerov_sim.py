import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tqdm

plt.style.use('seaborn-v0_8')
plt.rcParams.update({
	# "figure.figsize": (21.6, 7.2),    # Sets default figure dimensions
	"font.size": 18,        # Sets base font size to 18 (optimal for fullHD displays)
	# "figure.autolayout": True,   # Enables automatic layout adjustments
	'figure.constrained_layout.use': True,
	# "figure.figsize": (2*12.8, 2*7.2/3)    # Sets default figure dimensions
	"figure.figsize": (12.8, 7.2)    # Sets default figure dimensions
})


def S(v):
	return np.array([[ 0.00, -v[2],  v[1]],
				     [ v[2],  0.00, -v[0]],
					 [-v[1],  v[0],  0.00]])

class Simulator:
	def __init__(self, dt, eta, nu, m, rg, I0, Ma, Dl, Dq, current_params):
		self.dt = dt
		
		self.eta = eta
		self.eta_prev = eta
		self.eta1, self.eta2 = self.eta[0:3], self.eta[3:6]
		self.nu = nu
		self.nu1, self.nu2 = self.nu[0:3], self.nu[3:6]
		self.nu_rel = self.nu
		self.gamma = np.zeros(6)

		self.m = m
		self.rg = rg
		self.I0 = I0

		self.Mrb = np.zeros((6, 6))
		self.Mrb[0:3, 0:3] =  m * np.identity(3)	# kg
		self.Mrb[0:3, 3:6] = -m * S(rg)				# kg.m
		self.Mrb[3:6, 0:3] =  m * S(rg)				# kg.m
		self.Mrb[3:6, 3:6] =  I0					# kg.m2
		self.Ma = Ma
		self.M = self.Mrb + Ma
		self.Minv = np.linalg.inv(self.M)

		self.Dl = Dl
		self.Dq = Dq
		self.D = self.compute_D()

		self.Crb = np.zeros((6, 6))
		self.Ca = np.zeros((6, 6))
		self.C = np.zeros((6, 6))
		self.compute_C()

		self.T = np.zeros(6)

		self.current_speed = current_params[0]
		self.current_std = current_params[1]
		


	def compute_Crb(self):
		self.Crb[0:3, 3:6] = -self.m * S(self.nu1) - self.m * S(self.nu2) * S(self.rg)
		self.Crb[3:6, 0:3] = -self.m * S(self.nu1) + self.m * S(self.rg) * S(self.nu2)
		self.Crb[3:6, 3:6] = -np.matmul(self.I0, self.nu2)

	def compute_Ca(self):
		Ma11 = self.Ma[0:3, 0:3]
		Ma12 = self.Ma[0:3, 3:6]
		Ma21 = self.Ma[3:6, 0:3]
		Ma22 = self.Ma[3:6, 3:6]
		self.Ca[0:3, 3:6] = - S(np.matmul(Ma11, self.nu1) + np.matmul(Ma12, self.nu2))
		self.Ca[3:6, 0:3] = - S(np.matmul(Ma11, self.nu1) + np.matmul(Ma12, self.nu2))
		self.Ca[3:6, 3:6] = - S(np.matmul(Ma21, self.nu1) + np.matmul(Ma22, self.nu2))

	def compute_C(self):
		self.compute_Crb()
		self.compute_Ca()
		self.C = self.Crb + self.Ca

	def compute_D(self):
		self.D = self.Dl + np.diag(np.diag(self.Dq) * np.abs(self.nu_rel))

	def compute_T(self, T):
		self.T = np.zeros(6)
		self.T[2] = -2	# N (Restitution force: gravity + buoyancy)
		self.T += T

	def compute_gamma(self):
		self.gamma = np.matmul(self.Minv, - np.matmul(self.C, self.nu) - np.matmul(self.D, self.nu_rel) + self.T)

	def update_external_force(self):
		v_fluid =  np.array([np.random.normal(self.current_speed[0], self.current_std[0]), np.random.normal(self.current_speed[1], self.current_std[1]), np.random.normal(self.current_speed[2], self.current_std[2]), 0, 0, 0])
		self.nu_rel = self.nu - v_fluid

	def update_states(self, eta, nu):
		self.nu = nu
		self.eta = eta

	def update(self, T_ext):
		self.update_external_force()
		# DAMPING
		self.compute_D()
		# CORIOLIS AND CENTRIPETAL
		# self.compute_C()
		# EXTERNAL FORCES
		self.compute_T(T_ext)

		# Acceleration calculation
		self.compute_gamma()

