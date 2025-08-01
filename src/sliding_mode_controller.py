import numpy as np

# Sliding Mode Controller class for 3D
class SlidingModeController:
	def __init__(self, controller_params):
		self.k_smc = controller_params['k_smc']
		self.lambda_smc = controller_params['lambda_smc']
		self.phi = controller_params['phi']

		self.s = np.zeros(6)
		self.u = np.zeros(6)
		self.cmd = np.zeros(6)

	def update(self, eta_err, nu_err):
		self.s = self.lambda_smc * (- eta_err) + nu_err
		self.u = - self.k_smc * np.array([np.tanh(si / self.phi[i]) for i, si in enumerate(self.s)])
		self.cmd = self.u

	