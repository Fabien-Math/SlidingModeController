import numpy as np

from sliding_mode_controller import SlidingModeController
from thruster_system import ThrusterSystem

class ControllerManager:
	def __init__(self, controller_params: dict, thruster_system: ThrusterSystem):
		self.desired_tfs = []
		self.desired_tf = None
		self.last_desired_tf = None
		self.mission_finished = False

		self.abs_eta_err = np.zeros(6)
		self.eta_err = np.zeros(6)
		self.eta_tol = controller_params['eta_tol']
		self.nu_err = np.zeros(6)
		self.nu_tol = controller_params['nu_tol']

		self.T = thruster_system.T
		self.thrusters = thruster_system
		self.f_thrust = np.zeros(6)

		if controller_params['type'] == 'SMC':
			self.controller = SlidingModeController(controller_params)
		elif controller_params['type'] == 'PID':
			# TO DO
			...

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
			if np.all(np.abs(self.eta_err) < self.eta_tol) and np.all(np.abs(self.nu_err) < self.nu_tol) :
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
		self.abs_eta_err = self.desired_tf - eta
		eta_err = self.desired_tf - eta
		if np.linalg.norm(eta_err[:3]) > 1.0:
			eta_err[:3] /= np.linalg.norm(eta_err[:3])

		if np.linalg.norm(eta_err[3:]) > 0.5:
			eta_err[3:] /= np.linalg.norm(eta_err[3:])

		self.eta_err = eta_err
		self.nu_err = nu
	

	def update(self, dt, eta, nu):
		self.manage_waypoint(eta, nu)

		self.compute_error(eta, nu)

		self.controller.update(self.eta_err, self.nu_err)
		self.thrusters.update(dt, self.controller.cmd)  # Apply thruster dynamics
		