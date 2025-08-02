import numpy as np

class LoggingSystem:
	def __init__(self, robot):
		self.robot = robot
		self.etas = []
		self.nus = []
		self.thrust_forces = []
		self.thruster_forces = []
		self.etas_err = []
		self.abs_etas_err = []
		self.nus_err = []
		self.desired_tfs = np.array(robot.controller.desired_tfs)
		self.timestamps = []

		self.errs = False
		self.thrust_force = False

	def log_state(self, timestamp):
		"""Append the current robot state to logs."""
		self.etas.append(np.array(self.robot.eta))
		self.nus.append(np.array(self.robot.nu))
		
		if self.thrust_force is not None:
			self.thrust_forces.append(np.array(self.robot.thrusters.force))
			self.thruster_forces.append(np.array(self.robot.thrusters.thrust))
		
		if self.errs is not None:
			self.etas_err.append(np.array(self.robot.controller.etas_err))
			self.abs_etas_err.append(np.array(self.robot.controller.abs_etas_err))
			self.nus_err.append(np.array(self.robot.controller.nus_err))
		
		self.timestamps.append(timestamp)

	def clear_logs(self):
		"""Clear all logs."""
		self.etas.clear()
		self.nus.clear()
		self.thrust_forces.clear()
		self.thruster_forces.clear()
		self.etas_err.clear()
		self.nus_err.clear()
		self.timestamps.clear()

	def np_format(self):
		self.etas = np.array(self.etas)
		self.nus = np.array(self.nus)
		self.thrust_forces = np.array(self.thrust_forces)
		self.thruster_forces = np.array(self.thruster_forces)
		self.etas_err = np.array(self.etas_err)
		self.nus_err = np.array(self.nus_err)
		self.timestamps = np.array(self.timestamps)
		
			
	def save_to_csv(self, filename):
		import csv

		with open(filename, 'w', newline='') as csvfile:
			writer = csv.writer(csvfile)
			
			# Create header with dynamic sizes based on first entries
			header = ['timestamp']
			
			if self.etas and len(self.etas[0]) > 0:
				header += [f'eta_{i}' for i in range(len(self.etas[0]))]
			if self.nus and len(self.nus[0]) > 0:
				header += [f'nu_{i}' for i in range(len(self.nus[0]))]
			if self.thrust_forces and self.thrust_forces[0] is not None:
				header += [f'thrust_force_{i}' for i in range(len(self.thrust_forces[0]))]
			if self.thruster_forces and self.thruster_forces[0] is not None:
				header += [f'thruster_force_{i}' for i in range(len(self.thruster_forces[0]))]
			if self.etas_err and len(self.etas_err[0]) > 0:
				header += [f'etas_err_{i}' for i in range(len(self.etas_err[0]))]
			if self.abs_etas_err and len(self.abs_etas_err[0]) > 0:
				header += [f'etas_err_{i}' for i in range(len(self.abs_etas_err[0]))]
			if self.nus_err and len(self.nus_err[0]) > 0:
				header += [f'nus_err_{i}' for i in range(len(self.nus_err[0]))]

			writer.writerow(header)

			for i in range(len(self.timestamps)):
				row = [self.timestamps[i]]
				
				if self.etas and i < len(self.etas):
					row += list(self.etas[i])
				if self.nus and i < len(self.nus):
					row += list(self.nus[i])
				if self.thrust_forces and i < len(self.thrust_forces) and self.thrust_forces[i] is not None:
					row += list(self.thrust_forces[i])
				else:
					row += []
				if self.thruster_forces and i < len(self.thruster_forces) and self.thruster_forces[i] is not None:
					row += list(self.thruster_forces[i])
				else:
					row += []
				if self.etas_err and i < len(self.etas_err):
					row += list(self.etas_err[i])
				if self.abs_etas_err and i < len(self.abs_etas_err):
					row += list(self.abs_etas_err[i])
				if self.nus_err and i < len(self.nus_err):
					row += list(self.nus_err[i])

				writer.writerow(row)
