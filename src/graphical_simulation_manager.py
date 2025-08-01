from ocean import Ocean
from robot import Robot
from viewer import run_viewer
import tqdm
import numpy as np

class GraphicalSimulationManager:
	def __init__(self, simulation_params, robot_params, environment_params):
		self.dt = simulation_params['timestep']
		self.end_time = simulation_params['end_time']

		self.robot = Robot(robot_params)
		self.ocean = Ocean(environment_params)

	def simulate(self):
		etas = []
		nus = []
		for i in tqdm.tqdm(range(int(self.end_time/self.dt))):
			self.robot.update(self.dt, self.ocean)
			etas.append(self.robot.eta)
			nus.append(self.robot.nu)

			if self.robot.controller.mission_finished:
				print(f"Mision accomplished!\nTotal simulation time: {i*self.dt:.3f} s")
				break
		
		print(np.linalg.norm(nus[-1]))
		run_viewer(etas[::10], nus[::10])