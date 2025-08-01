from simulation_parser import load_config
from simulation_manager import SimulationManager
from graphical_simulation_manager import GraphicalSimulationManager

def main():
	# filename = "config/bluerov_config.yaml"
	filename = "config/nautile_config.yaml"
	scenario_params = load_config(filename=filename)
	
	simulation_params, robot_params, environment_params = scenario_params

	sim_manager = None
	if simulation_params['graphical']:
		sim_manager = GraphicalSimulationManager(simulation_params, robot_params, environment_params)
	else:
		sim_manager = SimulationManager(simulation_params, robot_params, environment_params)

	if sim_manager is not None:
		sim_manager.simulate()


if __name__ == '__main__':
	main()