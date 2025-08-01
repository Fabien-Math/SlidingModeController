import yaml
import numpy as np

def load_config(filename):
	"""
	Load full config including robot, thruster, controller, and current parameters from YAML.
	Returns dictionaries for each section with numpy arrays where applicable.
	"""
	with open(filename, 'r') as f:
		cfg = yaml.safe_load(f)
		
	simulation_cfg = cfg.get('simulation', {})
	simulation_params = {
		'timestep': simulation_cfg.get('timestep', 0.01),
		'end_time': simulation_cfg.get('end_time', 10.0),
		'graphical': simulation_cfg.get('graphical', True),
		'train': simulation_cfg.get('train', False),
	}

	# Robot parameters
	robot_cfg = cfg.get('robot', {})
	# Thruster parameters
	thruster_cfg = robot_cfg.get('thruster', {})
	thruster_params = {
		'T': np.array(thruster_cfg.get('T')),
		'n_thrusters': thruster_cfg.get('n_thrusters'),
		'thruster_limits': np.array(thruster_cfg.get('thruster_limits')),
		'wn': thruster_cfg.get('wn'),
		'zeta': thruster_cfg.get('zeta'),
	}

	# Controller parameters
	controller_cfg = robot_cfg.get('controller', {})
	ctrl_type = controller_cfg.get('type')
	controller_params = {
						'type': ctrl_type,
					  	'eta_tol': np.array(controller_cfg.get('eta_tol')),
        				'nu_tol': np.array(controller_cfg.get('nu_tol'))
	}

	if ctrl_type == 'SMC':
		smc_params = controller_cfg.get('smc_params', {})
		controller_params.update({
			'k_smc': np.array(smc_params.get('k_smc')),
			'lambda_smc': np.array(smc_params.get('lambda_smc')),
			'phi': np.array(smc_params.get('phi')),
		})
	elif ctrl_type == 'PID':
		pid_params = controller_cfg.get('pid_params', {})
		controller_params.update({
			'kp': pid_params.get('kp'),
			'ki': pid_params.get('ki'),
			'kd': pid_params.get('kd'),
		})
	else:
		raise ValueError(f"Unsupported controller type: {ctrl_type}")
	
	robot_params = {
		'name': robot_cfg.get('name', 'unknown'),
		'mission': np.array(robot_cfg["mission"]["waypoints"]),  # shape (N, 6)
		'initial_conditions': {
			'eta': np.array(robot_cfg.get('initial_conditions', {}).get('eta')),
			'nu': np.array(robot_cfg.get('initial_conditions', {}).get('nu'))
		},
		'mass_properties': {
			'm': robot_cfg.get('mass_properties', {}).get('m', 0.0),
			'rg': np.array(robot_cfg.get('mass_properties', {}).get('rg')),
			'I0': np.array(robot_cfg.get('mass_properties', {}).get('I0')),
			'Ma': np.array(robot_cfg.get('mass_properties', {}).get('Ma')),
		},
		'damping': {
			'Dl': np.array(robot_cfg.get('damping', {}).get('Dl')),
			'Dq': np.array(robot_cfg.get('damping', {}).get('Dq')),
		},
		'controller': controller_params,
		'thruster': thruster_params,
	}


	# Environment parameters
	env_cfg = cfg.get('environment', {})
	current_cfg = env_cfg.get('current', {})
	current_type = current_cfg.get('type', 'normal')
	current_params = {'type': current_type}
	
	if current_type == 'normal':
		current_params['speed'] = np.array(current_cfg.get('normal', {}).get('speed'))
		current_params['std'] = np.array(current_cfg.get('normal', {}).get('std'))
	elif current_type == 'jet':
		current_params['vector'] = np.array(current_cfg.get('jet', {}).get('vector'))
	elif current_type == 'constant':
		current_params['vector'] = np.array(current_cfg.get('constant', {}).get('vector'))
	elif current_type == 'time_series':
		current_params['time_series'] = [np.array(vec) for vec in current_cfg.get('time_series')]
	elif current_type == 'depth_profile':
		current_params['depth_profile'] = [
			{'depth': entry.get('depth', 0), 'vector': np.array(entry.get('vector'))}
			for entry in current_cfg.get('depth_profile')
		]
	else:
		raise ValueError(f"Unsupported current type: {current_type}")
	
	properties_cfg = env_cfg.get('properties', {})
	environment_params = {
		'gravity': np.array(properties_cfg.get('gravity', [0.0, 0.0, -9.81])),
		'water_density': properties_cfg.get('water_density', 1025.0),
		'water_viscosity': properties_cfg.get('water_viscosity', 1.0e-3),
		'current': current_params,
	}

	return simulation_params, robot_params, environment_params

