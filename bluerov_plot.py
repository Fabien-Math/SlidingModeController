import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

plt.style.use('seaborn-v0_8')
plt.rcParams.update({
	# "figure.figsize": (21.6, 7.2),    # Sets default figure dimensions
	"font.size": 18,        # Sets base font size to 18 (optimal for fullHD displays)
	# "figure.autolayout": True,   # Enables automatic layout adjustments
	'figure.constrained_layout.use': True,
	# "figure.figsize": (2*12.8, 2*7.2/3)    # Sets default figure dimensions
	"figure.figsize": (12.8, 7.2)    # Sets default figure dimensions
})


def plot_3D(var):
	# Create the figure and 3D axis
	fig = plt.figure(figsize=(8, 6))
	ax = fig.add_subplot(111, projection='3d')


	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.set_title('Underwater Robot Trajectory')


	anim_duration = 10 # s
	interval = 40 # ms
	etas_plot = var[::int(max(1, (len(var) * interval * 1e-3) / anim_duration))]

	# Set 1:1:1 aspect ratio by adjusting the axis limits
	xmin, xmax = np.min(etas_plot[:, 0]), np.max(etas_plot[:, 0])
	ymin, ymax = np.min(etas_plot[:, 1]), np.max(etas_plot[:, 1])
	zmin, zmax = np.min(etas_plot[:, 2]), np.max(etas_plot[:, 2])

	max_range = np.array([xmax-xmin, ymax-ymin, zmax-zmin]).max() / 2.0

	mid_x = (xmax+xmin) * 0.5
	mid_y = (ymax+ymin) * 0.5
	mid_z = (zmax+zmin) * 0.5

	ax.set_xlim(mid_x - max_range, mid_x + max_range)
	ax.set_ylim(mid_y - max_range, mid_y + max_range)
	ax.set_zlim(mid_z - max_range, mid_z + max_range)
	ax.invert_zaxis()
	ax.invert_yaxis()
	ax.set_proj_type('ortho')  # FOV = 0 deg

	# Initialize the robot marker
	robot_marker, = ax.plot([], [], [], marker='o', color='blue', markersize=5, label='Robot')

	# Initialize the trajectory trace
	trace_line, = ax.plot([], [], [], color='blue', alpha=0.5)

	# Initialize the trajectory storage
	trajectory_x, trajectory_y, trajectory_z = [], [], []

	# Animation update function
	def update(frame):
		# Current robot position
		x, y, z = etas_plot[frame][:3]
		trajectory_x.append(x)
		trajectory_y.append(y)
		trajectory_z.append(z)

		# Update robot marker and trajectory
		robot_marker.set_data([x], [y])
		robot_marker.set_3d_properties([z])

		trace_line.set_data(trajectory_x, trajectory_y)
		trace_line.set_3d_properties(trajectory_z)

		return robot_marker, trace_line

	# Create the animation
	ani = FuncAnimation(fig, update, frames=len(etas_plot), interval=interval, blit=True, repeat=False)

	# Show the legend and plot
	ax.legend()
	plt.show()




def plot(time, var, title = 'Title'):
	
	# Plot results
	fig, axs = plt.subplots(3, 2, figsize=(16, 8), sharex=True)
	# Positions
	axs[0, 0].plot(time, var[:, 0])
	axs[0, 0].set_ylabel(r'$x ~ [m]$')
	axs[0, 0].grid(True)

	axs[1, 0].plot(time, var[:, 1])
	axs[1, 0].set_ylabel(r'$y ~ [m]$')
	axs[1, 0].grid(True)

	axs[2, 0].set_ylabel(r'$z ~ [m]$')
	axs[2, 0].plot(time, var[:, 2])
	axs[2, 0].grid(True)
	axs[2, 0].set_xlabel(r'$time ~ [s]$')

	axs[0, 1].plot(time, var[:, 3])
	axs[0, 1].set_ylabel(r'$r ~ [rad]$')
	axs[0, 1].grid(True)

	axs[1, 1].plot(time, var[:, 4])
	axs[1, 1].set_ylabel(r'$p ~ [rad]$')
	axs[1, 1].grid(True)

	axs[2, 1].set_ylabel(r'$y ~ [rad]$')
	axs[2, 1].plot(time, var[:, 5])
	axs[2, 1].grid(True)
	axs[2, 1].set_xlabel(r'$time ~ [s]$')


	fig.suptitle(title)



def plot1V(x, y, title = 'Title'):
	
	# Plot results
	# Positions
	plt.plot(x, y)
	plt.grid(True)

	plt.suptitle(title)

def scatter1V(x, y, title = 'Title'):
	
	# Plot results
	# Positions
	plt.scatter(x, y)
	plt.grid(True)

	plt.suptitle(title)




def plot_nm(time, var, n, m, title = 'Title'):
	
	# Plot results
	fig, axs = plt.subplots(n, m, figsize=(16, 8), sharex=True)
	for i in range(m):
		for j in range(n):
			# Positions
			axs[j, i].plot(time, var[:, 0], label=f'{j + i*m}')
			axs[j, i].legend()
			axs[j, i].grid(True)
	for j in range(m):
		axs[n-1, j].set_xlabel(f'Time')

	fig.suptitle(title)

def show():
	plt.show()