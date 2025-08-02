import numpy as np
from OpenGL.GL import *

class RobotTrace:
	def __init__(self, max_points=100000):
		self.max_points = max_points
		self.positions = np.zeros((max_points, 3), dtype=np.float32)
		self.count = 0
		self.vbo = glGenBuffers(1)

	def update(self, new_positions):
		# Clip to max_points
		if len(new_positions) > self.max_points:
			new_positions = new_positions[::max(2, int(len(new_positions) / self.max_points))]

		self.count = len(new_positions)
		self.positions[:self.count] = new_positions

		# Upload to GPU
		glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
		glBufferData(GL_ARRAY_BUFFER, self.positions[:self.count].nbytes, self.positions[:self.count], GL_DYNAMIC_DRAW)
		glBindBuffer(GL_ARRAY_BUFFER, 0)

	def draw(self):
		if self.count < 2:
			return  # Need at least 2 points to draw a line
		glDisable(GL_LIGHTING)

		glEnableClientState(GL_VERTEX_ARRAY)
		glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
		glVertexPointer(3, GL_FLOAT, 0, None)

		glColor3f(0.1, 0.6, 0.9)  # Red trace
		glLineWidth(1.5)
		glDrawArrays(GL_LINE_STRIP, 0, self.count)

		glLineWidth(1.0)

		glBindBuffer(GL_ARRAY_BUFFER, 0)
		glDisableClientState(GL_VERTEX_ARRAY)
		glEnable(GL_LIGHTING)
