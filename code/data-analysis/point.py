import numpy as np

class Point:
	x = 0.0
	y = 0.0
	z = 0.0

	def __init__(self, *args):
		if (len(args) == 3):
			self.x = float(args[0])
			self.y = float(args[1])
			self.z = float(args[2])
		else:
			self.x = 0.0
			self.y = 0.0
			self.z = 0.0
	
	def __str__(self):
		return '(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ')'

	@staticmethod
	def points_2_xyz(array):
		n = len(array)
		x = np.array([array[i].x for i in range(n)])
		y = np.array([array[i].y for i in range(n)])
		z = np.array([array[i].z for i in range(n)])
		return x, y, z

	@staticmethod
	def xyz_2_points(x, y, z):
		n = len(x)
		assert n == len(y)
		assert n == len(z)
		points = [Point(x[i], y[i], z[i]) for i in range(n)]
		return points
