import numpy as np
import pandas as pd
from pykalman import KalmanFilter
from point import Point
import scipy.signal as signal
import math

class Frames:
	timestamp = []
	acc = []
	gyr = []
	mag = []
	touch = []

	def __init__(self):
		self.timestamp = []
		self.acc = []
		self.gyr = []
		self.mag = []

	def len(self):
		n = len(self.timestamp)
		assert n == len(self.acc)
		assert n == len(self.gyr)
		assert n == len(self.mag)
		return n

	def parseTime(self, str):
		tags = str.split('.')
		ms = int(tags[1])
		tags = tags[0].split(':')
		ms = ms + ((int(tags[0]) * 60 + int(tags[1])) * 60 + int(tags[2])) * 1000
		return ms

	def read_data(self, file):
		lines = open(file, 'r')
		for line in lines:
			tags = line.strip().split()
			if (len(tags) == 11):
				self.timestamp.append(int(tags[0]))
				self.gyr.append(Point(tags[1], tags[2], tags[3]))
				self.acc.append(Point(tags[4], tags[5], tags[6]))
				self.mag.append(Point(tags[7], tags[8], tags[9]))
				self.touch.append(int(tags[10]))
	
	def read_serial(self, timestamp, gyr, acc, mag, touch):
		self.timestamp = timestamp
		self.gyr = gyr
		self.acc = acc
		self.mag = mag
		self.touch = touch
	
	def fix_timestamp(self):
		self.timestamp = np.array(self.timestamp) - self.timestamp[0]
		n = len(self.timestamp)
		j = 0
		for i in range(1, n):
			if (self.timestamp[i] != self.timestamp[i - 1]):
				self.timestamp[j : i] = np.linspace(self.timestamp[j], self.timestamp[i], i - j, endpoint = False)
				j = i

	def kalman_filter(self, points):
		x, y, z = Point.points_2_xyz(points)
		
		kf = KalmanFilter(transition_matrices=np.array([[1, 1], [0, 1]]), transition_covariance=0.01 * np.eye(2))
		x_filtered = kf.filter(x)[0]
		y_filtered = kf.filter(y)[0]
		z_filtered = kf.filter(z)[0]
		x = x_filtered[:, 0]
		y = y_filtered[:, 0]
		z = z_filtered[:, 0]
		# dx = x_filtered[:, 1]
		# dy = y_filtered[:, 1]
		# dz = z_filtered[:, 1]

		return Point.xyz_2_points(x, y, z)

	def fix_acc(self):
		x, y, z = Point.points_2_xyz(self.acc)
		x -= np.median(x)
		y -= np.median(y)
		z -= np.median(z)
		self.acc = Point.xyz_2_points(x, y, z)
		self.acc = self.kalman_filter(self.acc)

	def fix_gyr(self):
		self.gyr = self.kalman_filter(self.gyr)

	def fix_mag(self):
		self.mag = self.kalman_filter(self.mag)

	def preprocess(self):
		self.fix_timestamp()
		self.fix_acc()
		self.fix_gyr()
		self.fix_mag()

	def caln_amplitude(self, points):
		n = len(points)
		A = [(points[i].x ** 2 + points[i].y ** 2 + points[i].z ** 2) ** 0.5 for i in range(n)]
		return A

	def caln_key_frame(self):
		A = self.caln_amplitude(self.gyr)
		return A.index(max(A))
