import os
import sys
from matplotlib import pyplot as plt
import math
import numpy as np
import pandas as pd
from point import Point
from frames import Frames
import scipy.signal as signal
from sklearn.decomposition import PCA
from scipy.stats import skew, kurtosis

def get_file_list():
	root_dir = '.'
	file_name = os.listdir(root_dir)
	
	file_list = []
	for i in range(0, len(file_name)):
		path = os.path.join(root_dir, file_name[i])
		if os.path.isfile(path) and path.split('.')[-1] == 'log':
			file_list.append(path)

	return file_list

def draw_points(timestamp, points, touch):
	n = len(timestamp)
	assert n == len(points)
	x, y, z = Point.points_2_xyz(points)
	plt.plot(timestamp, x, timestamp, y, timestamp, z, timestamp, touch)

def draw_peaks(timestamp, points):
	x, y, z = Point.points_2_xyz(points)
	peaks, _ = signal.find_peaks(abs(x), height=1000, distance=5, threshold=5)
	plt.plot(timestamp[peaks], x[peaks], '.')
	peaks, _ = signal.find_peaks(abs(y), height=1000, distance=5, threshold=5)
	plt.plot(timestamp[peaks], y[peaks], '.')
	peaks, _ = signal.find_peaks(abs(z), height=1000, distance=5, threshold=5)
	plt.plot(timestamp[peaks], z[peaks], '.')

def draw_frames(frames):
	timestamp = frames.timestamp
	acc = frames.acc
	gyr = frames.gyr
	touch = frames.touch

	plt.figure(file)

	plt.subplot(2, 1, 1)
	plt.title('acc')
	draw_points(timestamp, acc, np.array(touch) * 500)
	print frames.touch
	draw_peaks(timestamp, acc)

	plt.subplot(2, 1, 2)
	plt.title('gyr')
	draw_points(timestamp, gyr, np.array(touch) * 100)
	# key_gyr = frames.caln_key_frame()
	# plt.axvline(key_gyr - 100, color = 'green')
	# plt.axvline(key_gyr + 100, color = 'green')
	draw_peaks(timestamp, gyr)

	plt.show()

if __name__ == '__main__':
	# file_list = get_file_list()
	# for file in file_list:
	# 	frames = Frames()
	# 	frames.read_data(file)
	# 	frames.preprocess()
	# 	draw_frames(frames)
	file = 'lzp-index-index.log'
	frames = Frames()
	frames.read_data(file)
	frames.preprocess()
	draw_frames(frames)
