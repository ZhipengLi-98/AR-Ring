import serial
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
import time


ser = serial.Serial('COM3', 500000)

timestamp = []
acc = []
gyr = []
mag = []
touch = []

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
    mag = frames.mag
    touch = frames.touch

    plt.figure('test')

    plt.subplot(3, 1, 1)
    plt.title('acc')
    draw_points(timestamp, acc, np.array(touch) * 500)
    # print frames.touch
    draw_peaks(timestamp, acc)

    plt.subplot(3, 1, 2)
    plt.title('gyr')
    draw_points(timestamp, gyr, np.array(touch) * 100)
    # key_gyr = frames.caln_key_frame()
    # plt.axvline(key_gyr - 100, color = 'green')
    # plt.axvline(key_gyr + 100, color = 'green')
    draw_peaks(timestamp, gyr)

    plt.subplot(3, 1, 3)
    plt.title('mag')
    draw_points(timestamp, mag, np.array(touch) * 500)
    draw_peaks(timestamp, mag)

    # print('plot done')
    # plt.show()

if __name__ == '__main__':

    frames = Frames()

    start = time.time()
    clk = time.time()
    plt.ion()

    while ((clk - start) < 30.0):
        # print (clk - start)
        s = ser.read()
        line = ""
        while (s != '\n'):
            line += s
            s = ser.read()
        
        line = line.strip().split('-')
        if len(line) == 7 and (clk - start) > 1.0:
            # print line
            timestamp.append(int(line[0]))
            # gyr.append(line[1])
            # gyr.append(line[2])
            temp = line[3].strip().split()
            # gyr.append(temp[0])
            gyr.append(Point(line[1], line[2], temp[0]))
            accx = temp[1]
            accy = temp[2]
            temp = line[4].strip().split()
            # acc.append(temp[0])
            acc.append(Point(accx, accy, temp[0]))
            magx = temp[1]
            # mag.append(line[5])
            temp = line[6].strip().split()
            # mag.append(temp[0])
            mag.append(Point(magx, line[5], temp[0]))
            touch.append(int(temp[1]))

            frames.read_serial(timestamp, gyr, acc, mag, touch)
            frames.preprocess()
            draw_frames(frames)
            plt.pause(0.1)

        # if len(timestamp) > 1:
            # frames.read_serial(timestamp, gyr, acc, mag, touch)
            # frames.preprocess()
            # draw_frames(frames)
        
        clk = time.time()

    end = time.time()
    # plt.show()
    print("Done")
