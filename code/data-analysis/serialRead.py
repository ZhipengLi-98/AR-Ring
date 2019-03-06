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
import threading
import copy
import madgwickahrs

ser = serial.Serial('COM3', 500000)

threads = []
threadLock = threading.Lock()

timestamp = []
acc = []
gyr = []
mag = []
touch = []

c_timestamp = []
c_acc = []
c_gyr = []
c_mag = []
c_touch = []

saveQ = []
saveT = []
saveS = []

frames = Frames()
mad = madgwickahrs.MadgwickAHRS()

def draw(timestamp, gyr, acc, mag, touch):
    while True:
        if len(timestamp) > 1:
            if threadLock.acquire(100):
                c_timestamp = copy.deepcopy(timestamp)
                c_gyr = copy.deepcopy(gyr)
                c_acc = copy.deepcopy(acc)
                # c_mag = copy.deepcopy(mag)
                c_touch = copy.deepcopy(touch)
                threadLock.release()
                frames.read_serial(c_timestamp, c_gyr, c_acc, c_mag, c_touch)
                frames.preprocess()

                draw_frames(frames)

                plt.pause(0.001)

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
    # mag = frames.mag
    touch = frames.touch

    plt.figure('test')

    plt.subplot(2, 1, 1)
    plt.cla()
    plt.title('acc')
    draw_points(timestamp, acc, np.array(touch) * 10)
    # print frames.touch
    # draw_peaks(timestamp, acc)

    plt.subplot(2, 1, 2)
    plt.cla()
    plt.title('gyr')
    draw_points(timestamp, gyr, np.array(touch) * 100)
    # key_gyr = frames.caln_key_frame()
    # plt.axvline(key_gyr - 100, color = 'green')
    # plt.axvline(key_gyr + 100, color = 'green')
    # draw_peaks(timestamp, gyr)

    # plt.subplot(3, 1, 3)
    # plt.cla()
    # plt.title('mag')
    # draw_points(timestamp, mag, np.array(touch) * 500)
    # draw_peaks(timestamp, mag)

    # print('plot done')
    # plt.show()

def gravity_compensate(q, acc):
    g = [0.0, 0.0, 0.0]

    # get expected direction of gravity
    g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
    g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
    g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
    # compensate accelerometer readings with the expected direction of gravity
    return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]

def serialRead():
    start = time.time()
    clk = time.time()
    while True:
        # print (clk - start)
        s = ser.read()
        line = ""
        while (s != '\n'):
            line += s
            s = ser.read()
        
        # line = line.strip().split('-')
        if (clk - start) > 1.0:
            print line
            temp = line.strip().split()
            # print temp
            if len(temp) == 8:
                if threadLock.acquire(100):
                    if len(timestamp) > 300:
                        del timestamp[0]
                        del gyr[0]
                        del acc[0]
                        # del mag[0]
                        del touch[0]
                    # print temp

                    mg = [float(temp[1])*math.pi/180.0, float(temp[2])*math.pi/180.0, float(temp[3])*math.pi/180.0]
                    ma = [float(temp[4])/256.0, float(temp[5])/256.0, float(temp[6])/256.0]

                    # mm = [temp[7], temp[8], temp[9]]
                    

                    # mad.update(mg, ma, mm)
                    mad.update_imu(mg, ma)
                    qTemp = mad.quaternion


                    accR = gravity_compensate(qTemp, ma)

                    timestamp.append(int(temp[0]))
                    gyr.append(Point(temp[1], temp[2], temp[3]))
                    # acc.append(Point(ma[0], ma[1], ma[2]))
                    acc.append(Point(accR[0], accR[1], accR[2]))
                    # mag.append(Point(temp[7], temp[8], temp[9]))
                    touch.append(int(temp[7]))
                    threadLock.release()
        
        clk = time.time()


if __name__ == '__main__':
    plt.ion()

    threadA = threading.Thread(target=serialRead, args=())
    threads.append(threadA)
    threadB = threading.Thread(target=draw, args=(timestamp, gyr, acc, mag, touch))
    threads.append(threadB)
    threadA.start()
    threadB.start()
