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
import copy
import madgwickahrs

ser = serial.Serial('COM5', 500000)

timestamp = []
acc = []
gyr = []
mag = []
qua = []
touch = []

fileName = "test.txt"

mad = madgwickahrs.MadgwickAHRS()

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
                mg = [float(temp[1])*math.pi/180.0, float(temp[2])*math.pi/180.0, float(temp[3])*math.pi/180.0]
                ma = [float(temp[4])/256.0, float(temp[5])/256.0, float(temp[6])/256.0]
                # mm = [temp[7], temp[8], temp[9]]

                # mad.update(mg, ma, mm)
                mad.update_imu(mg, ma)
                #print(q.q0, q.q1, q.q2, q.q3)
                qTemp = mad.quaternion
                accR = gravity_compensate(qTemp, ma)
                
                f = open(fileName, "a")
                f.write(str(int(round(time.time() * 1000))))
                f.write(" ")
                f.write(str(float(temp[1])*math.pi/180.0))
                f.write(" ")
                f.write(str(float(temp[2])*math.pi/180.0))
                f.write(" ")
                f.write(str(float(temp[3])*math.pi/180.0))
                f.write(" ")
                f.write(str(float(temp[4])/256.0))
                f.write(" ")
                f.write(str(float(temp[5])/256.0))
                f.write(" ")
                f.write(str(float(temp[6])/256.0))
                f.write(" ")
                f.write(str(qTemp[0]))
                f.write(" ")
                f.write(str(qTemp[1]))
                f.write(" ")
                f.write(str(qTemp[2]))
                f.write(" ")
                f.write(str(qTemp[3]))
                f.write(" ")
                f.write(str(int(temp[7])))
                f.write("\n")
                f.close()

                timestamp.append(int(round(time.time() * 1000)))
                gyr.append(float(temp[1])*math.pi/180.0)
                gyr.append(float(temp[2])*math.pi/180.0)
                gyr.append(float(temp[3])*math.pi/180.0)
                # acc.append(Point(temp[4], temp[5], temp[6]))
                acc.append(float(temp[4])/256.0)
                acc.append(float(temp[5])/256.0)
                acc.append(float(temp[6])/256.0)
                # mag.append(Point(temp[7], temp[8], temp[9]))
                qua.append(qTemp[0])
                qua.append(qTemp[1])
                qua.append(qTemp[2])
                qua.append(qTemp[3])
                touch.append(int(temp[7]))
        
        clk = time.time()


if __name__ == '__main__':
    serialRead()

