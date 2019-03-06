import serial
import os
import sys
from matplotlib import pyplot as plt
import math
import numpy as np
import pandas as pd
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
config = "C:\Users\pc\Desktop\lzp\System\\x64\Release\config.txt"

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
    f = open(fileName, "a")
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
                
                f.write(str(int(round(time.time() * 1000))))
                f.write(" ")
                f.write(str(int(temp[0])))
                f.write(" ")
                f.write(str(float(temp[1])*math.pi/180.0))
                f.write(" ")
                f.write(str(float(temp[2])*math.pi/180.0))
                f.write(" ")
                f.write(str(float(temp[3])*math.pi/180.0))
                f.write(" ")
                f.write(str(float(accR[0])))
                f.write(" ")
                f.write(str(float(accR[1])))
                f.write(" ")
                f.write(str(float(accR[2])))
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
        
        clk = time.time()


if __name__ == '__main__':
    configuration = open(config, "r")
    fileName = configuration.read()
    fileName = fileName + ".txt"

    serialRead()

