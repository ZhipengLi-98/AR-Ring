import numpy as np
from sklearn.svm import SVC
import math
import random
import os
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split,cross_val_score,cross_validate
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import train_test_split
from sklearn.datasets import load_iris
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import LeaveOneOut
from sklearn.metrics import precision_score, recall_score, f1_score
from sklearn.externals import joblib
import serial
import time
import madgwickahrs
import subprocess

ser = serial.Serial('COM5', 500000)
mad = madgwickahrs.MadgwickAHRS()

timestamp = []
acc = []
gyr = []
mag = []
qua = []    
touch = []

data = ''
clf = joblib.load('clf.model')

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
    test = 0
    while True:
        # print (clk - start)
        s = ser.read()
        line = ""
        data = ''
        while (s != '\n'):
            line += s
            s = ser.read()
        
        # line = line.strip().split('-')
        if (clk - start) > 1.0:
            # print line
            temp = line.strip().split()
            # print temp
            if len(temp) == 8:
                mg = [float(temp[1])*math.pi/180.0, float(temp[2])*math.pi/180.0, float(temp[3])*math.pi/180.0]
                ma = [float(temp[4])/256.0, float(temp[5])/256.0, float(temp[6])/256.0]
                # mm = [temp[7], temp[8], temp[9]]

                # mad.update(mg, ma, mm)
                mad.update_imu(mg, ma)
                # print(q.q0, q.q1, q.q2, q.q3)
                qTemp = mad.quaternion
                accR = gravity_compensate(qTemp, ma)
                
                data += str(int(round(time.time() * 1000)))
                data += " "
                data += str(int(temp[0]))
                data += " "
                data += str(float(temp[1])*math.pi/180.0)
                data += " "
                data += str(float(temp[2])*math.pi/180.0)
                data += " "
                data += str(float(temp[3])*math.pi/180.0)
                data += " "
                data += str(float(accR[0]))
                data += " "
                data += str(float(accR[1]))
                data += " "
                data += str(float(accR[2]))
                data += " "
                data += str(qTemp[0])
                data += " "
                data += str(qTemp[1])
                data += " "
                data += str(qTemp[2])
                data += " "
                data += str(qTemp[3])
                data += " "
                data += str(int(temp[7]))
                # data += "\n"

                # print(data)
                result = predict(clf, data)
                if result == 1:
                    test += 1
                    print(test)
                # print(result)
        
        clk = time.time()


def caln_sequence(X):
    X = np.array(X)
    X_std = np.std(X)
    X_min = np.min(X)
    X_max = np.max(X)
    X_mean = np.mean(X)
    X_var = np.var(X)
    X_sc = np.mean((X - X_mean) ** 3) / pow(X_std, 3)
    X_ku = np.mean((X - X_mean) ** 4) / pow(X_std, 4)
    if (math.isnan(X_ku)):
        print 'data error'
        X_ku = 0
    return [X_mean, X_min, X_max, X_sc, X_ku]

imu_list = []

def qua_to_vec(q):
    return [float(2*(q[1]*q[3] - q[0]*q[2])), float(2*(q[0]*q[1] + q[2]*q[3])), float(2*(0.5 - q[1]**2 - q[2]**2))]

def predict(clf, imu):
    global imu_list

    if (len(imu_list) >= 10):
        imu_list.pop(0)
        imu_list.append(imu)
        data = []
        n = len(imu_list)
        for i in range(n):
            tags = imu_list[i].split()
            data.append([float(v) for v in tags])
        data = np.array(data).reshape(n, -1)
        
        gyr_x = data[:,2]
        gyr_y = data[:,3]
        gyr_z = data[:,4]
        acc_x = data[:,5]
        acc_y = data[:,6]
        acc_z = data[:,7]
        gra_x = np.zeros(n)
        gra_y = np.zeros(n)
        gra_z = np.zeros(n)
        for j in range(n):
            gra = qua_to_vec(data[j, 8:12])
            gra_x[j] = gra[0]
            gra_y[j] = gra[1]
            gra_z[j] = gra[2]
        feature = []
        feature.extend(caln_sequence(acc_x))
        feature.extend(caln_sequence(acc_y))
        feature.extend(caln_sequence(acc_z))
        feature.extend(caln_sequence(gra_x))
        feature.extend(caln_sequence(gra_y))
        feature.extend(caln_sequence(gra_z))
        feature.extend(caln_sequence(gyr_x))
        feature.extend(caln_sequence(gyr_y))
        feature.extend(caln_sequence(gyr_z))
        for v in feature:
            if math.isnan(v):
                return 0
        return clf.predict([feature])[0]
    else:
        imu_list.append(imu)
        return 0


if __name__ == "__main__":
    #info, data, name_set, features = input(4, 'index1')
    #train(info, data)
    #test(info, data, name_set, features)

    '''
    clf = joblib.load('clf.model')
    fin = open('./data/horizontal_gyz_index1/horizontal_gyz_index1_IMU.txt')
    lines = fin.readlines()
    cnt = 0
    for line in lines:
        line = line.strip('\n')
        if predict(clf, line) == 1:
            print line
    '''
    # pi= subprocess.Popen("C:/Users/pc/Desktop/lzp/System/x64/Release/System.exe",shell=True, stdout=subprocess.PIPE)

    clf = joblib.load('clf.model')
    serialRead()
