import madgwick
import math
from matplotlib import pyplot as plt
import numpy as np
import scipy.signal as signal

q = madgwick.Madgwick()

ans = []
timestamp = []
touch = []

def draw_points(timestamp, points, touch):
    n = len(timestamp)
    x = np.array([points[3 * i] for i in range(n)])
    y = np.array([points[3 * i + 1] for i in range(n)])
    z = np.array([points[3 * i + 2] for i in range(n)])
    plt.plot(timestamp, x, timestamp, y, timestamp, z, timestamp, touch)

def draw_peaks(timestamp, points):
    n = len(timestamp)
    x = np.array([points[3 * i] for i in range(n)])
    y = np.array([points[3 * i + 1] for i in range(n)])
    z = np.array([points[3 * i + 2] for i in range(n)])
    peaks, _ = signal.find_peaks(abs(x), height=1000, distance=5, threshold=5)
    plt.plot(timestamp[peaks], x[peaks], '.')
    peaks, _ = signal.find_peaks(abs(y), height=1000, distance=5, threshold=5)
    plt.plot(timestamp[peaks], y[peaks], '.')
    peaks, _ = signal.find_peaks(abs(z), height=1000, distance=5, threshold=5)
    plt.plot(timestamp[peaks], z[peaks], '.')

def gravity_compensate(q, acc):
    g = [0.0, 0.0, 0.0]

    # get expected direction of gravity
    g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
    g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
    g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]

    # compensate accelerometer readings with the expected direction of gravity
    return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]

if __name__ == '__main__':
    file = 'hard.log'
    lines = open(file, 'r')
    for line in lines:
        data = line.strip().split()
        if (len(data) == 11):
            timestamp.append(data[0])
            #print(u"Acc [g]")
            #print(data[4] + ' ' + data[5] + ' ' + data[6])
            #print(u"Gyr [deg/s]")
            #print(data[1] + ' ' + data[2] + ' ' + data[3])
            #print(u"Mag [G]")
            #print(data[7] + ' ' + data[8] + ' ' + data[9])
            touch.append(data[10])
            q.MadgwickAHRSupdate(
                gx = float(data[1])*math.pi/180.0,
                gy = float(data[2])*math.pi/180.0,
                gz = float(data[3])*math.pi/180.0,
                ax = float(data[4])/256.0, 
                ay = float(data[5])/256.0,
                az = float(data[6])/256.0,
                mx = float(data[7])/10.0,
                my = float(data[8])/10.0,
                mz = float(data[9])/10.0
            )
            #print(q.q0, q.q1, q.q2, q.q3)
            qTemp = [q.q0, q.q1, q.q2, q.q3]
            acc = [float(data[4])/256.0, float(data[5])/256.0, float(data[6])/256.0]
            accR = gravity_compensate(qTemp, acc)
            for i in accR:
                ans.append(i)
            #print(u"Acc Real")
            #print(accR)
            #print("")

    plt.figure(file)

    plt.subplot(1, 1, 1)
    plt.title('real acc')
    draw_points(timestamp, ans, touch)
    plt.show()
    # draw_peaks(timestamp, ans)    
