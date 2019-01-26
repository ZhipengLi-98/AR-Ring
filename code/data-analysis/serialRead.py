import serial

ser = serial.Serial('COM3', 500000)
cnt = 0
lines = []
while(cnt < 10):
    cnt += 1
    s = ser.read()
    line = []
    while(s != '\n'):
        line.append(s)
        s = ser.read()
    lines.append(line)

for i in range(0, 10):
    print(lines[i])