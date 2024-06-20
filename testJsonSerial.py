# import json
# import serial
# import time
# from math import atan, acos, sqrt, pow, pi, cos, sin
#
# theta1 = []
# theta2 = []
# theta3 = []
# l1 = 8
# l2 = 15
# l3 = 10
# value of x != 0
# xc = []
# yc = []
# zc = []
# x = [0.00349, 0.00349, 0.00349, 0.00349, 0.00349, 0.00349, 0.2]
# y = [-0.1999, -0.1999, -0.1999, 0.1999, 0.1999, 0.1999, 0]
# z = [0.15, 0.03, 0.15, 0.15, 0.03, 0.15, 0.1]
# x = 0.15
# y = 0.05
# z = 0.08
## Meter
# x = [0.15, 0.15, 0.15, 0.15, 0.15, 0.20, 0.20]
# y = [0.1, 0.1, -0.1, -0.1, 0.1, 0, 0]
# z = [0.08, 0.18, 0.18, 0.08, 0.08, 0.1, 0.1]

## centimete
# x = [15, 15, 15, 15, 15, 20, 20]
# y = [10, 10, -10, -10, 10, 0, 0]
# z = [8, 18, 18, 8, 8, 10, 10]
# xact = []
# yact = []
# zact = []

# for i in range(len(x)):
#     calcTheta1 = atan(y[i]/x[i])
#     calcTheta2 = atan((z[i]-l1)/sqrt(pow(x[i], 2) + pow(y[i], 2))) + acos((pow(l2, 2) + pow(x[i], 2) + pow(y[i], 2) + pow(z[i]-l1, 2) - pow(l3, 2)) / (2*l2 * sqrt(pow(x[i], 2) + pow(y[i], 2) + pow(z[i]-l1, 2))))
#     calcTheta3 = acos((pow(l3, 2) + pow(l2, 2) - pow(x[i], 2) - pow(y[i], 2) - pow(z[i]-l1, 2)) / (-2*l2*l3))
#
#     # xc.append(round(calcTheta1, 4))
#     # yc.append(round(((pi/2) - calcTheta2), 4))
#     # zc.append(round(calcTheta3, 4))
#
#     xfk = (l3 * cos(calcTheta2 + calcTheta3) + l2 * cos(calcTheta2)) * cos(calcTheta1)
#     yfk = (l3 * cos(calcTheta2 + calcTheta3) + l2 * cos(calcTheta2)) * sin(calcTheta1)
#     zfk = l3 * sin(calcTheta2 + calcTheta3) + l2 * sin(calcTheta2) + l1
#
#     # xact.append(round(xfk, 4))
#     # yact.append(round(yfk, 4))
#     # zact.append(round(zfk, 4))
#
#     xact.append(round(calcTheta1, 4))
#     yact.append(round(calcTheta2, 4))
#     zact.append(round(calcTheta3, 4))
#
#
#     finaltheta1 = calcTheta1 * 180 / pi
#     finaltheta2 = calcTheta2 * 180 / pi
#     finaltheta3 = calcTheta3 * 180 / pi
#    # append value
#     theta1.append(round(finaltheta1 + 150, 4))
#     theta2.append(round(240 - finaltheta2, 4))
#     theta3.append(round(150 - finaltheta3, 4))
#
# data = {
#     "satu": {
#         "theta1": theta1[0],
#         "theta2": theta2[0],
#         "theta3": theta3[0]
#     },
#     "dua": {
#         "theta1": theta1[1],
#         "theta2": theta2[1],
#         "theta3": theta3[1]
#     },
#     "tiga": {
#         "theta1": theta1[2],
#         "theta2": theta2[2],
#         "theta3": theta3[2]
#     },
#     "empat": {
#         "theta1": theta1[3],
#         "theta2": theta2[3],
#         "theta3": theta3[3]
#     },
#     "lima": {
#         "theta1": theta1[4],
#         "theta2": theta2[4],
#         "theta3": theta3[4]
#     },
#     "enam": {
#         "theta1": theta1[5],
#         "theta2": theta2[5],
#         "theta3": theta3[5]
#     },
#     "tujuh": {
#         "theta1": theta1[6],
#         "theta2": theta2[6],
#         "theta3": theta3[6]
#     }
# }
# sentData = json.dumps(data)
# sentData = sentData + '\n'
# print(sentData.encode())
# print(xact)
# print(yact)
# print(zact)
# s = serial.Serial('COM7', 115200)
# time.sleep(1)
# s.write(sentData.encode())
# time.sleep(1)
# s.close()

# import serial, time, json
# msg = []
# syncCheck = {
#     "start": "ok"
# }
# sentData = json.dumps(syncCheck)
# sentData = sentData + '\n'
# s = serial.Serial('COM10', 115200, timeout=0.1)
# time.sleep(2)
# s.write(sentData.encode())
# temp = s.readline()
# temp = temp.decode('utf8')
# msg.append(temp)
# print(msg)
# print(len(msg))
# del msg[0]
# print(msg)
# if "sync_ok\n" in temp:
#     print("abc")
# else:
#     print("cba")
# s.close()

## FORWARD KINEMATIC
# print(f"{x}, {y}, {z}")
# calcTheta1 = atan(y/x)
# calcTheta2 = atan((z-l1)/sqrt(pow(x, 2) + pow(y, 2))) + acos((pow(l2, 2) + pow(x, 2) + pow(y, 2) + pow(z-l1, 2) - pow(l3, 2)) / (2*l2 * sqrt(pow(x, 2) + pow(y, 2) + pow(z-l1, 2))))
# calcTheta3 = acos((pow(l3, 2) + pow(l2, 2) - pow(x, 2) - pow(y, 2) - pow(z-l1, 2)) / (-2*l2*l3))
# theta1 = calcTheta1 * 180 / pi
# theta2 = calcTheta2 * 180 / pi
# theta3 = calcTheta3 * 180 / pi
# xfk = (l3*cos(theta2-theta3) + l2*cos(theta2))*cos(theta1)
# yfk = (l3*cos(theta2-theta3) + l2*cos(theta2))*sin(theta1)
# zfk = l3*sin(theta2-theta3) + l2*sin(theta2) + l1
# print(f"{theta1}, {theta2}, {theta3}")
# print(f"{xfk}, {yfk}, {zfk}")

import json
import serial
import time
from math import atan, acos, sqrt, pow, pi

theta1 = []
theta2 = []
theta3 = []
rad1 = []
rad2 = []
rad3 = []
l1 = 0.125
l2 = 0.15
l3 = 0.18
# value of x != 0
# x = [0.15, 0.15, 0.15, 0.15, 0.15, 0.20, 0.20]
# y = [0.05, 0.05, 0.15, 0.15, 0.05, 0, 0]
# z = [0.08, 0.18, 0.18, 0.08, 0.08, 0.1, 0.1]
x = [0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.2]
y = [0.15, 0.15, 0.15, -0.15, -0.15, -0.15, 0]
z = [0.2, 0.04, 0.2, 0.2, 0.04, 0.2, 0.2]

# x = [0.15, 0.15, 0.0061, 0.0061, 0.0061, 0.0061, 0.0061]
# y = [0.15, 0, 0, 0, 0, 0, 0]
# z = [0.349, 0.349, 0.349, 0.349, 0.349, 0.349, 0.349]

a = [-0.0999999999999998, 0, 0, -0.170256693766065, 0, 0]
b = [0, -0.389653589745392, -0.279653610881055, 0, -0.127515221024406, -0.0107280701401606]
c = [0, -0.328416545719673, -0.417298516050250, 0, -0.0252016070810869, -0.111999712837656]


for i in range(len(x)):
    calcTheta1 = atan(y[i] / x[i])
    calcTheta2 = (atan((z[i] - l1) / sqrt(pow(x[i], 2) + pow(y[i], 2)))
                  + acos((pow(l2, 2) + pow(x[i], 2) + pow(y[i], 2) +
                          pow(z[i] - l1, 2) - pow(l3, 2)) / (
                                     2 * l2 * sqrt(pow(x[i], 2) + pow(y[i], 2) + pow(z[i] - l1, 2)))))
    calcTheta3 = acos((pow(l3, 2) + pow(l2, 2) - pow(x[i], 2) - pow(y[i], 2) - pow(z[i] - l1, 2)) / (-2 * l2 * l3))

    finaltheta1 = calcTheta1 * 180 / pi
    finaltheta2 = calcTheta2 * 180 / pi
    finaltheta3 = calcTheta3 * 180 / pi
    # append value
    theta1.append(round(finaltheta1 + 150, 4))
    theta2.append(round(190 - finaltheta2, 4))
    theta3.append(round(150 - finaltheta3, 4))

    rad1.append(round(calcTheta1, 4))
    rad2.append(round(calcTheta2, 4))
    rad3.append(round(calcTheta3, 4))

data = {
    "finalTheta": {
        "satu": {
            "theta1": theta1[0],
            "theta2": theta2[0],
            "theta3": theta3[0]
        },
        "dua": {
            "theta1": theta1[1],
            "theta2": theta2[1],
            "theta3": theta3[1]
        },
        "tiga": {
            "theta1": theta1[2],
            "theta2": theta2[2],
            "theta3": theta3[2]
        },
        "empat": {
            "theta1": theta1[3],
            "theta2": theta2[3],
            "theta3": theta3[3]
        },
        "lima": {
            "theta1": theta1[4],
            "theta2": theta2[4],
            "theta3": theta3[4]
        },
        "enam": {
            "theta1": theta1[5],
            "theta2": theta2[5],
            "theta3": theta3[5]
        },
        "tujuh": {
            "theta1": theta1[6],
            "theta2": theta2[6],
            "theta3": theta3[6]
        }
    }
}
data2 = {
    "matrixGain": {
        "satu": a,
        "dua": b,
        "tiga": c
    },
    "targetInRad": {
        "rad1": rad1,
        "rad2": rad2,
        "rad3": rad3
    }
}
sentData = json.dumps(data)
sentData = sentData + '\n'
print(sentData.encode())

sentData = json.dumps(data2)
sentData = sentData + '\n'
print(sentData.encode())
print(theta1)
print(theta2)
print(theta3)
s = serial.Serial('COM14', 115200)
time.sleep(2)
s.write(sentData.encode())
time.sleep(1)
s.close()
