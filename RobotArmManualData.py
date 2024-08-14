import json
from math import atan, acos, sqrt, pow, pi

theta1 = []
theta2 = []
theta3 = []
rad1 = []
rad2 = []
rad3 = []
l1 = 0.133
l2 = 0.15
l3 = 0.18
# l3 = 0.2

# ubah sesuai kebutuhan, semua koordinat harus memiliki jumlah data yang sama
# minimal 1 s.d maksimum 7
x1 = [12, 12, 12, 12, 12, 12, 20]
y2 = [-12, -12, -12, 12, 12, 12, 0]
z3 = [25, 3.5, 25, 25, 3.5, 25, 20]

# temp
x = []
y = []
z = []

# ganti dengan nilai matrix gain hasil dari aplikasi Kalkulasi LQR
a = "0.7255	-0.0000	0.0000	0.4093	-0.0000	0.0000"
b = "0.0000	0.4511	-0.0611	0.0000	0.5674	0.0584"
c = "-0.0000	-0.3128	0.0579	0.0000	-0.2775	0.1321"

# iterasi buka tutup gripper, ubah sesuai kebutuhan (1 s.d 7) tidak boleh sama
# komen keempat line dibawah ini jika tidak menggunakan gripper
gripper = {
    "buka": 5,
    "tutup": 2
}

# proses inverse kinematic
if '\t' in a:
    a = a.split("\t")
if '\t' in b:
    b = b.split("\t")
if '\t' in c:
    c = c.split("\t")
for xval, yval, zval in zip(x1, y2, z3):
    x.append(xval * 0.01)
    y.append(yval * 0.01)
    z.append(zval * 0.01)

for i in range(len(x)):
    calcTheta1 = atan(y[i] / x[i])
    calcTheta2 = (atan((z[i] - l1) / sqrt(pow(x[i], 2) + pow(y[i], 2)))
                  + acos((pow(l2, 2) + pow(x[i], 2) + pow(y[i], 2) +
                          pow(z[i] - l1, 2) - pow(l3, 2)) / (
                                 2 * l2 * sqrt(pow(x[i], 2) + pow(y[i], 2) + pow(z[i] - l1, 2)))))
    calcTheta3 = pi - acos((pow(l3, 2) + pow(l2, 2) - pow(x[i], 2) - pow(y[i], 2) - pow(z[i] - l1, 2)) / (2 * l2 * l3))

    finaltheta1 = calcTheta1 * 180 / pi
    finaltheta2 = calcTheta2 * 180 / pi
    finaltheta3 = calcTheta3 * 180 / pi
    # append value
    theta1.append(round(finaltheta1 + 150, 2))
    theta2.append(round(190 - finaltheta2, 2))
    theta3.append(round(150 - finaltheta3, 2))

    rad1.append(round(calcTheta1, 4))
    rad2.append(round(calcTheta2, 4))
    rad3.append(round(calcTheta3, 4))

data = {
    "theta": {
        "satu": theta1,
        "dua": theta2,
        "tiga": theta3
    },
    "thetaLen": len(theta1),
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
if gripper:
    data.update({"gripper": gripper})
sentData = json.dumps(data)
print("data tanpa newline")
print(sentData)
sentData = sentData + '\n'
print("data dengan newline")
print(sentData.encode())
