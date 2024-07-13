import serial.tools.list_ports
import time
import json
from math import atan, acos, sqrt, pow, pi
from datetime import datetime
from pprint import pprint


class SerialControl:
    def __init__(self):
        self.open = False
        self.msg = []
        self.coms = []
        self.sync_check = {
            "sync": "check"
        }
        self.sync_ok = "sync_ok"

    def serial_com_list(self, gui):
        ports = serial.tools.list_ports.comports()
        self.coms = [com[0] for com in ports]
        gui.comboBox.clear()
        gui.comboBox.addItems(self.coms)

    def serial_connect(self, gui):
        if not self.open:
            self.ser = serial.Serial(gui.comboBox.currentText(), 115200, timeout=0.1)
            time.sleep(2)
            print(self.ser.is_open)
        else:
            self.ser.close()
        # try:
        #     self.ser.close
        #     self.ser.is_open
        # except:
        #     print("conn")
        #     self.ser = serial.Serial(gui.comboBox.currentText(), 115200, timeout=0.1)
        #     time.sleep(2)

        try:
            if self.ser.is_open:
                print("Port successfully opened")
                sendData = json.dumps(self.sync_check)
                sendData = sendData + '\n'
                self.ser.write(sendData.encode())
                tdata = self.ser.readline()
                a = tdata.decode()
                print(a)
                if a == "sync_ok\n":
                    self.ser.status = True
                    self.open = True
                else:
                    print("tes")
                    gui.msgBox.setIcon(self.msgBox.icon().Warning)
                    gui.msgBox.setText(f"Gagal terhubung ke port {self.comboBox.currentText()}")
                    gui.msgBox.exec()
                    self.open = False

            else:
                self.ser = serial.Serial(gui.comboBox.currentText(), 115200, timeout=0.1)
                time.sleep(1)
                self.ser.status = True
                self.open = True
        except:
            self.ser.status = False
            self.open = False

    def serial_close(self):
        try:
            self.ser.is_open
            self.ser.close()
            self.ser.status = False
            self.open = False
        except:
            self.ser.status = False
            self.open = False

    def sent_trajectory_gain(self, gui, action, *trajectory):
        theta1 = []
        theta2 = []
        theta3 = []
        rad1 = []
        rad2 = []
        rad3 = []
        l1 = 0.145
        l2 = 0.15
        l3 = 0.18
        a = gui.inputMatrix_1.text()
        b = gui.inputMatrix_2.text()
        c = gui.inputMatrix_3.text()
        if '\t' in a:
            a = a.split("\t")
        if '\t' in b:
            b = b.split("\t")
        if '\t' in c:
            c = c.split("\t")

        if ' ' in a:
            a = a.split(" ")
        if ' ' in b:
            b = b.split(" ")
        if ' ' in c:
            c = c.split(" ")

        x = []
        y = []
        z = []
        for xval, yval, zval in zip(trajectory[0], trajectory[1], trajectory[2]):
            x.append(float(xval.text()))
            y.append(float(yval.text()))
            z.append(float(zval.text()))

        for i in range(len(x)):
            calcTheta1 = atan(y[i] / x[i])
            calcTheta2 = (atan((z[i] - l1) / sqrt(pow(x[i], 2) + pow(y[i], 2)))
                          + acos((pow(l2, 2) + pow(x[i], 2) + pow(y[i], 2) +
                                  pow(z[i] - l1, 2) - pow(l3, 2)) / (
                                         2 * l2 * sqrt(pow(x[i], 2) + pow(y[i], 2) + pow(z[i] - l1, 2)))))
            calcTheta3 = acos(
                (pow(l3, 2) + pow(l2, 2) - pow(x[i], 2) - pow(y[i], 2) - pow(z[i] - l1, 2)) / (-2 * l2 * l3))

            finaltheta1 = calcTheta1 * 180 / pi
            finaltheta2 = calcTheta2 * 180 / pi
            finaltheta3 = calcTheta3 * 180 / pi
            # theta mapping
            theta1.append(round(finaltheta1 + 150, 2))
            theta2.append(round(190 - finaltheta2, 2))
            theta3.append(round(150 - finaltheta3, 2))
            # radian
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
        if not gui.checkBox_2.isChecked():
            data.update({"gripper": {"buka": action[0], "tutup": action[1]}})
        pprint(data, sort_dicts=False, indent=4, width=100)
        sentData = json.dumps(data)
        sentData = sentData + '\n'
        self.ser.write(sentData.encode())
        temp = self.ser.readline()
        temp = temp.decode()
        print(temp)
        if "data ok" in temp:
            pass
        else:
            print("data not ok")


    def serial_log_print(self, signal):
        self.a = True
        signal.progress1.emit("Iterasi\tWaktu\tTorsi 1\tTorsi 2\tTorsi 3")
        signal.progress2.emit("Iterasi\tWaktu\tX\tY\tZ")
        count = 1
        while self.a:
            tdata = self.ser.readline()
            if tdata:
                dt = datetime.now().strftime("%H:%M:%S")
                data = json.loads(tdata)
                signal.progress1.emit(f"{count}\t{dt}\t{data.get('satu')}%\t{data.get('dua')}%\t{data.get('tiga')}%")
                data_torque = [data.get('t1'), data.get('t2'), data.get('t3')]
                signal.progress1_data.emit(data_torque)
                signal.progress2.emit(f"{count}\t{dt}\t{data.get('fk1')}\t{data.get('fk2')}\t{data.get('fk3')}")
                data_trajectory = [data.get('fk1'), data.get('fk2'), data.get('fk3')]
                signal.progress2_data.emit(data_trajectory)
                count += 1
                if "done" in data:
                    print("data ended")
                    self.a = False
                    signal.progress1.emit("=============================================")
                    signal.progress2.emit("=============================================")
                    signal.finished.emit()
                    del count
