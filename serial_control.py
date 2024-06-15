import serial.tools.list_ports
import numpy as np
import time
import json
from math import atan, acos, sqrt, pow, pi

class SerialControl():
    def __init__(self):
        self.data = 0
        self.msg = []
        self.coms = []
        self.sync_try_count = 10
        self.SynchChannel = 0
        self.sync_check = {
            "sync": "check"
        }
        self.sync_ok = "sync_ok"
        self.stream_start = {
            "start": "ok"
        }
        self.trajectory_start = {
            "start": "ok"
        }
        self.stream_stop = {
            "stop": "ok"
        }
        self.XAxisData = []
        self.YAxisData = []

    def serial_com_list(self, gui):
        ports = serial.tools.list_ports.comports()
        self.coms = [com[0] for com in ports]
        gui.comboBox.clear()
        gui.comboBox.addItems(self.coms)

    def serial_connect(self, gui):
        try:
            self.ser.is_open
        except:
            self.ser = serial.Serial(gui.comboBox.currentText(), 115200, timeout=0.1)
            # self.ser = serial.Serial('COM10', 115200, timeout=0.1)
            time.sleep(1)
            # self.ser.baudrate = 9600
            # self.ser.port = gui.comboBox.currentText()
            # self.ser.timeout = 0.1

        try:
            if self.ser.is_open:
                print("Port is already opened")
                self.ser.status = True
            else:
                self.ser = serial.Serial(gui.comboBox.currentText(), 115200, timeout=0.1)
                # self.ser = serial.Serial('COM10', 115200, timeout=0.1)
                time.sleep(1)
                # self.ser.baudrate = 9600
                # self.ser.port = gui.comboBox.currentText()
                # self.ser.timeout = 0.01
                # self.ser.open()
                self.ser.status = True
        except:
            self.ser.status = False

    def serial_close(self):
        try:
            self.ser.is_open
            self.ser.close()
            self.ser.status = False
        except:
            self.ser.status = False

    def set_ref_time(self):
        if len(self.XAxisData) == 0:
            self.ref_time = time.perf_counter()
        else:
            self.ref_time = time.perf_counter() - self.XAxisData[len(self.XAxisData) - 1]

    def Xdata_stream(self):
        if len(self.XAxisData) == 0:
            self.XAxisData.append(0)
        else:
            self.XAxisData.append(time.perf_counter() - self.ref_time)

    def Ydata_stream(self):
        # self.YAxisData[0].append(self.msg[0])
        for ChNumber in range(self.SynchChannel):
            self.YAxisData[ChNumber].append(self.IntMsg[ChNumber])

    def adjust_data(self):
        lenXdata = len(self.XAxisData)
        if (self.XAxisData[lenXdata - 1] - self.XAxisData[0]) > 5:
            del self.XAxisData[0]
            for ydata in self.YAxisData:
                del ydata[0]

        x = np.array(self.XAxisData)
        self.XDisplay = np.linspace(x.min(), x.max(), len(x), endpoint=False)
        self.YDisplay = np.array(self.YAxisData)

    def test_serial_sync(self, signal):
        sentData = json.dumps(self.sync_check)
        sentData = sentData + '\n'
        self.threading = True
        count = 0
        while self.threading:
            try:
                self.ser.write(sentData.encode())
                self.RowMsg = self.ser.readline()
                self.decode_message()
                if self.sync_ok in self.msg:
                    self.SynchChannel = int(self.msg[1])
                    self.GenChannels()
                    self.buildYdata()
                    print(self.Channels, self.YAxisData)
                    self.threading = False
                    signal.finished.emit()
                    break
                if self.threading == False:
                    signal.finished.emit()
                    break
            except Exception as e:
                print(e)
            count += 1
            if self.threading == False:
                signal.finished.emit()
                break
            if count > self.sync_try_count:
                count = 0
                time.sleep(0.5)
                if self.threading == False:
                    signal.finished.emit()
                    break

    def serial_sync(self, signal, gui):
        sentData = json.dumps(self.sync_check)
        sentData = sentData + '\n'
        self.threading = True
        count = 0
        while self.threading:
            try:
                self.ser.write(sentData.encode())
                gui.label_3.setText("..Syncing..")
                gui.label_3.setStyleSheet("color: rgb(251, 84, 43)")
                self.RowMsg = self.ser.readline()
                self.decode_message()
                if self.sync_ok in self.msg:
                    gui.label_3.setText("OK")
                    gui.label_3.setStyleSheet("color: rgb(30, 215, 96)")
                    gui.pushButton_4.setEnabled(False)
                    # self.SynchChannel = int(self.msg[1])
                    # self.GenChannels()
                    # self.buildYdata()
                    # print(self.Channels, self.YAxisData)
                    self.threading = False
                    signal.finished.emit()
                    break
                if self.threading == False:
                    signal.finished.emit()
                    break
            except Exception as e:
                print(e)
            count += 1
            if self.threading == False:
                signal.finished.emit()
                break
            if count > self.sync_try_count:
                count = 0
                gui.label_3.setText("Failed")
                gui.label_3.setStyleSheet("color: rgb(192, 83, 79)")
                time.sleep(0.5)
                if self.threading == False:
                    signal.finished.emit()
                    break

    def sent_trajectory(self, gui):
        theta1 = []
        theta2 = []
        theta3 = []
        rad1 = []
        rad2 = []
        rad3 = []
        l1 = 0.125
        l2 = 0.15
        l3 = 0.18
        a = gui.inputMatrix_1.text().split(" ")
        b = gui.inputMatrix_2.text().split(" ")
        c = gui.inputMatrix_3.text().split(" ")
        # matrix_1 = [float(num) for num in a]
        # matrix_2 = [float(num) for num in b]
        # matrix_3 = [float(num) for num in c]
        x = [float(gui.x0.text()), float(gui.x1.text()), float(gui.x2.text()), float(gui.x3.text()),
             float(gui.x4.text()), float(gui.x5.text()), float(gui.x6.text())]
        y = [float(gui.y0.text()), float(gui.y1.text()), float(gui.y2.text()), float(gui.y3.text()),
             float(gui.y4.text()), float(gui.y5.text()), float(gui.y6.text())]
        z = [float(gui.z0.text()), float(gui.z1.text()), float(gui.z2.text()), float(gui.z3.text()),
             float(gui.z4.text()), float(gui.z5.text()), float(gui.z6.text())]

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
            # theta mapped value
            theta1.append(round(finaltheta1 + 150, 4))
            theta2.append(round(190 - finaltheta2, 4))
            theta3.append(round(150 - finaltheta3, 4))
            # radian
            rad1.append(calcTheta1)
            rad2.append(calcTheta2)
            rad3.append(calcTheta3)

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
            },
            "targetInRad": {
                "rad1": rad1,
                "rad2": rad2,
                "rad3": rad3
            },
            "matrixGain": {
                "satu": a,
                "dua": b,
                "tiga": c
            }
        }
        print(a)
        print(b)
        print(c)
        print(x)
        print(y)
        print(z)
        print(data)
        sentData = json.dumps(data)
        sentData = sentData + '\n'
        self.ser.write(sentData.encode())

    def serial_trajectory_start(self):
        sentData = json.dumps(self.trajectory_start)
        sentData = sentData + '\n'
        self.ser.write(sentData.encode())
        temp = self.ser.readline()
        temp = temp.decode('utf8')
        print(temp)
        if "start_ok" in temp:
            pass
        else:
            print("not ok")
    def reset_data(self):
        self.YAxisData = []
        self.msg = []

    def decode_message(self):
        temp = self.RowMsg.decode()
        if len(temp) > 0:
            if "#" in temp:
                self.msg = temp.split("#")
                del self.msg[0]
                print(self.msg)
                if self.msg[0] in "D":
                    self.messageLen = 0
                    self.messageLenCheck = 0
                    del self.msg[0]
                    del self.msg[len(self.msg) - 1]
                    self.messageLen = int(self.msg[len(self.msg) - 1])
                    del self.msg[len(self.msg) - 1]
                    for item in self.msg:
                        self.messageLenCheck += len(item)

    def IntMsgFunc(self):
        self.IntMsg = [int(msg) for msg in self.msg]

    def StreamDataCheck(self):
        self.StreamData = False
        if self.SynchChannel == len(self.msg):
            if self.messageLen == self.messageLenCheck:
                self.StreamData = True
                self.IntMsgFunc()

    def GenChannels(self):
        self.Channels = [f"Ch{ch}" for ch in range(self.SynchChannel)]

    def buildYdata(self):
        self.YAxisData = []
        for _ in range(self.SynchChannel):
            self.YAxisData.append([])

    def ClearData(self):
        self.RowMsg = ""
        self.msg = []
        self.YAxisData = []
        self.YAxisData = []

    def serial_stop(self):
        sentData = json.dumps(self.stream_stop)
        sentData = sentData + '\n'
        self.ser.write(sentData.encode())

    def serial_stream(self, signal, gui):
        self.threading = True
        sentData = json.dumps(self.stream_start)
        sentData = sentData + '\n'
        while self.threading:
            try:
                self.ser.write(sentData.encode())
                self.RowMsg = self.ser.readline()
                self.decode_message()
                # print(f"{self.messageLen}, {self.messageLenCheck}")
                self.StreamDataCheck()
                if self.StreamData:
                    self.set_ref_time()
                    break
            except Exception as e:
                print(e)
        # gui.up_chart()
        while self.threading:
            try:
                self.RowMsg = self.ser.readline()
                self.decode_message()
                self.StreamDataCheck()
                if self.StreamData:
                    self.Xdata_stream()
                    self.Ydata_stream()
                    # Yval = [Ys[len(self.XAxisData) - 1] for Ys in self.YAxisData]
                    self.adjust_data()
                    # print(f"X Len: {len(self.XAxisData)}, Xstart:{self.XAxisData[0]}  Xend : {self.XAxisData[len(self.XAxisData) - 1]}, Xrange: {self.XAxisData[len(self.XAxisData) - 1] - self.XAxisData[0]} Ydata len: {len(self.YAxisData[0])} Yval: : {Yval} ")
            except Exception as e:
                print(e)
        if not self.threading:
            signal.finished.emit()

    def x_data(self, gui):
        gui.chart.plot(gui.x, gui.y, dash_capstyle='projecting', linewidth=1)
