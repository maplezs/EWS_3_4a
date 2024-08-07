import json
import sys
import re
import webbrowser
import os
import csv
from datetime import datetime
from math import floor, log10, inf
from PyQt6.QtCore import (QCoreApplication, QMetaObject, QObject, QRect, pyqtSignal, QThread)
from PyQt6.QtGui import QAction, QResizeEvent, QIcon, QPixmap
from PyQt6.QtWidgets import (QApplication, QComboBox, QGroupBox, QLabel, QFileDialog,
                             QLineEdit, QMainWindow, QMenu, QMenuBar, QCheckBox,
                             QPushButton, QMessageBox, QStatusBar, QWidget, QPlainTextEdit, QVBoxLayout)
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.backends.backend_qtagg import \
    NavigationToolbar2QT as NavigationToolbar
from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.figure import Figure
from serial_control import SerialControl


def zeros_count(decimal):
    return inf if decimal == 0 else -floor(log10(abs(decimal))) - 1


def openManual():
    webbrowser.open('https://github.com/maplezs/EWS_3_4A/blob/main/README.md', new=0)


class Ui_MainWindow():
    def __init__(self, serial, MainWindow):
        super().__init__()
        self.mainWindow = MainWindow
        self.data_torque = None
        self.data_torque_satu = []
        self.data_torque_dua = []
        self.data_torque_tiga = []
        self.dataPercentageTorqueOne = []
        self.dataPercentageTorqueTwo = []
        self.dataPercentageTorqueThree = []
        self.timeTorque = []
        self.data_trajectory = None
        self.data_trajectory_x = []
        self.data_trajectory_y = []
        self.data_trajectory_z = []
        self.timeTrajectory = []
        self.list_x_enabled = []
        self.list_y_enabled = []
        self.list_z_enabled = []
        self.list_x_disabled = []
        self.list_y_disabled = []
        self.list_z_disabled = []
        self.plot_windows = list()
        self.xAxisRange = 7
        self.serial = serial

    def setupUi(self):
        if not self.mainWindow.objectName():
            self.mainWindow.setObjectName(u"self.mainWindow")
        self.mainWindow.setMinimumSize(359, 158)
        self.mainWindow.resize(359, 158)
        self.mainWindow.setMaximumSize(359, 158)
        self.mainWindow.setWindowIcon(QIcon("icon-app.png"))
        self.actionTentang = QAction(self.mainWindow)
        self.actionTentang.setObjectName(u"actionTentang")
        self.actionTentang_2 = QAction(self.mainWindow)
        self.actionTentang_2.setObjectName(u"actionTentang_2")
        self.actionKeluar = QAction(self.mainWindow)
        self.actionKeluar.setObjectName(u"actionKeluar")
        self.centralwidget = QWidget(self.mainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(10, 0, 341, 111))
        self.pushButton = QPushButton(self.groupBox)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(260, 30, 75, 31))
        self.pushButton_2 = QPushButton(self.groupBox)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(170, 60, 71, 31))
        self.comboBox = QComboBox(self.groupBox)
        self.comboBox.setObjectName(u"comboBox")
        self.comboBox.setGeometry(QRect(170, 30, 71, 22))
        self.label = QLabel(self.groupBox)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(50, 20, 81, 31))
        self.widget_2 = QWidget(self.centralwidget)
        self.widget_2.setObjectName(u"widget_2")
        self.widget_2.setGeometry(QRect(0, 180, 971, 251))
        self.groupBox_4 = QGroupBox(self.widget_2)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.groupBox_4.setGeometry(QRect(10, 30, 481, 171))
        # X
        self.x0 = QLineEdit(self.groupBox_4)
        self.x0.setObjectName(u"x0")
        self.x0.setGeometry(QRect(50, 50, 51, 22))
        self.x1 = QLineEdit(self.groupBox_4)
        self.x1.setObjectName(u"x1")
        self.x1.setGeometry(QRect(110, 50, 51, 22))
        self.x2 = QLineEdit(self.groupBox_4)
        self.x2.setObjectName(u"x2")
        self.x2.setGeometry(QRect(170, 50, 51, 22))
        self.x3 = QLineEdit(self.groupBox_4)
        self.x3.setObjectName(u"x3")
        self.x3.setGeometry(QRect(230, 50, 51, 22))
        self.x4 = QLineEdit(self.groupBox_4)
        self.x4.setObjectName(u"x4")
        self.x4.setGeometry(QRect(290, 50, 51, 22))
        self.x5 = QLineEdit(self.groupBox_4)
        self.x5.setObjectName(u"x5")
        self.x5.setGeometry(QRect(350, 50, 51, 22))
        self.x6 = QLineEdit(self.groupBox_4)
        self.x6.setObjectName(u"x6")
        self.x6.setGeometry(QRect(410, 50, 51, 22))
        # Y
        self.y0 = QLineEdit(self.groupBox_4)
        self.y0.setObjectName(u"y0")
        self.y0.setGeometry(QRect(50, 80, 51, 22))
        self.y1 = QLineEdit(self.groupBox_4)
        self.y1.setObjectName(u"y1")
        self.y1.setGeometry(QRect(110, 80, 51, 22))
        self.y2 = QLineEdit(self.groupBox_4)
        self.y2.setObjectName(u"y2")
        self.y2.setGeometry(QRect(170, 80, 51, 22))
        self.y3 = QLineEdit(self.groupBox_4)
        self.y3.setObjectName(u"y3")
        self.y3.setGeometry(QRect(230, 80, 51, 22))
        self.y4 = QLineEdit(self.groupBox_4)
        self.y4.setObjectName(u"y4")
        self.y4.setGeometry(QRect(290, 80, 51, 22))
        self.y5 = QLineEdit(self.groupBox_4)
        self.y5.setObjectName(u"y5")
        self.y5.setGeometry(QRect(350, 80, 51, 22))
        self.y6 = QLineEdit(self.groupBox_4)
        self.y6.setObjectName(u"y6")
        self.y6.setGeometry(QRect(410, 80, 51, 22))
        # Z
        self.z0 = QLineEdit(self.groupBox_4)
        self.z0.setObjectName(u"z0")
        self.z0.setGeometry(QRect(50, 110, 51, 22))
        self.z1 = QLineEdit(self.groupBox_4)
        self.z1.setObjectName(u"z1")
        self.z1.setGeometry(QRect(110, 110, 51, 22))
        self.z2 = QLineEdit(self.groupBox_4)
        self.z2.setObjectName(u"z2")
        self.z2.setGeometry(QRect(170, 110, 51, 22))
        self.z3 = QLineEdit(self.groupBox_4)
        self.z3.setObjectName(u"z3")
        self.z3.setGeometry(QRect(230, 110, 51, 22))
        self.z4 = QLineEdit(self.groupBox_4)
        self.z4.setObjectName(u"z4")
        self.z4.setGeometry(QRect(290, 110, 51, 22))
        self.z5 = QLineEdit(self.groupBox_4)
        self.z5.setObjectName(u"z5")
        self.z5.setGeometry(QRect(350, 110, 51, 22))
        self.z6 = QLineEdit(self.groupBox_4)
        self.z6.setObjectName(u"z6")
        self.z6.setGeometry(QRect(410, 110, 51, 22))
        self.label_4 = QLabel(self.groupBox_4)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(70, 30, 16, 16))
        self.label_6 = QLabel(self.groupBox_4)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(130, 30, 16, 16))
        self.label_7 = QLabel(self.groupBox_4)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(190, 30, 16, 16))
        self.label_8 = QLabel(self.groupBox_4)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(250, 30, 16, 16))
        self.label_9 = QLabel(self.groupBox_4)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(310, 30, 16, 16))
        self.label_10 = QLabel(self.groupBox_4)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(30, 50, 16, 16))
        self.label_11 = QLabel(self.groupBox_4)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setGeometry(QRect(30, 80, 16, 16))
        self.label_12 = QLabel(self.groupBox_4)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setGeometry(QRect(30, 110, 16, 16))
        self.label_13 = QLabel(self.groupBox_4)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setGeometry(QRect(370, 30, 16, 16))
        self.label_14 = QLabel(self.groupBox_4)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setGeometry(QRect(430, 30, 16, 16))
        self.groupBox_3 = QGroupBox(self.widget_2)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.groupBox_3.setGeometry(QRect(490, 30, 471, 171))
        self.inputMatrix_1 = QLineEdit(self.groupBox_3)
        self.inputMatrix_1.setObjectName(u"inputMatrix_1")
        self.inputMatrix_1.setGeometry(QRect(40, 40, 421, 22))
        self.inputMatrix_2 = QLineEdit(self.groupBox_3)
        self.inputMatrix_2.setObjectName(u"inputMatrix_2")
        self.inputMatrix_2.setGeometry(QRect(40, 70, 421, 22))
        self.inputMatrix_3 = QLineEdit(self.groupBox_3)
        self.inputMatrix_3.setObjectName(u"inputMatrix_3")
        self.inputMatrix_3.setGeometry(QRect(40, 100, 421, 22))
        self.labelMatrix_1 = QLabel(self.groupBox_3)
        self.labelMatrix_1.setObjectName(u"labelMatrix_1")
        self.labelMatrix_1.setGeometry(QRect(20, 40, 16, 16))
        self.labelMatrix_2 = QLabel(self.groupBox_3)
        self.labelMatrix_2.setObjectName(u"labelMatrix_2")
        self.labelMatrix_2.setGeometry(QRect(20, 70, 31, 16))
        self.labelMatrix_3 = QLabel(self.groupBox_3)
        self.labelMatrix_3.setObjectName(u"labelMatrix_3")
        self.labelMatrix_3.setGeometry(QRect(20, 100, 16, 16))
        self.pushButton_5 = QPushButton(self.widget_2)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setGeometry(QRect(450, 210, 75, 31))
        self.groupBox_7 = QGroupBox(self.centralwidget)
        self.groupBox_7.setObjectName(u"groupBox_7")
        self.groupBox_7.setGeometry(QRect(10, 410, 401, 211))
        self.plainTextEdit = QPlainTextEdit(self.groupBox_7)
        self.plainTextEdit.setObjectName(u"plainTextEdit")
        self.plainTextEdit.setGeometry(QRect(10, 20, 381, 181))
        self.groupBox_5 = QGroupBox(self.centralwidget)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.groupBox_5.setGeometry(QRect(350, 0, 241, 211))
        self.pushButton_6 = QPushButton(self.groupBox_5)
        self.pushButton_6.setObjectName(u"pushButton_6")
        self.pushButton_6.setGeometry(QRect(20, 60, 75, 24))
        self.pushButton_7 = QPushButton(self.groupBox_5)
        self.pushButton_7.setObjectName(u"pushButton_7")
        self.pushButton_7.setGeometry(QRect(150, 60, 75, 24))
        self.pushButton_10 = QPushButton(self.groupBox_5)
        self.pushButton_10.setObjectName(u"pushButton_10")
        self.pushButton_10.setGeometry(QRect(80, 140, 75, 24))
        self.label_20 = QLabel(self.groupBox_5)
        self.label_20.setObjectName(u"label_20")
        self.label_20.setGeometry(QRect(80, 110, 71, 31))
        self.groupBox_8 = QGroupBox(self.centralwidget)
        self.groupBox_8.setObjectName(u"groupBox_8")
        self.groupBox_8.setGeometry(QRect(550, 410, 411, 211))
        self.plainTextEdit_2 = QPlainTextEdit(self.groupBox_8)
        self.plainTextEdit_2.setObjectName(u"plainTextEdit_2")
        self.plainTextEdit_2.setGeometry(QRect(10, 20, 391, 181))
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.groupBox_6.setGeometry(QRect(590, 0, 371, 211))
        self.groupBox_9 = QGroupBox(self.groupBox_6)
        self.groupBox_9.setObjectName(u"groupBox_9")
        self.groupBox_9.setGeometry(QRect(20, 30, 131, 171))
        self.comboBox_2 = QComboBox(self.groupBox_9)
        self.comboBox_2.addItem("1", [1, "tutup"])
        self.comboBox_2.addItem("2", [2, "tutup"])
        self.comboBox_2.addItem("3", [3, "tutup"])
        self.comboBox_2.addItem("4", [4, "tutup"])
        self.comboBox_2.addItem("5", [5, "tutup"])
        self.comboBox_2.addItem("6", [6, "tutup"])
        self.comboBox_2.addItem("7", [7, "tutup"])
        self.comboBox_2.setObjectName(u"comboBox_2")
        self.comboBox_2.setGeometry(QRect(20, 40, 91, 22))
        self.label_5 = QLabel(self.groupBox_9)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(40, 90, 31, 16))
        self.label_15 = QLabel(self.groupBox_9)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(40, 20, 31, 16))
        self.comboBox_4 = QComboBox(self.groupBox_9)
        self.comboBox_4.addItem("1", [1, "buka"])
        self.comboBox_4.addItem("2", [2, "buka"])
        self.comboBox_4.addItem("3", [3, "buka"])
        self.comboBox_4.addItem("4", [4, "buka"])
        self.comboBox_4.addItem("5", [5, "buka"])
        self.comboBox_4.addItem("6", [6, "buka"])
        self.comboBox_4.addItem("7", [7, "buka"])
        self.comboBox_4.setObjectName(u"comboBox_4")
        self.comboBox_4.setObjectName(u"comboBox_4")
        self.comboBox_4.setGeometry(QRect(20, 110, 91, 22))
        self.checkBox_2 = QCheckBox(self.groupBox_9)
        self.checkBox_2.setObjectName(u"checkBox_2")
        self.checkBox_2.setGeometry(QRect(20, 140, 101, 20))
        self.groupBox_10 = QGroupBox(self.groupBox_6)
        self.groupBox_10.setObjectName(u"groupBox_10")
        self.groupBox_10.setGeometry(QRect(240, 30, 111, 171))
        self.pushButton_8 = QPushButton(self.groupBox_10)
        self.pushButton_8.setObjectName(u"pushButton_8")
        self.pushButton_8.setGeometry(QRect(20, 40, 75, 24))
        self.pushButton_9 = QPushButton(self.groupBox_10)
        self.pushButton_9.setObjectName(u"pushButton_9")
        self.pushButton_9.setGeometry(QRect(20, 110, 75, 24))
        self.label_18 = QLabel(self.groupBox_10)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setGeometry(QRect(30, 20, 51, 16))
        self.label_19 = QLabel(self.groupBox_10)
        self.label_19.setObjectName(u"label_19")
        self.label_19.setGeometry(QRect(20, 90, 71, 16))
        self.label_16 = QLabel(self.groupBox_6)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setGeometry(QRect(152, 70, 91, 41))
        self.label_17 = QLabel(self.groupBox_6)
        self.label_17.setObjectName(u"label_17")
        self.label_17.setGeometry(QRect(190, 100, 20, 21))
        self.pushButton_3 = QPushButton(self.groupBox_6)
        self.pushButton_3.setObjectName(u"pushButton_3")
        self.pushButton_3.setGeometry(QRect(160, 120, 31, 31))
        self.pushButton_4 = QPushButton(self.groupBox_6)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(200, 120, 31, 31))
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(410, 440, 141, 151))
        self.logo = QPixmap("logo100crop.png")
        self.mainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self.mainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 616, 22))
        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuBantuan = QMenu(self.menubar)
        self.menuBantuan.setObjectName(u"menuBantuan")
        self.mainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self.mainWindow)
        self.statusbar.setObjectName(u"statusbar")
        self.mainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuBantuan.menuAction())
        self.menuFile.addAction(self.actionKeluar)
        self.menuBantuan.addAction(self.actionTentang)
        self.menuBantuan.addAction(self.actionTentang_2)

        self.msgBox = QMessageBox()

        self.retranslateUi()
        self.configureWidget()

        QMetaObject.connectSlotsByName(self.mainWindow)

    def retranslateUi(self):
        self.mainWindow.setWindowTitle(QCoreApplication.translate("self.mainWindow", u"Robot Arm Control App", None))
        self.actionTentang.setText(QCoreApplication.translate("self.mainWindow", u"Manual", None))
        self.actionTentang_2.setText(QCoreApplication.translate("self.mainWindow", u"About", None))
        self.actionKeluar.setText(QCoreApplication.translate("self.mainWindow", u"Exit", None))
        self.groupBox.setTitle(QCoreApplication.translate("self.mainWindow", u"Port Settings", None))
        self.pushButton.setText(QCoreApplication.translate("self.mainWindow", u"Refresh", None))
        self.pushButton_2.setText(QCoreApplication.translate("self.mainWindow", u"Connect", None))
        self.label.setText(QCoreApplication.translate("self.mainWindow", u"Available port:", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("self.mainWindow", u"Waypoint Trajectory", None))
        self.label_4.setText(QCoreApplication.translate("self.mainWindow", u"1", None))
        self.label_6.setText(QCoreApplication.translate("self.mainWindow", u"2", None))
        self.label_7.setText(QCoreApplication.translate("self.mainWindow", u"3", None))
        self.label_8.setText(QCoreApplication.translate("self.mainWindow", u"4", None))
        self.label_9.setText(QCoreApplication.translate("self.mainWindow", u"5", None))
        self.label_10.setText(QCoreApplication.translate("self.mainWindow", u"X", None))
        self.label_11.setText(QCoreApplication.translate("self.mainWindow", u"Y", None))
        self.label_12.setText(QCoreApplication.translate("self.mainWindow", u"Z", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("self.mainWindow", u"Matrix K (Gain)", None))
        self.label_13.setText(QCoreApplication.translate("self.mainWindow", u"6", None))
        self.label_14.setText(QCoreApplication.translate("self.mainWindow", u"7", None))
        self.labelMatrix_1.setText(QCoreApplication.translate("self.mainWindow", u"1", None))
        self.labelMatrix_2.setText(QCoreApplication.translate("self.mainWindow", u"2", None))
        self.labelMatrix_3.setText(QCoreApplication.translate("self.mainWindow", u"3", None))
        self.pushButton_5.setText(QCoreApplication.translate("self.mainWindow", u"Send", None))
        self.menuFile.setTitle(QCoreApplication.translate("self.mainWindow", u"File", None))
        self.menuBantuan.setTitle(QCoreApplication.translate("self.mainWindow", u"Help", None))
        self.groupBox_7.setTitle(QCoreApplication.translate("self.mainWindow", u"Torque Log", None))
        self.groupBox_5.setTitle(QCoreApplication.translate("self.mainWindow", u"Data Configuration", None))
        self.pushButton_6.setText(QCoreApplication.translate("self.mainWindow", u"Save", None))
        self.pushButton_7.setText(QCoreApplication.translate("self.mainWindow", u"Open", None))
        self.pushButton_10.setText(QCoreApplication.translate("MainWindow", u"Save log", None))
        self.label_20.setText(
            QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Save log data</p></body></html>", None))
        self.groupBox_8.setTitle(QCoreApplication.translate("self.mainWindow", u"Trajectory Log", None))
        self.groupBox_6.setTitle(QCoreApplication.translate("self.mainWindow", u"Misc Configuration", None))
        self.groupBox_9.setTitle(QCoreApplication.translate("self.mainWindow", u"Servo Gripper", None))
        self.comboBox_2.setItemText(0, QCoreApplication.translate("self.mainWindow", u"1st Iteration", None))
        self.comboBox_2.setItemText(1, QCoreApplication.translate("self.mainWindow", u"2nd Iteration", None))
        self.comboBox_2.setItemText(2, QCoreApplication.translate("self.mainWindow", u"3rd Iteration", None))
        self.comboBox_2.setItemText(3, QCoreApplication.translate("self.mainWindow", u"4th Iteration", None))
        self.comboBox_2.setItemText(4, QCoreApplication.translate("self.mainWindow", u"5th Iteration", None))
        self.comboBox_2.setItemText(5, QCoreApplication.translate("self.mainWindow", u"6th Iteration", None))
        self.comboBox_2.setItemText(6, QCoreApplication.translate("self.mainWindow", u"7th Iteration", None))
        self.label_5.setText(QCoreApplication.translate("self.mainWindow", u"Open", None))
        self.label_15.setText(QCoreApplication.translate("self.mainWindow", u"Shut", None))
        self.comboBox_4.setItemText(0, QCoreApplication.translate("self.mainWindow", u"1st Iteration", None))
        self.comboBox_4.setItemText(1, QCoreApplication.translate("self.mainWindow", u"2nd Iteration", None))
        self.comboBox_4.setItemText(2, QCoreApplication.translate("self.mainWindow", u"3rd Iteration", None))
        self.comboBox_4.setItemText(3, QCoreApplication.translate("self.mainWindow", u"4th Iteration", None))
        self.comboBox_4.setItemText(4, QCoreApplication.translate("self.mainWindow", u"5th Iteration", None))
        self.comboBox_4.setItemText(5, QCoreApplication.translate("self.mainWindow", u"6th Iteration", None))
        self.comboBox_4.setItemText(6, QCoreApplication.translate("self.mainWindow", u"7th Iteration", None))
        self.groupBox_10.setTitle(QCoreApplication.translate("self.mainWindow", u"Plotting", None))
        self.pushButton_8.setText(QCoreApplication.translate("self.mainWindow", u"Plot", None))
        self.pushButton_9.setText(QCoreApplication.translate("self.mainWindow", u"Plot", None))
        self.label_18.setText(QCoreApplication.translate("self.mainWindow", u"Torque", None))
        self.label_19.setText(QCoreApplication.translate("self.mainWindow", u"Trajectory", None))
        self.label_16.setText(
            QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Num of Iteration</p></body></html>", None))
        self.label_17.setText(
            QCoreApplication.translate("MainWindow", u"<html><head/><body><p>7</p></body></html>", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"+", None))
        self.checkBox_2.setText(QCoreApplication.translate("self.mainWindow", u"No Gripper", None))
        self.label_2.setPixmap(self.logo)

    def configureWidget(self):
        self.actionKeluar.setShortcut("Ctrl+Q")
        self.actionKeluar.triggered.connect(self.mainWindow.close)
        self.actionTentang.triggered.connect(lambda: openManual())
        self.actionTentang_2.triggered.connect(lambda: self.openAboutWindow())
        self.pushButton.clicked.connect(lambda: self.serial.serialListPort(self))
        self.pushButton_2.clicked.connect(lambda: self.serialConnect())
        self.pushButton_3.clicked.connect(lambda: self.kurangiIterasi())
        self.pushButton_4.clicked.connect(lambda: self.tambahIterasi())
        self.pushButton_5.clicked.connect(lambda: self.sendData())
        self.pushButton_6.clicked.connect(lambda: self.saveFile())
        self.pushButton_7.clicked.connect(lambda: self.openFile())
        self.pushButton_8.clicked.connect(lambda: self.plotWindow("Torsi", self.data_torque))
        self.pushButton_9.clicked.connect(lambda: self.plotWindow("Trajektori", self.data_trajectory_input,
                                                                  self.data_trajectory))
        self.pushButton_10.clicked.connect(lambda: self.saveCSVFile())

        self.checkBox_2.stateChanged.connect(lambda: self.checkBoxToogle())
        self.plainTextEdit.setReadOnly(True)
        self.plainTextEdit_2.setReadOnly(True)
        # initial port listing
        self.serial.serialListPort(self)
        self.list_x = [self.x0, self.x1, self.x2, self.x3, self.x4, self.x5, self.x6]
        self.list_y = [self.y0, self.y1, self.y2, self.y3, self.y4, self.y5, self.y6]
        self.list_z = [self.z0, self.z1, self.z2, self.z3, self.z4, self.z5, self.z6]
        self.list_gain = [self.inputMatrix_1, self.inputMatrix_2, self.inputMatrix_3]
        [self.list_x_enabled.append(x) for x in self.list_x]
        [self.list_y_enabled.append(y) for y in self.list_y]
        [self.list_z_enabled.append(z) for z in self.list_z]
        self.pushButton_4.setEnabled(False)

        # disable plot button on app start
        self.pushButton_8.setEnabled(False)
        self.pushButton_9.setEnabled(False)
        self.pushButton_10.setEnabled(False)

        self.groupBox_3.hide()
        self.groupBox_4.hide()
        self.groupBox_5.hide()
        self.groupBox_6.hide()
        self.groupBox_7.hide()
        self.groupBox_8.hide()

    def kurangiIterasi(self):
        self.list_x_enabled[-1].setEnabled(False)
        self.list_y_enabled[-1].setEnabled(False)
        self.list_z_enabled[-1].setEnabled(False)
        self.list_x_disabled.append(self.list_x_enabled[-1])
        self.list_y_disabled.append(self.list_y_enabled[-1])
        self.list_z_disabled.append(self.list_z_enabled[-1])
        self.list_x_enabled.pop()
        self.list_y_enabled.pop()
        self.list_z_enabled.pop()
        self.label_17.setText(f"{len(self.list_x_enabled)}")
        print(len(self.list_x_enabled))
        if len(self.list_x_enabled) == 6:
            self.pushButton_4.setEnabled(True)
        if len(self.list_x_enabled) == 1:
            self.pushButton_3.setEnabled(False)

    def tambahIterasi(self):
        self.list_x_disabled[-1].setEnabled(True)
        self.list_y_disabled[-1].setEnabled(True)
        self.list_z_disabled[-1].setEnabled(True)
        self.list_x_enabled.append(self.list_x_disabled[-1])
        self.list_y_enabled.append(self.list_y_disabled[-1])
        self.list_z_enabled.append(self.list_z_disabled[-1])
        del self.list_x_disabled[-1]
        del self.list_y_disabled[-1]
        del self.list_z_disabled[-1]
        self.label_17.setText(f"{len(self.list_x_enabled)}")
        print(len(self.list_x_enabled))
        if len(self.list_x_enabled) == 2:
            self.pushButton_3.setEnabled(True)
        if len(self.list_x_enabled) == 7:
            self.pushButton_4.setEnabled(False)

    def checkGripper(self):
        if not self.checkBox_2.isChecked():
            if self.comboBox_2.currentText() == self.comboBox_4.currentText():
                self.msgBox.setText(f"Shut and open action of the gripper can't be in the same iteration!")
                self.msgBox.setWindowTitle("Information")
                self.msgBox.setIcon(self.msgBox.icon().Information)
                self.msgBox.exec()
                return False
            else:
                return True
        else:
            return True

    def checkFilled(self):
        self.unfilled1 = False
        self.unfilled2 = False

        # check if all the column is filled and the value is in valid range
        for x, y, z in zip(self.list_x_enabled, self.list_y_enabled, self.list_z_enabled):
            x = x.text()
            y = y.text()
            z = z.text()
            if not x or not y or not z:
                if not self.unfilled1:
                    self.unfilled1 = True

        for g in self.list_gain:
            g = g.text()
            if not g:
                print(g)
                if not self.unfilled2:
                    self.unfilled2 = True

        if self.unfilled1 or self.unfilled2:
            self.msgBox.setText(f"All column must be filled!")
            self.msgBox.setWindowTitle("Information")
            self.msgBox.setIcon(self.msgBox.icon().Information)
            self.msgBox.exec()
            return False
        else:
            return True

    def enableButton(self):
        self.pushButton_5.setEnabled(True)
        self.pushButton_8.setEnabled(True)
        self.pushButton_9.setEnabled(True)
        self.pushButton_10.setEnabled(True)

    def clearData(self):
        data_torque = [self.data_torque_satu, self.data_torque_dua, self.data_torque_tiga]
        data_trajektori = [self.data_trajectory_x, self.data_trajectory_y, self.data_trajectory_z]

        [data.clear() for data in data_torque]
        [data.clear() for data in data_trajektori]
        self.timeTrajectory.clear()
        self.timeTorque.clear()
        self.dataPercentageTorqueOne.clear()
        self.dataPercentageTorqueTwo.clear()
        self.dataPercentageTorqueThree.clear()

    def appendTorqueLog(self, data):
        self.plainTextEdit.appendPlainText(data)

    def appendTrajectoryLog(self, data):
        self.plainTextEdit_2.appendPlainText(data)

    def saveTorqueData(self, data):
        self.data_torque_satu.append(data[0])
        self.data_torque_dua.append(data[1])
        self.data_torque_tiga.append(data[2])
        self.timeTorque.append(data[3])
        self.dataPercentageTorqueOne.append(data[4])
        self.dataPercentageTorqueTwo.append(data[5])
        self.dataPercentageTorqueThree.append(data[6])

        if (len(self.data_torque_satu) == len(self.list_x_enabled)
                and len(self.data_torque_dua) == len(self.list_y_enabled)
                and len(self.data_torque_tiga) == len(self.list_z_enabled)):
            self.data_torque = [self.data_torque_satu, self.data_torque_dua, self.data_torque_tiga]
            print(f"data torque final {self.data_torque}")

    def saveTrajectoryData(self, data):
        self.data_trajectory_x.append(data[0] * 100)
        self.data_trajectory_y.append(data[1] * 100)
        self.data_trajectory_z.append(data[2] * 100)
        self.timeTrajectory.append(data[3])

        if (len(self.data_trajectory_x) == len(self.list_x_enabled)
                and len(self.data_trajectory_y) == len(self.list_y_enabled)
                and len(self.data_trajectory_z) == len(self.list_z_enabled)):
            self.data_trajectory = [self.data_trajectory_x, self.data_trajectory_y, self.data_trajectory_z]
            self.data_trajectory_input = [[float(x.text()) for x in self.list_x_enabled],
                                          [float(y.text()) for y in self.list_y_enabled],
                                          [float(z.text()) for z in self.list_z_enabled]]
            print(f"data trajectory servo {self.data_trajectory}")
            print(f"data trajectory input {self.data_trajectory_input}")

    def sendData(self):
        check_filled = self.checkFilled()
        check_gripper = self.checkGripper()
        if check_filled and check_gripper:
            if self.serial.open:
                self.plainTextEdit.clear()
                self.plainTextEdit_2.clear()
                self.clearData()

                gripper_action_tutup = self.comboBox_2.itemData(self.comboBox_2.currentIndex())
                gripper_action_buka = self.comboBox_4.itemData(self.comboBox_4.currentIndex())
                gripper_action = [gripper_action_buka[0], gripper_action_tutup[0]]

                self.serial.serialSendData(self, gripper_action, self.list_x_enabled, self.list_y_enabled,
                                                 self.list_z_enabled)
                self.xAxisRange = len(self.list_x_enabled)
                print(self.xAxisRange)
                self.pushButton_5.setEnabled(False)
                self.pushButton_8.setEnabled(False)
                self.pushButton_9.setEnabled(False)
                self.pushButton_10.setEnabled(False)

                self.thread = QThread()
                self.worker = Worker(self.serial)
                self.worker.moveToThread(self.thread)
                #
                self.thread.started.connect(self.worker.run)
                self.worker.finished.connect(self.thread.quit)
                self.worker.finished.connect(self.worker.deleteLater)
                self.worker.finished.connect(self.enableButton)
                self.thread.finished.connect(self.thread.deleteLater)
                self.worker.progress1.connect(self.appendTorqueLog)
                self.worker.progress2.connect(self.appendTrajectoryLog)
                self.worker.progress1_data.connect(self.saveTorqueData)
                self.worker.progress2_data.connect(self.saveTrajectoryData)

                self.thread.start()

            else:
                self.msgBox.setText(f"Port is not connected!")
                self.msgBox.setWindowTitle("Information")
                self.msgBox.setIcon(self.msgBox.icon().Information)
                self.msgBox.exec()

    def checkBoxToogle(self):
        if self.checkBox_2.isChecked():
            self.comboBox_2.setEnabled(False)
            self.comboBox_4.setEnabled(False)
        else:
            self.comboBox_2.setEnabled(True)
            self.comboBox_4.setEnabled(True)

    def serialConnect(self):
        if self.pushButton_2.text() == "Connect":
            self.serial.serialConnect(self)
            if self.serial.open:
                self.plainTextEdit.clear()
                self.plainTextEdit_2.clear()
                self.pushButton_2.setText(QCoreApplication.translate("self.mainWindow", u"Disconnect", None))
                self.pushButton.setEnabled(False)
                self.comboBox.setEnabled(False)

                # message box berhasil
                self.msgBox.setText(f"Successfully connected to port {self.comboBox.currentText()}")
                self.msgBox.setWindowTitle("Information")
                self.msgBox.setIcon(self.msgBox.icon().Information)
                self.msgBox.exec()
                # resize window dan ubah geometri elemen
                self.mainWindow.setMaximumSize(970, 666)
                self.mainWindow.resize(970, 666)
                self.groupBox.setGeometry(QRect(10, 0, 341, 211))
                self.pushButton.setGeometry(QRect(230, 90, 75, 31))
                self.pushButton_2.setGeometry(QRect(130, 110, 71, 31))
                self.comboBox.setGeometry(QRect(130, 80, 71, 22))
                self.label.setGeometry(QRect(40, 90, 81, 31))

                self.groupBox_3.show()
                self.groupBox_4.show()
                self.groupBox_5.show()
                self.groupBox_6.show()
                self.groupBox_7.show()
                self.groupBox_8.show()

            else:
                # message box gagal
                self.msgBox.setIcon(self.msgBox.icon().Warning)
                self.msgBox.setText(f"Fail to connect to port {self.comboBox.currentText()}")
                self.msgBox.exec()
        else:
            self.serial.serialClose()
            self.plainTextEdit.clear()
            self.pushButton_2.setText(QCoreApplication.translate("self.mainWindow", u"Connect", None))
            self.pushButton.setEnabled(True)
            self.comboBox.setEnabled(True)
            self.mainWindow.resize(359, 158)
            self.mainWindow.setMaximumSize(359, 158)
            self.groupBox.setGeometry(QRect(10, 0, 341, 111))
            self.pushButton.setGeometry(QRect(260, 30, 75, 31))
            self.pushButton_2.setGeometry(QRect(170, 60, 71, 31))
            self.comboBox.setGeometry(QRect(170, 30, 71, 22))
            self.label.setGeometry(QRect(50, 20, 81, 31))

    def openAboutWindow(self):
        self.msgBox.about(None, "Robot Arm Control App", "<center></center><b>Robot Arm Control App</b><br/><br/>\
            <a href ='https://github.com/maplezs/EWS_3_4a'>\
            https://github.com/maplezs/EWS_3_4a</a><br/><br/>\
            Developed by Rafsanjani Nurul Irsyad<br/><br/>\
            2024")

    def openFile(self):
        file_name, _ = QFileDialog.getOpenFileName(self.mainWindow, "Open File", "", "Text Files (*.txt)")
        if file_name:
            with open(file_name, "r") as f:
                saved_data = f.read()
                saved_data = json.loads(saved_data)
                if saved_data:
                    fullLen = False
                    for x, y, z in zip(self.list_x, self.list_y, self.list_z):
                        x.clear()
                        y.clear()
                        z.clear()

                    if self.list_x_disabled:
                        for x, y, z in zip(self.list_x_disabled, self.list_y_disabled, self.list_z_disabled):
                            x.setEnabled(True)
                            y.setEnabled(True)
                            z.setEnabled(True)
                    self.list_x_disabled.clear()
                    self.list_y_disabled.clear()
                    self.list_z_disabled.clear()

                    [x.setEnabled(True) for x in self.list_x_disabled]
                    [y.setEnabled(True) for y in self.list_y_disabled]
                    [z.setEnabled(True) for z in self.list_z_disabled]
                    self.list_x_enabled.clear()
                    self.list_y_enabled.clear()
                    self.list_z_enabled.clear()
                    [self.list_x_enabled.append(x) for x in self.list_x]
                    [self.list_y_enabled.append(y) for y in self.list_y]
                    [self.list_z_enabled.append(z) for z in self.list_z]

                    if len(self.list_x_enabled) == 7:
                        self.pushButton_4.setEnabled(False)
                        self.pushButton_3.setEnabled(True)
                    if len(self.list_x_enabled) == 1:
                        self.pushButton_3.setEnabled(False)
                        self.pushButton_4.setEnabled(True)

                    self.label_17.setText(f"{len(self.list_x_enabled)}")
                    if len(saved_data.get("trajectory").get("x")) == 7:
                        fullLen = True
                    if not fullLen:
                        for i in range(7 - len(saved_data.get("trajectory").get("x"))):
                            self.kurangiIterasi()

                    for x, value in zip(self.list_x_enabled, saved_data.get("trajectory").get("x")):
                        precision = zeros_count(float(value))
                        if precision > 0 and precision is not inf:
                            x.setText(format(float(value), f'.{precision + 1}f'))
                        else:
                            x.setText(str(value))

                    [y.setText(str(value)) for y, value in
                     zip(self.list_y_enabled, saved_data.get("trajectory").get("y"))]

                    [z.setText(str(value)) for z, value in
                     zip(self.list_z_enabled, saved_data.get("trajectory").get("z"))]

                    self.inputMatrix_1.setText('\t'.join(saved_data.get("matrixGain").get("satu")))
                    self.inputMatrix_2.setText('\t'.join(saved_data.get("matrixGain").get("dua")))
                    self.inputMatrix_3.setText('\t'.join(saved_data.get("matrixGain").get("tiga")))

                    if not saved_data.get('gripper'):
                        self.comboBox_2.setCurrentIndex(0)
                        self.comboBox_4.setCurrentIndex(0)
                        self.checkBox_2.setChecked(False)

                    if saved_data.get('gripper') == "nan":
                        self.checkBox_2.setChecked(True)
                        return

                    if saved_data.get('gripper'):
                        self.checkBox_2.setChecked(False)
                        self.comboBox_2.setCurrentIndex(saved_data.get('gripper').get('tutup') - 1)
                        self.comboBox_4.setCurrentIndex(saved_data.get('gripper').get('buka') - 1)

    def saveFile(self):
        check_data = self.checkFilled()
        check_gripper = self.checkGripper()
        if check_data and check_gripper:
            file_name, _ = QFileDialog.getSaveFileName(self.mainWindow, "Save File", "", "Text Files (*.txt)")
            if file_name.endswith(".txt"):
                matrix_1 = re.split(r'\t| ', self.inputMatrix_1.text())
                matrix_2 = re.split(r'\t| ', self.inputMatrix_2.text())
                matrix_3 = re.split(r'\t| ', self.inputMatrix_3.text())
                print(matrix_1)
                print(matrix_2)
                print(matrix_3)
                save_data = {
                    "matrixGain": {
                        "satu": matrix_1,
                        "dua": matrix_2,
                        "tiga": matrix_3
                    },
                    "trajectory": {
                        "x": [x.text() for x in self.list_x_enabled],
                        "y": [y.text() for y in self.list_y_enabled],
                        "z": [z.text() for z in self.list_z_enabled]
                    }
                }
                if self.checkBox_2.isChecked():
                    save_data.update({"gripper": "nan"})
                else:
                    save_data.update({'gripper': {'buka': self.comboBox_4.currentIndex() + 1,
                                                  'tutup': self.comboBox_2.currentIndex() + 1}})
                with open(file_name, "w") as f:
                    f.write(json.dumps(save_data, indent=4))
            else:
                self.msgBox.setText("Data not saved")
                self.msgBox.setIcon(self.msgBox.icon().Information)
                self.msgBox.exec()
                return

    def saveCSVFile(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if not os.path.exists('saved_log_data'):
            os.makedirs('saved_log_data')
        torqueLogPath = os.path.join('saved_log_data', f'torque_log-{timestamp}.csv')
        trajectoryLogPath = os.path.join('saved_log_data', f'trajectory_log-{timestamp}.csv')
        dataTrajectoryLog = []
        dataTorqueLog = []
        for i in range(len(self.data_trajectory_x)):
            dataTrajectoryLog.append([i + 1, self.timeTrajectory[i], round(self.data_trajectory_x[i], 3),
                                      round(self.data_trajectory_y[i], 3), round(self.data_trajectory_z[i], 3)])
            dataTorqueLog.append([i + 1, self.timeTorque[i], round(self.data_torque_satu[i], 3),
                                  round(self.data_torque_dua[i], 3), round(self.data_torque_tiga[i], 3),
                                  self.dataPercentageTorqueOne[i], self.dataPercentageTorqueTwo[i],
                                  self.dataPercentageTorqueThree[i]])
        with open(torqueLogPath, mode='w', newline='', encoding='utf-8') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Iteration', 'Time', 'Torque 1', 'Torque 2', 'Torque 3', 'Torque 1 Percentage',
                                 'Torque 2 Percentage', 'Torque 3 Percentage'])
            csv_writer.writerows(dataTorqueLog)

        with open(trajectoryLogPath, mode='w', newline='', encoding='utf-8') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Iteration', 'Time', 'X', 'Y', 'Z'])
            csv_writer.writerows(dataTrajectoryLog)

        dataTrajectoryLog.clear()
        dataTorqueLog.clear()

    def plotWindow(self, tipe, *data):
        window = PlotWindow()
        self.plot_windows.append(window)
        window.setWindowTitle(f"Plot {tipe}")
        if tipe == "Trajektori":
            window.setupPlot(tipe, data[0], data[1], self.xAxisRange)
        else:
            window.setupPlot(tipe, data[0], self.xAxisRange)
        window.show()


class PlotWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.widget = QtWidgets.QWidget()
        self.setCentralWidget(self.widget)
        layout = QVBoxLayout(self.widget)
        self.static_canvas = FigureCanvas(Figure(figsize=(5, 5)))
        layout.addWidget(NavigationToolbar(self.static_canvas, self))
        layout.addWidget(self.static_canvas)
        self.ax_1 = self.static_canvas.figure.add_subplot(231)
        self.ax_2 = self.static_canvas.figure.add_subplot(232)
        self.ax_3 = self.static_canvas.figure.add_subplot(233)
        self.resize(1172, 685)

    def resizeEvent(self, e: QResizeEvent, a=1):
        print(e.size().width())
        print(e.size().height())

    def setupPlot(self, tipe, *data):
        static_ax_list = [self.ax_1, self.ax_2, self.ax_3]
        print(data)
        if tipe == "Trajektori":
            for i in range(len(static_ax_list)):
                static_ax_list[i].plot(range(1, data[2] + 1), data[0][i], label='Reference')
                static_ax_list[i].legend()
                static_ax_list[i].plot(range(1, data[2] + 1), data[1][i], label='Actual')
                static_ax_list[i].legend()
                static_ax_list[i].set_xlabel('Iteration')
                static_ax_list[i].set_ylabel('Trajectory (cm)')
                if i == 2:
                    static_ax_list[0].set_title('X Trajectory')
                    static_ax_list[1].set_title('Y Trajectory')
                    static_ax_list[2].set_title('Z Trajectory')

        else:
            for i in range(len(static_ax_list)):
                static_ax_list[i].plot(range(1, data[1] + 1), data[0][i])
                static_ax_list[i].set_xlabel('Iteration')
                static_ax_list[i].set_ylabel('Torque (N.m)')
                if i == 2:
                    static_ax_list[0].set_title('Torque Base Joint')
                    static_ax_list[1].set_title('Torque Middle Joint')
                    static_ax_list[2].set_title('Torque Top Joint')


class Worker(QObject):
    finished = pyqtSignal()
    progress1 = pyqtSignal(str)
    progress2 = pyqtSignal(str)
    progress1_data = pyqtSignal(list)
    progress2_data = pyqtSignal(list)

    def __init__(self, serial):
        super().__init__()
        self.serial = serial

    def run(self):
        self.serial.serialLogPrint(self)
        self.finished.emit()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWindow = QMainWindow()
    windows = Ui_MainWindow(SerialControl(), mainWindow)
    windows.setupUi()
    mainWindow.show()
    sys.exit(app.exec())
