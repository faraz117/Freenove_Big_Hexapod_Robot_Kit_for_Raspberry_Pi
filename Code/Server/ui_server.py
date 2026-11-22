# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'C:\Users\Freenove\Desktop\树莓派四足狗\界面\Server\ui_server.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_server(object):
    def setupUi(self, server):
        server.setObjectName("server")
        server.resize(420, 580)
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        server.setFont(font)
        server.setStyleSheet("QWidget{\n"
"background:#484848;\n"
"}\n"
"QAbstractButton{\n"
"border-style:none;\n"
"border-radius:0px;\n"
"padding:5px;\n"
"color:#DCDCDC;\n"
"background:qlineargradient(spread:pad,x1:0,y1:0,x2:0,y2:1,stop:0 #858585,stop:1 #383838);\n"
"}\n"
"QAbstractButton:hover{\n"
"color:#000000;\n"
"background-color:#008aff;\n"
"}\n"
"QAbstractButton:pressed{\n"
"color:#DCDCDC;\n"
"border-style:solid;\n"
"border-width:0px 0px 0px 4px;\n"
"padding:4px 4px 4px 2px;\n"
"border-color:#008aff;\n"
"background-color:#444444;\n"
"}\n"
"\n"
"QLabel{\n"
"color:#DCDCDC;\n"
"\n"
"\n"
"}\n"
"QLabel:focus{\n"
"border:1px solid #00BB9E;\n"
"\n"
"}\n"
"\n"
"\n"
"")
        self.obstacle_label = QtWidgets.QLabel(server)
        self.obstacle_label.setGeometry(QtCore.QRect(10, 10, 400, 50))
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        font.setPointSize(12)
        self.obstacle_label.setFont(font)
        self.obstacle_label.setAlignment(QtCore.Qt.AlignCenter)
        self.obstacle_label.setWordWrap(True)
        self.obstacle_label.setObjectName("obstacle_label")
        self.camera_label = QtWidgets.QLabel(server)
        self.camera_label.setGeometry(QtCore.QRect(110, 70, 200, 150))
        self.camera_label.setStyleSheet("background:#222222; border:1px solid #666666;")
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setScaledContents(False)
        self.camera_label.setObjectName("camera_label")
        self.stats_label = QtWidgets.QLabel(server)
        self.stats_label.setGeometry(QtCore.QRect(10, 230, 400, 45))
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        font.setPointSize(11)
        self.stats_label.setFont(font)
        self.stats_label.setAlignment(QtCore.Qt.AlignCenter)
        self.stats_label.setObjectName("stats_label")
        self.grid_title_label = QtWidgets.QLabel(server)
        self.grid_title_label.setGeometry(QtCore.QRect(10, 280, 400, 20))
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        font.setPointSize(9)
        self.grid_title_label.setFont(font)
        self.grid_title_label.setAlignment(QtCore.Qt.AlignCenter)
        self.grid_title_label.setObjectName("grid_title_label")
        self.detection_table = QtWidgets.QTableWidget(server)
        self.detection_table.setGeometry(QtCore.QRect(70, 305, 280, 120))
        self.detection_table.setRowCount(3)
        self.detection_table.setColumnCount(3)
        self.detection_table.setHorizontalHeaderLabels(["30°", "90°", "150°"])
        self.detection_table.setVerticalHeaderLabels(["70°", "90°", "130°"])
        self.detection_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.detection_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.detection_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.detection_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.detection_table.setFocusPolicy(QtCore.Qt.NoFocus)
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        font.setPointSize(14)
        font.setBold(True)
        self.detection_table.setFont(font)
        self.detection_table.setObjectName("detection_table")
        for row in range(3):
            for col in range(3):
                item = QtWidgets.QTableWidgetItem('0')
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.detection_table.setItem(row, col, item)
        self.states = QtWidgets.QLabel(server)
        self.states.setGeometry(QtCore.QRect(130, 435, 160, 70))
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        font.setPointSize(36)
        self.states.setFont(font)
        self.states.setAlignment(QtCore.Qt.AlignCenter)
        self.states.setObjectName("states")
        self.pushButton_On_And_Off = QtWidgets.QPushButton(server)
        self.pushButton_On_And_Off.setGeometry(QtCore.QRect(170, 515, 80, 30))
        self.pushButton_On_And_Off.setObjectName("pushButton_On_And_Off")

        self.retranslateUi(server)
        QtCore.QMetaObject.connectSlotsByName(server)

    def retranslateUi(self, server):
        _translate = QtCore.QCoreApplication.translate
        server.setWindowTitle(_translate("server", "Server"))
        self.obstacle_label.setText(_translate("server", "No obstacle detected yet"))
        self.camera_label.setText(_translate("server", "No camera feed"))
        self.stats_label.setText(_translate("server", "CPU: -- %   RAM: -- %\nBatt(load): -- %   Batt(Pi): -- %"))
        self.grid_title_label.setText(_translate("server", "Tilt (rows) / Pan (cols) — 0=clear, U=ultrasonic, C=camera, B=both"))
        self.states.setText(_translate("server", "Running"))
        self.pushButton_On_And_Off.setText(_translate("server", "Pause"))

