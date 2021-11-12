# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'search_devices.ui'
#
# Created by: PyQt5 UI code generator 5.11.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_search_device(object):
    def setupUi(self, search_device):
        search_device.setObjectName("search_device")
        search_device.resize(290, 292)
        self.listWidget_deviceList = QtWidgets.QListWidget(search_device)
        self.listWidget_deviceList.setGeometry(QtCore.QRect(5, 30, 271, 192))
        self.listWidget_deviceList.setObjectName("listWidget_deviceList")
        self.horizontalLayoutWidget = QtWidgets.QWidget(search_device)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(0, 230, 282, 41))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton_Cancel = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_Cancel.setObjectName("pushButton_Cancel")
        self.horizontalLayout.addWidget(self.pushButton_Cancel)
        self.pushButton_Connect = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_Connect.setObjectName("pushButton_Connect")
        self.horizontalLayout.addWidget(self.pushButton_Connect)
        self.pushButton_Search = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_Search.setObjectName("pushButton_Search")
        self.horizontalLayout.addWidget(self.pushButton_Search)

        self.retranslateUi(search_device)
        QtCore.QMetaObject.connectSlotsByName(search_device)

    def retranslateUi(self, search_device):
        _translate = QtCore.QCoreApplication.translate
        search_device.setWindowTitle(_translate("search_device", "Dialog"))
        self.pushButton_Cancel.setText(_translate("search_device", "Cancel"))
        self.pushButton_Connect.setText(_translate("search_device", "Connect"))
        self.pushButton_Search.setText(_translate("search_device", "Search"))

