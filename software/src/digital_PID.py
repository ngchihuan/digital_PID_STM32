import sys
from PyQt5.QtWidgets import QMainWindow,QApplication,QDialog, QApplication, QFileDialog
#from PyQt5.QtCore import pyqtSignal,pyqtSlot
from PyQt5.QtCore import *
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import os
from digital_controller_ui import *
from search_device_ui import *
import usb.core
import usb.util

def search_ctr_boards():
    ctrlist = list(usb.core.find(idVendor=0x04d2, idProduct=0x3039, find_all=True))
    devList=[]
    print ("Found controller boards", ctrlist)
        # was it found?
    if ctrlist is None:
        raise ValueError('Controller boards not found')  

    ctrnum = len(ctrlist)
    sys.stdout.write(str(ctrnum) + " Controller board(s) found \n")
    for d in ctrlist:
        devList.append(controller(d))
        print(usb.util.get_string(d, d.iSerialNumber))
        print(usb.util.get_string(d, d.iProduct ))
        #usb.util.get_string(dev, 256, dev.iManufacturer)
    return devList

class controller():
    def __init__(self,usbObj):
        self.sernum=0
        self.usbadd='test'
        self.dev = usbObj
        self.iProduct = usb.util.get_string(self.dev, self.dev.iProduct )
        


class searchDevice(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_search_device()
        self.ui.setupUi(self)
        self.search_controller()
        #self.ui.pushButton_Connect.clicked.connect(self.connect_clicked)
        self.ui.pushButton_Cancel.clicked.connect(self.cancel_clicked)
        self.ui.pushButton_Search.clicked.connect(self.search_clicked)

        self.ui.listWidget_deviceList.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection
        )
        self.ui.listWidget_deviceList.setFocusPolicy(Qt.StrongFocus)


    def search_controller(self):
        '''
        Search for controllers connected to the host PC

        '''
        print("searching for controllers")
        self.deviceList = search_ctr_boards()
        print(self.deviceList)

        self.update_deviceList(self.deviceList)


    def update_deviceList(self,deviceList):
        self.ui.listWidget_deviceList.clear()
        for i,f in enumerate(deviceList):
            item = QtWidgets.QListWidgetItem(f.iProduct)
            item.setData(QtCore.Qt.UserRole, f) #Attached an object to each item in the QListWidgetItem
            self.ui.listWidget_deviceList.addItem(item)
        

    def retrieveSelectedObject(self):
        self.selected_controller = self.ui.listWidget_deviceList.selectedItems()[0].data(QtCore.Qt.UserRole)

        return self.selected_controller
              
    def return_controller(self):
        return self.selected_controller

    def search_clicked(self):
        self.search_controller()

    def cancel_clicked(self):
        self.close()

    #def connect_clicked(self):
        #self.retrieveSelectedObject()

class mainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.SearchUi = searchDevice()
        self.show()
        self.ui.actionSearch.triggered.connect(self.SearchDialog)

        self.SearchUi.ui.pushButton_Connect.clicked.connect(self.connect_clicked)
        #self.controller = selected from the device search
        #self.retrieve_params()#retrieve params from the controller flash
        #self.update_display() # update params on the gui
        #self.model = CtrModel()
        
    def SearchDialog(self):
        self.SearchUi.exec_()

    def connect_clicked(self):
        self.controller = self.SearchUi.retrieveSelectedObject()
        print(self.controller)
        self.setup_controller()

    def setup_controller(self):
        self.ui.lineEdit_iProduct.setText(self.controller.iProduct)

if __name__=="__main__":
    app = QApplication(sys.argv)
    w=mainWindow()
    w.show()
    sys.exit(app.exec())