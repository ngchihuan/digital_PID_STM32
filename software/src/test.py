from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QObject,pyqtSlot


class test_window(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)
        self.chooseDevice = QtWidgets.QComboBox()
        layout.addWidget(self.chooseDevice)
        
        deviceList = ['hello','how','are']
        for i in deviceList:
            self.chooseDevice.addItem(str(i))
            
        self.chooseDevice.currentIndexChanged.connect(self.comboSlot)

    def comboSlot(self,i):
        print('slot activated')
        print(f'index {i} with text {self.chooseDevice.currentText()} is chosen')
if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = test_window()
    window.show()
    app.exec_()