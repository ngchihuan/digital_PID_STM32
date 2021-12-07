from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QObject,pyqtSlot

from pid_controller import DAC_conv_fac_ltc2785_16bit
from pid_controller import returnSpanMode

class ADC_DAC(QtWidgets.QWidget):
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("ADC_DAC")
        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)
        
        #adc1_box = QtWidgets.QGroupBox()
        self.adc1 = ADC("ADC 1")
        self.adc2 = ADC("ADC 2")
        self.dac1 = DAC( "DAC 1")
        self.dac2 = DAC( "DAC 2")
              
        
        layout.addWidget(self.adc1.adc_box, 0, 0)
        layout.addWidget(self.adc2.adc_box, 0, 1)
        layout.addWidget(self.dac1.dac_box, 1, 0)
        layout.addWidget(self.dac2.dac_box, 1, 1)
        
        #self.dac1.setSpanInput.valueChanged.connect(self.updateVoltSpan)
        
    def updateVoltSpan(self):
        print('test')
        
        
        
class ADC():
    def __init__(self,name):
        self.adc_box = QtWidgets.QGroupBox()
        self.name = name
        self.adc_box.setTitle(self.name)
        self.adc_box.setMinimumWidth(200)
        
class DAC(QObject):
    def __init__(self,name):
        super().__init__()
        
        
        self.minV = 0
        self.maxV = 5.0
        
        self.dac_box = QtWidgets.QGroupBox()
        self.name = name
        self.dac_box.setTitle(self.name)
        self.dac_box.setMinimumWidth(400)
        
        self.dac_form = QtWidgets.QFormLayout(self.dac_box)
        
        self.setVoltInput = QtWidgets.QDoubleSpinBox(decimals=3)
        self.setVoltInput.setSingleStep(0.001)
        
        self.dac_form.addRow(QtWidgets.QLabel("Output"), self.setVoltInput)
        
        self.setSpanInput = QtWidgets.QSpinBox()
        self.setSpanInput.setSingleStep(1)
        self.setSpanInput.setMaximum(5)
        self.dac_form.addRow(QtWidgets.QLabel("Span Mode"), self.setSpanInput)
        
        self.SpanSpec_layout = QtWidgets.QGridLayout()
        self.minV_label = QtWidgets.QLabel(str(self.minV))
        self.maxV_label = QtWidgets.QLabel(str(self.maxV))
        self.SpanSpec_layout.addWidget(self.minV_label,0,1)
        self.SpanSpec_layout.addWidget(QtWidgets.QLabel("min V"),0,0)
        
        self.SpanSpec_layout.addWidget(QtWidgets.QLabel("max V"),1,0)
        self.SpanSpec_layout.addWidget(self.maxV_label,1,1)
        
        self.dac_form.addRow(self.SpanSpec_layout)
        
        self.setSpanInput.valueChanged.connect(self.updateVoltSpan)
        
    
    def updateVoltSpan(self):
        spanMode = self.setSpanInput.value()
        ret_spanMode = returnSpanMode(spanMode)
        
        self.minV = ret_spanMode[1]
        self.maxV = ret_spanMode[2]
        
        self.minV_label.setText(str(ret_spanMode[1]))
        self.maxV_label.setText(str(ret_spanMode[2]))
        
if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = ADC_DAC()
    window.show()
    app.exec_()
