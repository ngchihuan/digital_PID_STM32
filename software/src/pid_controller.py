'''
Created on 07-03-2021

@author: NCH
'''
import sys
import glob
import serial
import json
import datetime
import time
import usb.core
import usb.util
import numpy as np
import matplotlib.pyplot  as plt
import struct

ADC_conv_fac_ltc2377 = 5/4/(2**20 -1)
ADC_conv_fac_ltc2380 = 10/(2**24 -1)
DAC_conv_fac_ltc2785_16bit = 65535.0/5.0 #for range of 5V

usb_commands ={
    "SET_OUTPUT_VOLTAGE_1" : 0x01 ,
    "SET_OUTPUT_VOLTAGE_2" : 0x02 ,

    "SET_SPAN_1" :						0x03,
    "SET_SPAN_2" : 0x04,

    "READ_DAC1_REGIS" : 0x05,
    "READ_DAC2_REGIS" : 0x06,

    "READ_ADC1": 0x07,
    "READ_ADC2": 0x09,

    "TRANX_PID_1_CTRL" :				0X1A,
    "TRANX_PID_2_CTRL" :				0X2A,

    "SET_P_1" : 0x0C,
    "SET_I_1" : 0x0D,
    "SET_D_1" : 0x0E,
    "SET_SETPOINT_1" : 				0x0F,

    "SET_P_2" : 0x3C,
    "SET_I_2" : 0x3D,
    "SET_D_2" : 0x3E,
    "SET_SETPOINT_2" : 				0x3F,


    "START_PID": 0xFF,
    "STOP_PID": 0xAF
    
}
def dec_hex(x):
    r0 = x % 16
    q0 = x//16

    r1= q0 % 16
    q1 = q0 //16

    r2= q1 % 16
    q2 = q1 //16

    r3= q2 % 16
    q3 = q2 //16

    print(r3,r2,r1,r0)

    t1 = r0 + 16*r1
    t2= r2+ r3*16

    print(t2,t1)
    return (t1,t2)

def float_bytearray_convert(value):
    ba = bytearray(struct.pack("f", value))  
    return ba

def convert_multi_reads(num_run,ret,ADCR,conversion_fac=1):

    '''
    for ltc2377
    for i in range(num_run):
        ADCR.append( conversion_fac*\
            (ret[4*i+2]*16**4 + ret[4*i+1]*16**2 +ret[4*i]))
    '''

    for i in range(num_run):
        #temp_value= 20.0/(2**24-1) * ( ret[4*i+2]*16**4 + ret[4*i+1]*16**2 +ret[4*i]) -10
        temp_value = transfer_func_LTC2380( ret[4*i+2]*16**4 + ret[4*i+1]*16**2 +ret[4*i])
        #temp_value = (temp_value +5.0)*2.5
        ADCR.append( temp_value)

    
def transfer_func_LTC2380(digicode):
    '''
    Summary of the LTC2380 
    011..111 -> 5V
    ...
    0x00 -> 0 volt
    0xfff -> (0 - VLSB)volt
    ...
    100..00 -> -5V

    int digicode: output code of ADC in decimal.

    returns:
    input voltage to the ADC
    '''
    FS= 12.05
    VLSB = FS/(2**24)
    maxcode = 2**24-1
    if (digicode > maxcode):
        raise ValueError('input out of range')
    
    if (digicode >= 2**23):
        a = (FS/2 - VLSB)/(2**24 - 2**23)
        b = -FS/2 -a*2**23
    else:
        a=FS/2/( (2**24-1)/2)
        b=0.0
    res = a*digicode +b
    return res




class controller_search():
    def __init__(self):
#        bk = libusb01.get_backend()
#        self.ddslist = usb.core.find(idVendor=0x04B4, idProduct=0x1236, backend = bk, find_all=True)
        self.ctrlist = list(usb.core.find(idVendor=0x04d2, idProduct=0x3039, find_all=True))
        print ("Found controller boards", self.ctrlist)
        # was it found?
        if self.ctrlist is None:
            raise ValueError('Controller boards not found')  

        self.ctrnum = len(self.ctrlist)
        sys.stdout.write(str(self.ctrnum) + " Controller board(s) found \n")

        # Make a dictionary of boards with serial number as a key        
        self.CtrBoards = {}
        self.sernums = []

        
        for ctritem in self.ctrlist:
            d = controller(ctritem)
            self.sernums.append(str(d.sernum))
            self.CtrBoards[d.sernum] = d
        
        print ("All boards: " + str(self.sernums))

def search_ctr_boards():
    ctrlist = list(usb.core.find(idVendor=0x04d2, idProduct=0x3039, find_all=True))
    print ("Found controller boards", ctrlist)
        # was it found?
    if ctrlist is None:
        raise ValueError('Controller boards not found')  

    ctrnum = len(ctrlist)
    sys.stdout.write(str(ctrnum) + " Controller board(s) found \n")
    return ctrlist

class controller():
    def __init__(self, ctr_usb_addre):
        self.dev = ctr_usb_addre #dev is the usbcore obj 
        self.cfg = self.dev.get_active_configuration()
        self.cfg = self.dev[0]
        self.intf = self.cfg[(1,0)]
        self.ep_write = self.intf[0] # end point of writing
        self.ep_read = self.intf[1] #end point of reading

        if self.dev.is_kernel_driver_active(0):
            self.reattach = True
            self.dev.detach_kernel_driver(0)
        #self.sm = self.get_sm()
        #self.PID_start()
        
        self.cmds = bytearray(b'')
        
        self.cmds.append(0x00)
        self.cmds.append(0x00)
        self.cmds.append(0x00)
        self.cmds.append(0x00)


    def close_dev(self):
        print("Closing device")
        usb.util.dispose_resources(self.dev)

        if self.reattach:
            self.dev.attach_kernel_driver(0)

    def write_mess(self,msg):
        self.ep_write.write(msg)

    def read_mess(self,num_read=1):
        ret = self.ep_read.read(num_read)
        return ret

    def get_sm(self):
        print("Inquiring controller board's status")
        
        self.cmds = bytearray(b'')
        
        self.cmds.append(0xA3)
        self.write_mess(self.cmds)
        ret = self.read_mess()
        print(ret)
        return ret
    
    def read_ADC_1_buffer(self,num_read=1):
        '''
        Do not begin ADC
        Just Read ADC 1 buffer[0 -> num_read -1]
        
        '''
        ADCR=[]
        print("Reading ADC 1 Buffer for ", num_read)
        self.cmd = bytearray(b'')
        self.cmd.append(0x0A)
        self.cmd.append(num_read)
        self.cmd.append(0x00)
        self.cmd.append(0x00)
        self.write_mess(self.cmd)
        ret = self.ep_read.read(num_read*100)
        #print(ret)

        convert_multi_reads(num_read,ret,ADCR,conversion_fac=ADC_conv_fac_ltc2377)
        #plt.plot(ADCR,'-o')
        print(np.mean(ADCR))
   

    def read_PID_1_buffer(self,num_read=1):
        '''
        READ PID 1 buff
        Read ADC 1 buffer [0 -> num_read -1]
        DAC 1 set [num_read -> 2*num_read -1]
        
        '''
        ADCR=[]
        print("Reading PID 1 Buffer for ", num_read)
        self.cmd = bytearray(b'')
        self.cmd.append(usb_commands["TRANX_PID_1_CTRL"])
        self.cmd.append(num_read)
        self.cmd.append(0x00)
        self.cmd.append(0x00)
        print(self.cmd)
        self.write_mess(self.cmd)
        ret = self.ep_read.read(num_read*16)
        print(np.size(ret))
        print(ret)
        return ret
        #convert_multi_reads(num_read,ret,ADCR)
        #print(ADCR)       

    def process_PID_buffer(self,PID_buffer):
         '''
         Take num_read*2 size array from read_PID_1_buffer and process
         output_processed is reading of ADC
         control_processed is reading of DAC
         '''
         size = np.size(PID_buffer)
         output_processed=[]
         control_processed = []
         output = PID_buffer[:int(size/2)]
         control = PID_buffer[int(size/2):]
         print('size of output ', np.size(output))
         print('size of control ', np.size(control))
         convert_multi_reads(int(size/8),output, output_processed, conversion_fac=ADC_conv_fac_ltc2377)
         convert_multi_reads(int(size/8),control, control_processed,conversion_fac=1/DAC_conv_fac_ltc2785_16bit)
         #plt.plot(output_processed,'-o')
         #plt.show()
         return output_processed,control_processed

    def read_ADC_1(self,num_run=1):
        #try num_run>0 & num_run<100
        '''
        num_run is a multiple of 512 runs
        num_run =10 => obtain 5120 ADC reading
        num_run <=100
        '''
        print('Reading ADC for ',num_run*512)
        self.cmd = bytearray(b'')
        self.cmd.append(0x07)
        self.cmd.append(num_run)
        self.cmd.append(0x00)
        self.cmd.append(0x00)
        self.write_mess(self.cmd)

        ret=[]
        ADCR=[]
        
        size_convert_const=512
        bytes_received = 512*4


        for i in range (num_run):
            
            print('transfer number ',i)

            bytes_received =size_convert_const*4
            print('expect to received bytes', bytes_received )
            
            
            
            #ret = self.ep_read.read(bytes_received)
            ret = self.ep_read.read(5000)
            print('Size of received ',np.size(ret))
            print(np.size(ret))

            print('\n')
             
            convert_multi_reads(size_convert_const,ret,ADCR, conversion_fac=ADC_conv_fac_ltc2377 )
        #print(ret)
        print('size ADCR ', np.size(ADCR))
        #print(ADCR)
        print(ADCR[2])
        print(np.mean(ADCR))
        #plt.plot(ADCR,'-o')
        plt.show()
        return ret




    def readADC_2(self):
        pass

    def multi_read_ADC_2(self):
        pass

    def PID_start(self):
        self.cmd = bytearray(b'')
        self.cmd.append(0xFF)

        self.write_mess(self.cmd)
        print('PID started')

    def PID_hold(self):
        '''
        Stop PID, but keep the contro the output constant
        '''
        self.cmd = bytearray(b'')
        self.cmd.append(0xAF)

        self.write_mess(self.cmd)
        print('PID hold')
        

    def PID_shutdown(self):
        '''
        Stop PID, and ramp down the control output to zero
        '''
        print('PID stopped ')
        

    def set_PID_params(self,params):
        #params = [P,I,D,Vmax,Vmin]
        set_I_params()
        set_D_params()
        set_P_params()
        

    def set_I1_params(self,Iparams):
        print('Setting I of PID 1', Iparams)
        cmd2=float_bytearray_convert(Iparams)
        self.cmd = bytearray(b'')
        self.cmd.append(usb_commands["SET_I_1"])
        for i in cmd2:
            self.cmd.append(i)
        self.write_mess(self.cmd)
        ret=self.read_mess(10)
        print('Received ',[ "0x%02x" % b for b in ret ])

    def set_D1_params(self,Dparams):
        print('Setting D of PID 1', Dparams)
        cmd2=float_bytearray_convert(Dparams)
        self.cmd = bytearray(b'')
        self.cmd.append(usb_commands["SET_D_1"])
        for i in cmd2:
            self.cmd.append(i)
        self.write_mess(self.cmd)
        ret=self.read_mess(10)
        print('Received ',[ "0x%02x" % b for b in ret ])

    def set_P1_params(self,Pparams):
        print('Setting P of PID1', Pparams)
        cmd2=float_bytearray_convert(Pparams)
        self.cmd = bytearray(b'')
        self.cmd.append(usb_commands["SET_P_1"])
        for i in cmd2:
            self.cmd.append(i)
        self.write_mess(self.cmd)
        ret=self.read_mess(10)
        print('Received ',[ "0x%02x" % b for b in ret ])
        



    def toggle_led_test(self,num_run):
        #try num_run>0 & num_run<2**16
        t1,t2=dec_hex(num_run)
        self.cmd = bytearray(b'')
        self.cmd.append(0xA4)
        self.cmd.append(t1)
        self.cmd.append(t2)
        self.cmd.append(0x00)
        #self.cmd.append(int(num_run/1000))
        self.write_mess(self.cmd)
    

if __name__=="__main__":
    ctrlist = search_ctr_boards()
    controller_1 = controller(ctrlist[0])
    controller_1.PID_hold()

    controller_1.set_P1_params(1.0)
    controller_1.set_I1_params(0.0)
    controller_1.set_D1_params(0.0)

    #controller_1.PID_start()
    '''
    for i in range(3):
        time.sleep(0.01)
        ret=controller_1.read_PID_1_buffer(200)

    output_processed, control_processed = controller_1.process_PID_buffer(ret)
    plt.plot(output_processed,'-o')
    control_processed = np.asarray(control_processed)-0.01
    plt.plot(control_processed,'-o',color='red',label='control')
    plt.show()
    '''
    ret = controller_1.read_ADC_1()
    #print(ret)

    '''
    controller_1.PID_hold()
    controller_1.PID_start()
    for i in range(3):
        time.sleep(0.01)
        ret=controller_1.read_PID_1_buffer(250)
    controller_1.PID_hold()
    output_processed, control_processed = controller_1.process_PID_buffer(ret)
    plt.plot(output_processed,'-o')
    control_processed = np.asarray(control_processed)-0.01
    plt.plot(control_processed,'-o',color='red',label='control')
    plt.show()
    '''





    #controller_1.read_ADC_1(num_run=100)
    #controller_1.toggle_led_test(600)
    #print(ret)
