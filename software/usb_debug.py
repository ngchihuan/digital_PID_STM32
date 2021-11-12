import numpy as np
import usb.core
import usb.util
import sys
import time
import matplotlib.pyplot  as plt

ADC_conv_fac_ltc2377 = 5/2/(2**20 -1)
def read_ADC_ltc2377(ep,ep_read):
    print("reading from ADC")
    msg = bytearray(b'')
    msg.append(0x07)
    msg.append(0x00)
    msg.append(0x00)
    msg.append(0x00)
    ep.write(msg)
    ret = ep_read.read(5)
    print(ret)
    ADCR = ret[2]*16**4 + ret[1]*16**2 +ret[0]
    measured_voltage = ADCR * ADC_conv_fac_ltc2377
    print('measured voltage ')
    print(measured_voltage)

def multi_read_ADC_ltc2377(num_run,ep, ep_read):
    #num_run must be in unit of 1000
    print("multi read ADC")
    msg = bytearray(b'')
    msg.append(0x08)
    msg.append(int(num_run/1000))
    msg.append(0x00)
    msg.append(0x00)
    ep.write(msg)
    temp = int(num_run/512)
    ADCR=[]
    size_convert=512
    if temp>1:
        for i in range (temp+1):
            print(i)
            if (i<temp):
                ret = ep_read.read(num_run*4)
            else:
                ret = ep_read.read(4*(num_run- temp*512))
                size_convert = (num_run- temp*512)
            print(np.size(ret))
            convert_multi_reads(size_convert,ret,ADCR)
    print('size ADCR ', np.size(ADCR))
    print(ADCR)
    sp = np.abs(np.fft.fft(ADCR))
    plt.plot(sp,'-o')
    #plt.plot(ADCR,'-o')
    plt.show()
    #ret = ep_read.read(num_run*4)
    #ret= np.asarray(ret)
    #print(np.size(ret))
    #print(ret)
    #convert_multi_reads(num_run,ret)

def convert_multi_reads(num_run,ret,ADCR):

    for i in range(num_run):
        ADCR.append( ADC_conv_fac_ltc2377*\
            (ret[4*i+2]*16**4 + ret[4*i+1]*16**2 +ret[4*i]))
    print (np.size(ADCR))
    #print (ADCR)

# find our device
#dev = usb.core.find(idVendor=0x0483, idProduct=0x5740)

def maintest():
    dev = usb.core.find(idVendor=0x04d2, idProduct=0x3039)

    # was it found?
    if dev is None:
        raise ValueError('Device not found')
    else:
        print(dev)
    reattach = True
    if dev.is_kernel_driver_active(0):
        reattach = True
        dev.detach_kernel_driver(0)

    for cfg in dev:
        sys.stdout.write('Conf ' + str(cfg.bConfigurationValue) + '\n')
        for intf in cfg:
            sys.stdout.write('\t' + \
                            'interfaceNum,alterset \t' + \
                            str(intf.bInterfaceNumber) + \
                            ',' + \
                            str(intf.bAlternateSetting) + \
                            '\n')
            for ep in intf:
                sys.stdout.write('\t\t' + \
                                'ep \t' + \
                                str(ep.bEndpointAddress) + \
                                '\n')

    cfg = dev.get_active_configuration()
    cfg=dev[0]
    intf = cfg[(1,0)]

    ep=intf[0]
    ep_read=intf[1]


    #set span dac 1
    '''
    msg = bytearray(b'')
    msg.append(0x03)
    msg.append(0x00)
    msg.append(0x00)
    msg.append(0x00)
    ep.write(msg)

    '''
    '''
    for i in range(2):
        msg = bytearray(b'')
        msg.append(0x01)
        msg.append(0xFF)
        msg.append(0xAF)
        msg.append(0x00)
        ep.write(msg)
    '''

    #read_ADC_ltc2377(ep_read);
    #multi_read_ADC_ltc2377(3,ep, ep_read)
    read_ADC_ltc2377(ep, ep_read)

    # This is needed to release interface, otherwise attach_kernel_driver fails 
    # due to "Resource busy"
    usb.util.dispose_resources(dev)

    if reattach:
        dev.attach_kernel_driver(0)


    
if __name__ == '__main__':
    maintest()

