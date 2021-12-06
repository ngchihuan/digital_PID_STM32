
from pid_controller import transfer_func_LTC2380 as tf
from pid_controller import controller as pid
from pid_controller import search_ctr_boards
from pid_controller import int_to_bytearray
from pid_controller import *
import time

def test_pid():
    ctrlist = search_ctr_boards()
    controller_1 = pid(ctrlist[0])
    controller_1.PID_hold()


    controller_1.setDACSpan1(0)
    controller_1.setDACVolt1(4.0)
    
    time.sleep(5)
    controller_1.setDACSpan1(2)
    controller_1.setDACVolt1(-1.0)
    
    time.sleep(5)
    controller_1.setDACSpan1(3)
    controller_1.setDACVolt1(-10.0)
    
    time.sleep(5)
    controller_1.setDACSpan1(0)
    controller_1.setDACVolt1(-10.0)

a=2**18-1
b = a.to_bytes(4,'big')
ba = bytearray(b)
print(ba)
print(ba[0])

test_pid()