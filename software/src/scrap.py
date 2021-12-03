
from pid_controller import transfer_func_LTC2380 as tf
from pid_controller import controller as pid
from pid_controller import search_ctr_boards
from pid_controller import int_to_bytearray
from pid_controller import *

ctrlist = search_ctr_boards()
controller_1 = pid(ctrlist[0])
controller_1.PID_hold()
controller_1.setDACSpan1(2)
controller_1.setDACMinVolt1()

res = int_to_bytearray(255)
print(res)
print(res[0])

V=[-5 ,0 ,5]
inputCode=[]
for v in V:
    res = convert_Vout_to_inputCode(v,minVspan=-5)
    inputCode.append(res)
    print(f"input code of {v} Volts is {res}")
