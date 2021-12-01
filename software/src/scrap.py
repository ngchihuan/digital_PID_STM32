
from pid_controller import transfer_func_LTC2380 as tf

code1 = 2**23
code2 = 2**24-1
code3 = (2**24-1)/2
code4 = 8388607
res1= tf(code1)
print(f'voltage of {code1} is {res1}')

print(f'voltage of {code2} is {tf(code2)}')

print(f'voltage of {code3} is {tf(code3)}')

print(f'voltage of {code4} is {tf(code4)}')