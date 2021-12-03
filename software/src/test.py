import struct
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
def test_raise():
    a=0
    raise TypeError
    print('after raising typeerr')
    return a

def float_bytearray_convert(value):
    ba = bytearray(struct.pack("f", value))  
    return ba

if __name__=='__main__':
    x=1000
    t1,t2=dec_hex(x)
    msg = bytearray(b'')
    msg.append(t1)
    msg.append(t2)
    print(msg)

    res= test_raise()
    print(f"res is {res}")

    
    