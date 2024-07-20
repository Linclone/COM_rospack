import time
import random
from UAV_interface import U_send, set_serial

data = []
U_long = 0
U_num = 3
set_serial('/dev/ttyACM0')
for i in range(4320000):  #4320000

    U_long = 500
    if i <= 255:
        data.append(0x00)
        data.append(0x00)
        data.append(int(hex(i)[2:], 16))
        for ii in range(U_long-3):
            data.append(int(hex(random.randint(1, 255))[2:], 16))

    if i <= 65535 and i > 255:
        hex_num = hex(i)
        hex_str = hex_num[2:].zfill(4) 
        hex_array = [hex_str[i:i + 2] for i in range(0, len(hex_str), 2)]
        data.append(0x00)
        data.append(int(hex_array[0], 16))
        data.append(int(hex_array[1], 16))
        for iii in range(U_long-3):
            data.append(int(hex(random.randint(1, 255))[2:], 16))
    if i > 65535 :
        hex_num = hex(i)
        hex_str = hex_num[2:].zfill(6) 
        hex_array = [hex_str[i:i + 2] for i in range(0, len(hex_str), 2)]
        data.append(int(hex_array[0], 16))
        data.append(int(hex_array[1], 16))
        data.append(int(hex_array[2], 16))
        for iiii in range(U_long-3):
            data.append(int(hex(random.randint(1, 255))[2:], 16))

    U_send(data, U_long, U_num)
    #print(data)
    print(i,U_long)
    time.sleep(0.01)
    data = [] 
