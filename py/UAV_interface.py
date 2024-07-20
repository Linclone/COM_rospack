import serial
import threading
import crcmod

class DataReceivedEvent:    
    def __init__(self):
        self._handlers = []
        self._lock = threading.Lock()
    def add_handler(self, handler):
        with self._lock:
            self._handlers.append(handler)
    def notify(self, data):    
        with self._lock:
            for handler in self._handlers:
                handler(data)

crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
Frame_num = 0  

global data_received_event
data_received_event = DataReceivedEvent()

def set_serial(com): 
    global ser
    ser = serial.Serial(com, 115200, timeout=0.1)

def U_send(data, length, num):
  
    global Frame_num
    
    list_7e = []
    list_7d = []
  
    # for i in range(len(data)):
    #     if data[i] == 0x7e:
    #         data[i] = 0x7d
    #         list_7e.append(i)
    # if len(list_7e) != 0:
    #     for i in list_7e:
    #         data.insert(i + 1, 0x5e)
 
    # for i in range(len(data)):
    #     if data[i] == 0x7d:
    #         list_7d.append(i)
    # if len(list_7d) != 0:
    #     for i in list_7d:
    #         data.insert(i + 1, 0x5d)
 
    data = bytes(data).hex()
 
    length = int((len(data)/2))
    if length >= 255:
        hex_num = hex(length)
        hex_str = hex_num[2:].zfill(4)
        hex_array = [hex_str[i:i + 2] for i in range(0, len(hex_str), 2)]
        Playload_longth = hex_array[0] + hex_array[1]
    else:
        hex_str = hex(length)[2:].zfill(2)
        Playload_longth = '00' + hex_str
    Frame_data = '7e3f00' + Playload_longth + hex(num)[2:].zfill(2) + hex(Frame_num)[2:].zfill(2) + data
    crc_value=hex(crc16(bytes.fromhex(data)))[2:].zfill(4)

    Frame_data = Frame_data + crc_value + 'aa'
   
    Frame_data1 = [Frame_data[i:i + 2] for i in range(0, len(Frame_data), 2)]
    Frame_data2 = tuple([int(i, 16) for i in Frame_data1])
  
    ser.write(Frame_data2)
   
    if Frame_num == 255:
        Frame_num = 0
    else:
        Frame_num += 1
    return 0

def U_receive(Ser):
    temp_length = 0  
    temp_data = b''  
    cout = True     
    cout1 = 0       
    ser = serial.Serial(Ser, 115200, timeout=0.1)
    while True:
        try:                            
            data = ser.read_all()
            if data != b'':
                #print('')    
                try:
                    if cout1 == 4:       
                        temp_length = 0
                        temp_data = b''
                        cout = True
                        cout1 = 0
                    if data.hex().find('7e3f00ffff0500aa') != -1:  
                        temp_length = 0
                        temp_data = b''
                        cout = True
                        cout1 = 0
                    if cout == True and data[:3].hex() == '7e3f00':  
                        datalength = data[3] << 8 | data[4]         
                        playload = temp_data + data[7:-3]            
                        if datalength == len(playload):              
                            data_crc = data[-3] << 8 | data[-2]
                            crc = crc16(playload)
                            if crc == data_crc:               
                                Frame_data = playload.hex()
                                Frame_data1 = [Frame_data[i:i + 2] for i in range(0, len(Frame_data), 2)]
                                Frame_data2 = [int(i, 16) for i in Frame_data1]
                                data_received_event.notify(Frame_data2)   
                                temp_length = 0     
                                temp_data = b''
                                cout = True
                        else:               
                            cout = False
                            cout1 += 1
                            temp_length = datalength
                            temp_data = data[7:]
                    else:                                    
                        if data[:4].hex() == 'aa7e3f00':     
                            data = data[1:]
                            datalength = data[3] << 8 | data[4]
                            # playload = temp_data + data[7:datalength + 7]
                            playload = temp_data + data[7:-3]
                            if datalength == len(playload):
                                data_crc = data[-3] << 8 | data[-2]
                                crc = crc16(playload)
                                if crc == data_crc:
                                    Frame_data = playload.hex()
                                    Frame_data1 = [Frame_data[i:i + 2] for i in range(0, len(Frame_data), 2)]
                                    Frame_data2 = [int(i, 16) for i in Frame_data1]
                                    data_received_event.notify(Frame_data2)
                                    temp_length = 0
                                    temp_data = b''
                                    cout = True
                            else:
                                cout = False
                                cout1 += 1
                                temp_length = datalength
                                temp_data = data[7:]
                        else:                               
                            datalength = temp_length
                            #playload = temp_data + data[7:datalength + 7]
                            playload = temp_data + data
                            playload= playload[:-3]
                            cout1 += 1
                            if datalength == len(playload):
                                data_crc = data[-3] << 8 | data[-2]
                                crc = crc16(playload)
                                if crc == data_crc:
                                    Frame_data = playload.hex()
                                    Frame_data1 = [Frame_data[i:i + 2] for i in range(0, len(Frame_data), 2)]
                                    Frame_data2 = [int(i, 16) for i in Frame_data1]
                                    data_received_event.notify(Frame_data2)
                                    temp_length = 0
                                    temp_data = b''
                                    cout = True
                            else:
                                temp_length = datalength
                                temp_data = temp_data + data
                except:
                    continue
        except:
            try:
                print(1)
                ser = serial.Serial(Ser, 115200, timeout=0.1)   
                continue
            except:
                pass
