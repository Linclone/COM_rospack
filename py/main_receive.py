import threading
import time
from UAV_interface import data_received_event, U_receive, set_serial

n=0
def data_received_callback(data):
    global n
    n=n+1
    print(n,len(data))


data_received_event.add_handler(data_received_callback)

def main():
    while True:
        print("Main function is running...")
        time.sleep(1)

if __name__ == "__main__":
    #global Ser
    #Ser = 'COM4'
    #set_serial(Ser)
    # n=1
    t1 = threading.Thread(target=U_receive,args=('/dev/ttyACM1',))
    t1.start()
    main()


