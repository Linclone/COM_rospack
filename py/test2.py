from UAV_interface import data_received_event, U_receive, set_serial
import threading
import time

def data_received_callback(data):
    print(data)

data_received_event.add_handler(data_received_callback)

def main():
    while True:
        print("Main function is running...")
        time.sleep(3)

if __name__ == "__main__":
    try:
        set_serial('COM4')
        t1 = threading.Thread(target=U_receive)
        t1.start()
        #t1.join() 
        main()
    except:
        set_serial('COM4')
        t1 = threading.Thread(target=U_receive)
        t1.start()
        #t1.join() 
        print(1)
        main()
