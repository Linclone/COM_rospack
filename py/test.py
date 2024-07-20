# UAV_interface.py
class UAVInterface:
    def __init__(self):
        self.data_received_event = Event() 

    def process_data(self, data):
        self.data_received_event.set() 

from UAV_interface import UAVInterface

uav_interface = UAVInterface()

uav_interface.data_received_event.set()  

