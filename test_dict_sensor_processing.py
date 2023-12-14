#!/usr/bin/env python
import numpy as np
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '../..'))
from std_msgs.msg import Int16, UInt32, String, Empty, Bool
import configparser
import rospy
import socket
import time
import struct
from sensors.srv import vacuum, bar

'''
bit 6 = vacuum
bit 7 = bar
bit 8 = 
'''

class SENSOR_PROCESSING:
    def __init__(self, host, port):
        # node name
        rospy.init_node('sensorProcessing', anonymous=False)
        rospy.Service('vacuum_control', vacuum, self.handle_vacuum_switch)
        rospy.Service('bar_control', bar, self.handle_bar_switch)
        rospy.loginfo("Vacuum service node is ready.")

        config = configparser.ConfigParser()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', '..', 'conf.ini')
        config.read(os.path.abspath(config_path))
        self.loginfo = config['sensorprocessing']['loginfo']
        self.host = host
        self.port = port

        self.unpacked_data = None
        self.packed_data = [None,None, None,None,None, None, None, None]

        self.vac = None
        self.extend = None
        self.retract = None
        self.real_value = None

        
        # self.patterns = [[0, 0, 0, 0]]
        self.patterns = [87, 68, 0, 0, 0,]
         # publishing rate
        self.pub_sensor_rate = float(config['sensorprocessing']['publish_rate']) 


    def handle_vacuum_switch(self, req):
        if req.switchVacuum == 1:
            rospy.loginfo("switchVacuum set to 1, Responding with True")
            # self.vac = 1
            self.packed_data[2] = 1
            return True
        elif req.switchVacuum == 0:
            rospy.loginfo("switchVacuum set to 0, Responding with True")
            self.packed_data[2] = 0
            return True
        else:
            rospy.loginfo("Invalid rosservice arguments")
            return False

    def handle_bar_switch(self, req):
        if req.toggleState == 1:
            rospy.loginfo("bar toggleState set to extend, Responding with True")
            self.packed_data[3] = 1
            self.packed_data[4] = 0
            return True
        elif req.toggleState == -1:
            rospy.loginfo("bar toggleState set to retract, Responding with True")
            self.packed_data[3] = 0
            self.packed_data[4] = 1
            return True
        elif req.toggleState == 0:
            rospy.loginfo("bar toggleState set to stop, Responding with True")
            self.packed_data[3] = 0
            self.packed_data[4] = 0
            return True
        else:
            rospy.loginfo("Invalid rosservice arguments")
            return False


    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_sensor_rate), self.socket_publish)
        rospy.spin()

    def socket_publish(self, event=None):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            
            ################### BELOW LINE IS FOR testing connection timing out successfully. ##########
            # server_socket.settimeout(5)  # Set a timeout of 5 seconds
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            server_socket.bind((self.host, self.port))
            server_socket.listen()
            print(f'Server started on {self.host}:{self.port}')
            conn, addr = server_socket.accept()
            with conn:
                print('Connected by', addr)
                running = True
                while running:
                    # for pattern in self.patterns:
                    #     lenSend = conn.sendall(bytearray(pattern))


                    total_data = b''
                    buffer_size = 11 
                    while len(total_data) < buffer_size:
                        chunk = conn.recv(buffer_size - len(total_data))
                        if not chunk:
                            break
                        total_data += chunk

                    if len(total_data) == buffer_size:
                        print(f"Received data: {total_data}")
                        
                        if total_data[0] == 68 and total_data[1] == 43:
                            format_string = '=2B4BfB'  # This should match a total of 11 bytes
                            self.unpacked_data = struct.unpack(format_string, total_data)
                            # packed_data = struct.pack(format_string, total_data)

                            print("unpacked_data: ", self.unpacked_data)

                            # Extracting individual parts
                            watchdog = self.unpacked_data[:2]
                            # bool_values = [bool(val) for val in unpacked_data[2:6]]  # Adjusted for 4 boolean values
                            self.vac = self.packed_data[2]
                            self.extend = self.packed_data[3]
                            self.retract = self.packed_data[4]
                            self.real_value = self.unpacked_data[6]  # Adjusted index for the real value
                            checksum = self.unpacked_data[-1]

                            self.packed_data[:2] = self.unpacked_data[:2]
                            self.packed_data[6] = self.unpacked_data[6]

                            print("vac: ", self.vac)
                            print("battery: ", self.real_value)
                            print("packed_data: ", self.packed_data)

                    conn.sendall(bytearray(total_data))  ############## chnage this and send the chnaged array after packing

                    # except socket.timeout:
                    #     print("Timed out waiting for acknowledgment!")
                    #     running = False
                    #     break
                    # except (socket.error, BrokenPipeError):
                    #     print("Error: Connection to client lost.")
                    #     conn.close()
                    #     sys.exit(1)
    
                    # time.sleep(.2)


'''

                        # conn.recv(15)

                        print(f"Sent pattern: {pattern}")
                        print("Waiting for acknowledgment...")
        
                        try:
                            ack = conn.recv(16)
                            if not ack:
                                print("No acknowledgment received!")
                                running = False
                                break
                            else:
                                # if len(ack) == 15:
                                
                                ack_convert = struct.unpack('16B', ack)
                                if ack_convert[0] == 43 and ack_convert[1] == 39:
                                    print(f"Received acknowledgment: {ack_convert}")

'''








if __name__ == "__main__":
    try:
        HOST = '192.168.7.10'
        PORT = 50001
        server = SENSOR_PROCESSING(HOST, PORT)
        rospy.loginfo("sensor_processing_v2.py: Waiting for 3 sec. to load the all the files...")
        rospy.sleep(3)
        server.run()
    except rospy.ROSInterruptException:
        pass    
