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
        # self.patterns = [[0, 0, 0, 0]]
        self.patterns = [[87, 68, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]]
         # publishing rate
        self.pub_sensor_rate = float(config['sensorprocessing']['publish_rate']) 


    def handle_vacuum_switch(self, req):
        if req.switchVacuum == 1:
            rospy.loginfo("switchVacuum set to 1, Responding with True")
            # self.patterns = [[87, 68, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 ]]
            self.patterns[0][6] = 1
            return True
        elif req.switchVacuum == 0:
            rospy.loginfo("switchVacuum set to 0, Responding with True")
            self.patterns[0][6] = 0
            return True
        else:
            rospy.loginfo("Invalid rosservice arguments")
            return False

    def handle_bar_switch(self, req):
        if req.toggleState == 1:
            rospy.loginfo("bar toggleState set to extend, Responding with True")
            self.patterns[0][7] = 1
            self.patterns[0][8] = 0
            return True
        elif req.toggleState == -1:
            rospy.loginfo("bar toggleState set to retract, Responding with True")
            self.patterns[0][8] = 1
            self.patterns[0][7] = 0
            return True
        elif req.toggleState == 0:
            rospy.loginfo("bar toggleState set to stop, Responding with True")
            self.patterns[0][8] = 0
            self.patterns[0][7] = 0
            return True
        else:
            rospy.loginfo("Invalid rosservice arguments")
            return False

    def socket_publish(self, event=None):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            
            ################### BELOW LINE IS FOR testing connection timeing out successfully. ##########
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
                    for pattern in self.patterns:
                        lenSend = conn.sendall(bytearray(pattern))

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
                        except socket.timeout:
                            print("Timed out waiting for acknowledgment!")
                            running = False
                            break
                        except (socket.error, BrokenPipeError):
                            print("Error: Connection to client lost.")
                            conn.close()
                            sys.exit(1)
        
                        time.sleep(.2)


    def run(self):
        rospy.Timer(rospy.Duration(1. / self.pub_sensor_rate), self.socket_publish)
        rospy.spin()


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

