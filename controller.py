import roslib
import rospy
#from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool, Float64
import time
from sensor_msgs.msg import Image
import zmq
import pickle
import json
from threading import Thread

from tqdm import tqdm
import math
from collections import deque

from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed



rospy.init_node('controller', anonymous=True)

global socket
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.bind("tcp://*:8000")

# Coordinates of bounding box
bbox = None

######################################################################
# PID Classes and functions
vehicle_velocity = 0

# Constants
SYSTEM_LAG = 0.3 #Value in seconds
SHIFT_THRESHOLD = 0.01 #Vehicle velocity at which we decide if gear is fwd/back
IMAGE_HEIGHT = 1440
IMAGE_WIDTH = 1920
IMAGE_DEPTH = 3

heights_list = deque([], 5)
centers_list = deque([], 5)


class PID:
    def __init__(self, kp, kd, ki):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.reset()

    def reset(self):
        self.last_e = None
        self.last_t = None
        self.integ_e = 0.

    def __call__(self, err, dt=None):
        now_t = time.time()
        self.last_t = self.last_t or now_t
        self.last_e = self.last_e or err
        mydt = dt or (now_t - self.last_t)
        mydt = max(mydt, 1e-6)
        err_dot = (err - self.last_e) / mydt
        self.integ_e = self.integ_e + err * mydt
        if abs(err) > 0.3:
            self.integ_e = 0
        kp_term = self.kp * err
        kd_term = self.kd * err_dot
        ki_term = self.ki * self.integ_e
        actuation = kp_term + kd_term + ki_term

        print("Current error: "+str(err)+", error dot: "+str(err_dot)+", integ error: "+str(self.integ_e))

        if abs(actuation) > 7.0:
            if actuation <= 0:
                actuation = -7.0
            else:
                actuation = 7.0

        self.last_t = now_t
        self.last_e = err
        return actuation, kp_term, kd_term, ki_term, self.integ_e

# Declaring publishers for the vehicle
topic_prefix = '/pacmod/as_rx/'
pub_cmd_topic = {'brake': (topic_prefix+'brake_cmd', PacmodCmd),
                 'accel': (topic_prefix+'accel_cmd', PacmodCmd),
                 'turn': (topic_prefix+'turn_cmd', PacmodCmd),
                 'gear': (topic_prefix+'shift_cmd', PacmodCmd),
                 'steer': (topic_prefix+'steer_cmd', PositionWithSpeed),
                 'enable': (topic_prefix+'enable', Bool)}

pub_enable = rospy.Publisher(*pub_cmd_topic['enable'], queue_size=1)
pub_brake = rospy.Publisher(*pub_cmd_topic['brake'], queue_size=1)
pub_accel = rospy.Publisher(*pub_cmd_topic['accel'], queue_size=1)
pub_turn = rospy.Publisher(*pub_cmd_topic['turn'], queue_size=1)
pub_gear = rospy.Publisher(*pub_cmd_topic['gear'], queue_size=1)
pub_steer = rospy.Publisher(*pub_cmd_topic['steer'], queue_size=1)

vehicle_velocity = 0
GEAR = PacmodCmd.SHIFT_FORWARD
    

def talker():
    global heights_list, GEAR
    global centers_list
    pid_steering = PID(5, 0.1, 0)

    rate = rospy.Rate(30) #TODO: Increase this

    pid_speed = PID(12.0, 0.2, 10.0)

    while not rospy.is_shutdown():
        if bbox == None:
            continue

        DESIRED_HEIGHT = 550
        box_height = bbox[3] - bbox[1]

        heights_list.append((box_height, time.time()))

        if len(heights_list) > 3:
            box_height += SYSTEM_LAG*(heights_list[-2][0] - heights_list[-3][0])/(heights_list[-2][1] - heights_list[-3][1])

        error = float(DESIRED_HEIGHT - box_height)/100.0
        actuation, _, _, _, _ = pid_speed(error)
        
       # print("Current height is: "+str(box_height))
       # print("Current actuation is: "+str(actuation))  
       # print("Current velocity is:"+str(vehicle_velocity))
       # print("Current error is " +str(error*100))
        if GEAR == PacmodCmd.SHIFT_FORWARD:
            print("Current gear is: Forward")
        else:
            print("Current gear is: Reverse")

        pub_enable.publish(True)

        
        # Go forward or backward
        average_height = 0
        for i in range(len(heights_list)):
            average_height += heights_list[i][0]

        average_height = float(average_height)/len(heights_list)

        if abs(vehicle_velocity) < SHIFT_THRESHOLD and abs(DESIRED_HEIGHT - average_height) > 100:
            if actuation > 0 :
                GEAR = PacmodCmd.SHIFT_FORWARD
            else:
                GEAR = PacmodCmd.SHIFT_REVERSE

        if abs(vehicle_velocity) > 0.3:
            actuation = 0.3
            pub_enable.publish(True)
            pub_gear.publish(ui16_cmd=PacmodCmd.SHIFT_FORWARD, enable=True)
            pub_accel.publish(f64_cmd=actuation, enable=True) 
            pub_brake.publish(f64_cmd=0.0, enable=True)       
            #print("Current velocity "+str(vehicle_velocity)+" accel signal = "+str(actuation))
            #print("###############################################")
            #print("###############################################")

        else: 
            if actuation > 0 and GEAR == PacmodCmd.SHIFT_FORWARD or actuation < 0 and GEAR == PacmodCmd.SHIFT_REVERSE:

                # Cap the actuation so we don't accelerate too much
                if abs(actuation) > 0.1:
                    actuation = 0.1

                # Minimum acceleration for car to move
                ACC_OFFSET = 0.32    
                actuation = actuation + ACC_OFFSET
                
                pub_enable.publish(True)
                pub_gear.publish(ui16_cmd=GEAR, enable=True)
                pub_accel.publish(f64_cmd=actuation, enable=True) 
                pub_brake.publish(f64_cmd=0.0, enable=True)         
                print("Sent acceleration signal: "+str(actuation))
                print("Sent brake signal: "+str(0.0))
            
            else:
                pub_enable.publish(True)
                actuation_acc = 0.0
                BRAKE_OFFSET = 0.15
                actuation = actuation * 1

                if abs(actuation) < 0.1:
                    actuation = 0.0

                if abs(actuation) > 0.9:
                    actuation = 0.9
                    actuation_acc = 0.0

                #actuation = 0.0
                pub_enable.publish(True)
                pub_brake.publish(f64_cmd=actuation+BRAKE_OFFSET, enable=True)
                pub_accel.publish(f64_cmd=actuation_acc, enable=True)           
                print("Sent brake signal: "+str(actuation))   
                print("Sent acceleration signal: "+str(actuation_acc))         

        print("---------------------------------------")
        print("")

        print("STEERING NOW ")
        

        DESIRED_OFFSET = IMAGE_WIDTH//2
        box_center = (bbox[2] + bbox[0])/2
        centers_list.append((box_center, time.time()))

        if len(centers_list) > 3:
            box_center += SYSTEM_LAG*(centers_list[-2][0] - centers_list[-3][0])/(centers_list[-2][1] - centers_list[-3][1])

        error = float(DESIRED_OFFSET - box_center)/100.0
        steering_act, _, _, _, _ = pid_steering(error)

        angular_velocity_limit = 2  
        if GEAR == PacmodCmd.SHIFT_REVERSE:
            steering_act = -1.0*steering_act
        pub_steer.publish(
            angular_position=steering_act, angular_velocity_limit=angular_velocity_limit
        )
        print("Person is at " + str((bbox[0] + bbox[1])/2))
        print("Steering error: " + str(error*100.0))
        print("Steering act: " + str(steering_act))

        rate.sleep()

####################################################################
# Listeners
def listener():
    global socket

    print("Listener Started")

    def callback(data):
        global socket
        #print("Sending image on socket")
        socket.send(data.data)

    rospy.Subscriber("/mako_1/mako_1/image_raw", Image, callback)
    rospy.spin()

def detector_listener():
    global socket, bbox
    
    while True:
        bbox_json = json.loads(socket.recv().decode())
        x1, y1, x2, y2 = bbox_json.get("x1"), bbox_json.get("y1"), bbox_json.get("x2"), bbox_json.get("y2")
        
        bbox = [x1, y1, x2, y2]
        #print("Person at :"+str(x1))

def velocity_listener():
    
    def set_speed_on_vehicle(data):
        global vehicle_velocity
        vehicle_velocity = data.data    
    
    rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, set_speed_on_vehicle)  
    rospy.spin()

def enabler():
    while True:
        pub_enable.publish(True)

#####################################################################
# Starting threads
detector_thread = Thread(target=detector_listener)
detector_thread.daemon = True
detector_thread.start()

talker_thread = Thread(target=talker)
talker_thread.daemon = True
talker_thread.start()

velocity_thread = Thread(target=velocity_listener)
velocity_thread.daemon = True
velocity_thread.start()

enabler_thread = Thread(target=enabler)
enabler_thread.daemon = True
enabler_thread.start()

#####################################################################
# Main Stuff
pub_enable.publish(True)
pub_enable.publish(True)

listener()
