import roslib
import rospy
from std_msgs.msg import Bool, Float64
import time
from tqdm import tqdm
import time, select
import math

from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates 


positions = [0,0,0]
velocities = [0,0,0]


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
        self.integ_e = self.integ_e + (err - self.last_e) * mydt
        kp_term = self.kp * err
        kd_term = self.kd * err_dot
        ki_term = self.ki * self.integ_e
        actuation = kp_term + kd_term + ki_term

        if abs(actuation) > 7.0:
            if actuation <= 0:
                actuation = -7.0
            else:
                actuation = 7.0

        self.last_t = now_t
        self.last_e = err
        return actuation, kp_term, kd_term, ki_term, self.integ_e

def position_callback(data):
    global positions
    gem_index = data.name.index('gem')
    positions[0] = data.pose[gem_index].position.x
    positions[1] = data.pose[gem_index].position.y
    positions[2] = data.pose[gem_index].position.z

    velocities[0] = data.twist[gem_index].linear.x
    velocities[1] = data.twist[gem_index].linear.y
    velocities[2] = data.twist[gem_index].linear.z

def norm(v):
    return math.sqrt(v[0]**2 + v[1]**2)

def dot(a,b):
    return a[0]*b[0] + a[1]*b[1]


def talker():
    pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10)
    rospy.init_node('speeder', anonymous=True)
    rate = rospy.Rate(10)
    pid_steer = PID(70,0.2,0.1)

    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #pub.publish(speed=2)
        desired_heading = -math.pi/4
        current_angle = velocities

        angle = math.atan2(velocities[1], velocities[0])
        print((angle-desired_heading)*180/math.pi)

        acuation, _, _, _, _ = pid_steer(angle - desired_heading)
        print("actuation")
        print(acuation)
        print("---------------------")

        pub.publish(steering_angle=acuation, speed= 3)

        rate.sleep()

if __name__ == '__main__':
    rospy.Subscriber('/gazebo/model_states', ModelStates, position_callback)
    talker()
