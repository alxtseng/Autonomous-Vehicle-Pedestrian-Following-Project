import roslib
import rospy
#from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool, Float64
import time
from sensor_msgs.msg import Image as ImageMsg
import pickle

from PIL import Image
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import torch
import torchvision.transforms as transforms
from torchvision.models.detection import maskrcnn_resnet50_fpn
from cv_bridge import CvBridge
import cv2

global counter, model, device, bridge


#model.to(device)

def callback(data):
    global counter, model, device, bridge
    data_img = np.asarray(bridge.imgmsg_to_cv2(data, desired_encoding='bgr8'))/255.0
    data_tensor = [transforms.ToTensor()(data_img).float().to(device)]


    print("Start of function")

    counter += 1
    print(counter)

    #print(data_tensor[0].shape)

    torch.cuda.empty_cache()

    if counter > 1:
        return 

    with torch.no_grad():
        prediction = model(data_tensor)
        del data_img
        del data_tensor

    # Plotting
    #fig, (ax1, ax2) = plt.subplots(1, 2)

    #ax1.imshow(data_img)
    #ax1.set_title('Raw image')

    # for i in range(len(prediction)):
    #     x0, y0, x1, y1 = prediction[i]['boxes'][0]
    #     mask = prediction[i]['masks'][0, 0].mul(255).byte().cpu().numpy()
    #     person_detected = False
    #     score = 0
    #     for prediction_idx, label in enumerate(prediction[i]['labels']):
    #         if label == 1: #person 
    #             score = prediction[i]['scores'][prediction_idx].item()
    #             person_detected = True
    #             #ax2.imshow(Image.fromarray(mask))
    #             rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, linewidth=1, edgecolor='r')
    #             #ax2.add_patch(rect)
    #             #ax2.annotate('score: {:.3}'.format(score), (x0, y0), color='w')
    #             #ax2.set_title('Prediction')
    #             break
    
    #global counter
    rospy.loginfo("I got an image from %s", rospy.get_caller_id())
    
    score=0

    if score > 0.5:
        rospy.loginfo("Person Detected")
        print("Braking")
        #pub_brake.publish(f64_cmd=2.0, enable=True, ignore=False, clear=False)     
    else:
        print("Not braking")
        #pub_brake.publish(f64_cmd=0.0, enable=True, ignore=False, clear=False)

    # print data.data   

# Declaring the code as a node
rospy.init_node('turn_node', anonymous=True)

# Dictionary for topics
topic_prefix = '/pacmod/as_rx/'
# pub_cmd_topic = {'brake': (topic_prefix+'brake_cmd', PacmodCmd),
#                  'accel': (topic_prefix+'accel_cmd', PacmodCmd),
#                  'turn': (topic_prefix+'turn_cmd', PacmodCmd),
#                  'gear': (topic_prefix+'shift_cmd', PacmodCmd),
#                  'steer': (topic_prefix+'steer_cmd', PositionWithSpeed),
#                  'enable': (topic_prefix+'enable', Bool)}

# Making the publishers
# pub_enable = rospy.Publisher(*pub_cmd_topic['enable'], queue_size=1)
# pub_brake = rospy.Publisher(*pub_cmd_topic['brake'], queue_size=1)
# pub_accel = rospy.Publisher(*pub_cmd_topic['accel'], queue_size=1)
# pub_turn = rospy.Publisher(*pub_cmd_topic['turn'], queue_size=1)
# pub_gear = rospy.Publisher(*pub_cmd_topic['gear'], queue_size=1)
# pub_steer = rospy.Publisher(*pub_cmd_topic['steer'], queue_size=1)

# Get control to computer
# pub_enable.publish(True)



# Listener 
def listener():
    rospy.Subscriber('/mako_1/mako_1/image_raw', ImageMsg, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
	global counter, model, device, bridge
	counter = 0
	model = maskrcnn_resnet50_fpn(pretrained=True)
	bridge = CvBridge()
	device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
	model.to(device)
	model.eval()
	print(device)
	listener()

# # Indicator
# while not rospy.is_shutdown():  
#     # pub_turn.publish(ui16_cmd=PacmodCmd.TURN_RIGHT, enable=True, ignore=False, clear=False)
#     print "Published"   
    
#     r.sleep()

# Making a subscriber for image data

