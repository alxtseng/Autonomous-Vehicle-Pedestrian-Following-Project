#!/usr/bin/env python

import os
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



if __name__ == '__main__':
    model = maskrcnn_resnet50_fpn(pretrained=True)
    bridge = CvBridge()

    print("Reached here")
    # Load checkpoint
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    #model.load_state_dict(torch.load('checkpoints/classifier.pt', map_location=device))
    model.eval()
    print(device)

    with open("data/img.pkl", "rb") as pkl_in:
        data = pickle.load(pkl_in)
        #height, width, data = pickle.load(pkl_in)

    #data_ints = [ord(i) for i in data]
    #data_img = np.array(data_ints).reshape((height, width, 3))/255.0
    data_img = np.asarray(bridge.imgmsg_to_cv2(data, desired_encoding='bgr8'))/255.0
    data_tensor = [transforms.ToTensor()(data_img).float().to(device)]
    print(data_tensor[0].shape)

    with torch.no_grad():
        prediction = model(data_tensor)

    # Plotting
    fig, (ax1, ax2) = plt.subplots(1, 2)

    ax1.imshow(data_img)
    ax1.set_title('Raw image')

    for i in range(len(prediction)):
        x0, y0, x1, y1 = prediction[i]['boxes'][0]
        mask = prediction[i]['masks'][0, 0].mul(255).byte().cpu().numpy()
        person_detected = False
        for prediction_idx, label in enumerate(prediction[i]['labels']):
            if label == 1: 
                score = prediction[i]['scores'][prediction_idx].item()
                person_detected = True
                ax2.imshow(Image.fromarray(mask))
                rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, linewidth=1, edgecolor='r')
                ax2.add_patch(rect)
                ax2.annotate('score: {:.3}'.format(score), (x0, y0), color='w')
                ax2.set_title('Prediction')
                break
        
        score = int(person_detected)

    plt.savefig('results/test.png')
