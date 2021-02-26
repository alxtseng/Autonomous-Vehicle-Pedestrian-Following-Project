import zmq
import numpy as np
import pickle
import json
from imageai.Detection import ObjectDetection

# Mako Camera Image Constants
IMAGE_HEIGHT = 1440
IMAGE_WIDTH = 1920
IMAGE_DEPTH = 3

# Socket setup
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.connect("tcp://localhost:8000")


if __name__ == "__main__":
    images = 0
     # Setup object detector
    detector = ObjectDetection()
    detector.setModelTypeAsTinyYOLOv3()
    detector.setModelPath("./yolo-tiny.h5")
    detector.loadModel("fast")
    print("Model sent to gPU")

    while True:
        # Wait for an image buffer
        image_buffer = socket.recv()

        # Read the image from the buffer
        im = np.frombuffer(
            image_buffer[: IMAGE_HEIGHT * IMAGE_WIDTH * IMAGE_DEPTH], dtype=np.uint8
        ).reshape(IMAGE_HEIGHT, IMAGE_WIDTH, -1)
        
        if images % 5 == 0:
            print("Image {} recieved".format(images))
        images += 1

        image_out, objects = detector.detectObjectsFromImage(
            input_type="array",
            input_image=im,
            output_type="array",
            minimum_percentage_probability=60,
        )
        #print(objects)
        bounding_box_coords = None
        for obj in objects:
            if obj["name"] == "person":
                print("Person Detected")
                bounding_box_coords = obj["box_points"]
                x1, y1, x2, y2 = obj["box_points"]
                print(bounding_box_coords)
                break

        if bounding_box_coords:
            data = json.dumps({"x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2)})
            socket.send(data.encode())