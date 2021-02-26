**Autonomous-Vehicle-Pedestrian-Following-Project**

This was my final project for my graduate level autonomous vehicle course that I worked on with Kartikeya Sharma, Samir Wadhwa, and Adam Stewart.

Our final project was getting an autonomous vehicle to follow a person at a given distance. We used the camera data and vehicle speed reported by the vehicle. We implemented a pedestrian detector (tiny-YOLO) and a method to forecast the bounding box based on earlier predictions (to account for camera lag). We used PID control to ensure the vehicle remains at a constant distance (within a threshold) from the detected pedestrian, and also to maintain the pedestrian at the center of the frame (for steering). The vehicle was able to move forward as the pedestrian moved away, reverse as the pedestrian moved closer, and steer left/right (for both forward and reverse motion) to ensure the pedestrian was in the center of the frame, with the car pointed towards them.

Some links:

Outside view: https://www.youtube.com/watch?v=2sJhYTt34Bw
Inside view 1: https://www.youtube.com/watch?v=JO78qfSGBYI
Inside view 2: https://www.youtube.com/watch?v=egU0Jge1wMk
