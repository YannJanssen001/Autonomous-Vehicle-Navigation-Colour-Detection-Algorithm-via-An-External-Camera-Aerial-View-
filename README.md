# Autonomous-Vehicle-Navigation-Colour-Detection-Algorithm-via-An-External-Camera-Aerial-View-

Abstract:
This project aimed to develop a navigation system for a Mars rover to autonomously navigate from
Point A to Point B using colour detection technology. An external camera captured aerial images
of the rover and a target object, both identified by a distinct green colour, simulating the green lab
notebooks. These images underwent a colour detection algorithm to determine the coordinates of
the rover and target object in the camera's plane. The algorithm, implemented on a Raspberry Pi,
converted pixel distances into real-world measurements, enabling precise navigation. Results
showed a linear relationship between distances in the camera's plane and their real-world
counterparts, determined through linear interpolation. The navigation system algorithm was run
multiple times for three arbitrary path routes, with the rover averaging 16.9% off from the centre
point of the target object's coordinates. Despite this deviation, the rover successfully reached the
vicinity of the target object in all tests. This outcome demonstrates the system's effectiveness in
reaching intended destinations within an acceptable margin. This project showcases the feasibility
of autonomous robotic vehicle navigation using colour detection technology and offers valuable
insights for subsequent research endeavours to build upon and progress further.

Explanation of Code [colordetection.py - Code Created by Yann Janssen]: This code uses OpenCV and NumPy to detect green objects in an image and calculate the distance between their centers. The ColorDetector class loads an image, converts it to HSV color space, and creates a mask to isolate green areas. It then finds contours in the mask, sorts them by area, and draws bounding rectangles around the two largest contours. The CenterDistanceCalculator class calculates the center points of these rectangles and the distance between them. The script then prints the calculated distances and uses the paramiko library to send this data to a Raspberry Pi over SSH.

Explanation of Code [MovingAlgorithm.py - Code Created by Yann Janssen]: The code below the RobotMovement class is the default code for controlling a vehicle's movement on a Raspberry Pi. It includes functions for motor control and setup. The RobotMovement class, constructed using the color detection algorithm from the first code block, calculates travel coordinates for the robot. It reads pixel distances from a file, calculates gradients, and interpolates times for movement. The class includes methods for moving forward, backward, and turning. The script creates an instance of RobotMovement, moves the robot to specified coordinates, and handles safe termination on a keyboard interrupt.


Group Project Members: Yann Raphael Janssen, Rafid Hossain, Leyan Ouyang, Adrian Murselaj, and Antoine Valadon. 

Yann Raphael Janssen (myself) was the group leader and created the codes provided above.
Leyan Ouyang was in charge of implementing a code for an external camera to capture images and transmit data between devices.
Rafid Hossain and Adrian Murselaj were in charge of hardware components of the experiment, and research.
Antoine Valadon suggested on what types of navigation algorithms to use and provided further improvements (Orientation of the Rover).
Everyone contributed to conducting the experiment in the methodology section: distance conversion rate of the paper (section II.4), and writing the lab report.
