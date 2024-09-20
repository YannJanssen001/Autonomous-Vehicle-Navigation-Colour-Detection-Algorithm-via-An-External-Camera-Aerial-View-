import cv2
import numpy as np
import math
import time

class ColorDetector:
    def __init__(self, image_path):
        self.image_path = image_path
        self.rectangles = []

    def detect_color(self):
        # Load the image
        image = cv2.imread(self.image_path)

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color range for darker green in HSV
        lower_green = np.array([36, 100, 50])
        upper_green = np.array([86, 255, 150])

        # Threshold the HSV image to get only green colors
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours in the green mask
        contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area in descending order and take the first two
        largest_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

        for i, contour in enumerate(largest_contours):
            # Calculate the bounding rectangle for the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Store the corners of the bounding rectangle
            rectangle = {
                "top_left": (x, y),
                "top_right": (x + w, y),
                "bottom_left": (x, y + h),
                "bottom_right": (x + w, y + h),
            }
            self.rectangles.append(rectangle)

            # Draw the rectangle on the image
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

       # Show the image with the rectangles
        cv2.imshow('Image with rectangles', image)
        cv2.waitKey(5000) # 1000 milliseconds = 1 second
        cv2.destroyAllWindows()

        return self.rectangles


class CenterDistanceCalculator:
    @staticmethod
    def center_point(rectangle):
        x = (rectangle['top_left'][0] + rectangle['top_right'][0]) / 2
        y = (rectangle['top_left'][1] + rectangle['bottom_left'][1]) / 2
        return [x, y]

    @staticmethod
    def distance(point1, point2):
        x_distance = point2[0] - point1[0]
        y_distance = point2[1] - point1[1]
        total_distance = math.sqrt(x_distance**2 + y_distance**2)
        return x_distance, y_distance, total_distance


# Usage
detector = ColorDetector('rrafid.jpg')
rectangles = detector.detect_color()

calculator = CenterDistanceCalculator()

robot_center = calculator.center_point(rectangles[0])
object_center = calculator.center_point(rectangles[1])

x_distance, y_distance, total_distance = calculator.distance(robot_center, object_center)

print(f"Robot center point: {robot_center}")
print(f"Object center point: {object_center}")
print(f"x distance: {x_distance}")
print(f"y distance: {y_distance}")
print(f"Total distance: {total_distance}")

import paramiko

# Prepare the data to be sent
data_to_send = f"{x_distance},{y_distance},{total_distance}"

# Create a new SSH client
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

# Connect to the Raspberry Pi
ssh.connect(hostname='raspberrypi.local', username='Mars2024', password='Patrick123')

# Send the data to the Raspberry Pi
stdin, stdout, stderr = ssh.exec_command(f"echo '{data_to_send}' > ~/pixel_distances.txt")
print("Output:", stdout.read())
print("Error:", stderr.read())

# Close the SSH connection
ssh.close()