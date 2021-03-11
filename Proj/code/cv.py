#!/usr/bin/env python
# encoding: utf-8

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2

font = cv2.FONT_HERSHEY_SIMPLEX

# cnt_index 0-green 1-red 2-blue 3-yellow 4-purple
# hsv color threshold value
# green
lower_green = np.array([35, 110, 206])
upper_green = np.array([77, 255, 255])

# red
lower_red = np.array([0, 127, 128])
upper_red = np.array([10, 255, 255])

# blue
lower_blue = np.array([100, 43, 46])
upper_blue = np.array([124, 255, 255])

# yellow
lower_yellow = np.array([26, 110, 106])
upper_yellow = np.array([34, 255, 255])

# purple
lower_purple = np.array([125, 43, 46])
upper_purple = np.array([155, 255, 255])

if __name__ == "__main__":
    node_name = 'cv_publisher'
    rospy.init_node(node_name, anonymous=True)
    
    pub = rospy.Publisher(
        'cv_position_publisher',
        Float64MultiArray,
        queue_size=10
    )

    # Use opencv to capture video information from the camera 
    cap = cv2.VideoCapture(1)
    # Flag bit, set 150 when the position of the block is sent, as the camera FPS is 30
    flag = 0

    while rospy.is_shutdown() == False and cap.isOpened():
        ret, frame = cap.read()
        # If the position has been sent, skip the following 5s
        if(flag > 0):
            flag -= 1
            continue
    
        if ret == False:
            break

        # Convert RGB info. to HSV, and use color mask to distinguish
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
       
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        mask_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        mask_purple = cv2.inRange(hsv_img, lower_purple, upper_purple)

        # Median filter
        mask_green = cv2.medianBlur(mask_green, 7)
        mask_red = cv2.medianBlur(mask_red, 7)
        mask_blue = cv2.medianBlur(mask_blue, 7)
        mask_yellow = cv2.medianBlur(mask_yellow, 7)
        mask_purple = cv2.medianBlur(mask_purple, 7)
        
        # Extract contour info. based on color info.
        contours1, hierarchy1 = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours2, hierarchy2 = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours3, hierarchy3 = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours4, hierarchy4 = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours5, hierarchy5 = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Find the contour with maximum color info. in the frame, i.e. the color of the block
        cnts = [contours1, contours2, contours3, contours4, contours5]
        cnts_len = [len(contours1), len(contours2), len(contours3), len(contours4), len(contours5)]
        cnts_index = cnts_len.index(max(cnts_len))

        for cnt in cnts[cnts_index]:
            # Filter contours with insufficent info.
            if(len(cnt) < 100): continue

            # Use rectangle to fit the contour, i.e. to get the shape of the block
            (x, y, w, h) = cv2.boundingRect(cnt)  
            # Use circle to fit the contour
            # (x, y), radius = cv2.minEnclosingCircle(cnt)

            # The position of 
            x = int(x)
            # Use linear fitting function to transform coordinates from the camera coordinate system 
            # to the mechanical arm coordinate system
            y_real = 0.0316*y + 4.2561
            y = int(y)

            # Draw a rectangle on the extracted contour
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(frame, "point", (x, y - 5), font, 0.7, (0, 255, 0), 2)

            # Return for the central point of the extracted contour as the position of the block
            x_center = x + w//2
            y_center = y_real + h//2

            # When the block position reaches the boundary
            if(x_center >= 290):
                flag = 150
                y_final = y_real
                color = cnts_index
                data = Float64MultiArray()
                data.data = [y_final+16, color]
                pub.publish(data)
                print(y_final, color)
                break

        cv2.imshow("color_dection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()