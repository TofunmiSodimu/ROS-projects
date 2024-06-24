# import the opencv library
import cv2
import numpy as np 
  
# define a video capture object
vid = cv2.VideoCapture(0)
  
while(True):
      
    # Capture the video frame by frame
    ret, frame = vid.read()

    # Blur image
    blur_img = cv2.medianBlur(frame,5)

    # Convert image to grayscale
    gray_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)

    # Use houghcircles function to detect circles in image
    circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT,1,90,param1=70,param2=60,minRadius=0,maxRadius=0)
    
    # Print out detected circles
    if(circles is not None):
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            #draw the outer circle (image, center coordinates, radius, color, thickness)
            cv2.circle(gray_img, (i[0], i[1]), i[2], (0,255,0),3)
            #draw the center of the circle
            cv2.circle(gray_img,(i[0],i[1]),2,(0,0,255),3)
            #print out pixel coordinates of center of ball
            print("Circle center coordinates:", i[0], ",", i[1])

        # Display images with detected circles
        cv2.imshow('detected circles',gray_img)

    # the 'e' button is set as the quitting button
    if cv2.waitKey(1) & 0xFF == ord('e'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()