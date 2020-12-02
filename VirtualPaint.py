import cv2
import numpy as np
from collections import deque

blue_lower = np.array([0,113,186])          #range for detecting orange sticker 
blue_upper = np.array([30,255,255])

kernel = np.ones((5,5), np.uint8)           #kernel for Masking
         
blueIndex = 0

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)] 
colorIndex = 0

utencilType = 4

paintWindow = np.zeros((471,636,3)) + 255       #White screen
paintWindow = cv2.rectangle(paintWindow, (40,10), (140,65), (0,0,0), 2)      #Button for Clear all   
paintWindow = cv2.rectangle(paintWindow, (160,10), (255,65), colors[0], -1)  #Button for Blue Color
paintWindow = cv2.rectangle(paintWindow, (275,10), (370,65), colors[1], -1)
paintWindow = cv2.rectangle(paintWindow, (390,10), (485,65), colors[2], -1)
paintWindow = cv2.rectangle(paintWindow, (505,10), (600,65), colors[3], -1)
paintWindow = cv2.rectangle(paintWindow, (40,85), (140,140), (0,0,0), 2)
paintWindow = cv2.rectangle(paintWindow, (40,160), (140,215), (0,0,0), 2)


cv2.putText(paintWindow, "CLEAR ALL", (49,43), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons
cv2.putText(paintWindow, "BLUE", (185, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
cv2.putText(paintWindow, "GREEN", (298, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
cv2.putText(paintWindow, "RED", (420, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
cv2.putText(paintWindow, "YELLOW", (520, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.LINE_AA)
cv2.putText(paintWindow, "MARKER", (57,117), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons
cv2.putText(paintWindow, "PEN", (72,192), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons


cv2.namedWindow('Paint' , cv2.WINDOW_AUTOSIZE)      #creates window

color_setting = (255,255,255)

cap = cv2.VideoCapture(0)                            #Capture from Webcam (0)    
cap.set(3, 680)    #3. CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
cap.set(4,450)     #4. CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
#cap.set(10,130)    #10. CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).

def colorSet(x2,y2):
    if 160 < x2 < 255 and 1 < y2 < 65:
            color_setting = (colors[0])
    elif 275 < x2 < 370 and 1 < y2 < 65:
        color_setting = (colors[1])
    elif 390 < x2 < 485 and 1 < y2 < 65:
        color_setting = (colors[2])
    elif 505 < x2 < 600 and 1 < y2 < 65:
        color_setting = (colors[3])
    return color_setting
    



x1,y1 = 0,0
while True:
    success, img = cap.read()
    img = cv2.flip(img,1)       #horizontal flip
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)   #feed converted to HSV
    img = cv2.rectangle(img, (40,10), (140,65), (0,0,0), 2)  #buttons for img window (essentially same as above)
    img = cv2.rectangle(img, (160,10), (255,65), colors[0], -1)
    img = cv2.rectangle(img, (275,10), (370,65), colors[1], -1)
    img = cv2.rectangle(img, (390,10), (485,65), colors[2], -1)
    img = cv2.rectangle(img, (505,10), (600,65), colors[3], -1)
    img = cv2.rectangle(img, (40,85), (140,140), (0,0,0), 2)
    img = cv2.rectangle(img, (40,160), (140,215), (0,0,0), 2)


    cv2.putText(img, "CLEAR ALL", (49,43), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)
    cv2.putText(img, "BLUE", (185, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "GREEN", (298, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "RED", (420, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "YELLOW", (520, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(img, "MARKER", (57,117), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons
    cv2.putText(img, "PEN", (72,192), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons
    
    blueMask = cv2.inRange(imgHSV, blue_lower, blue_upper)  
    blueMask = cv2.erode(blueMask, kernel, iterations = 2)      #A pixel in the original image (either 1 or 0) will be considered 1 only if all the pixels under the kernel is 1, otherwise it is eroded (made to zero).
    blueMask = cv2.morphologyEx(blueMask, cv2.MORPH_OPEN, kernel) 
    blueMask = cv2.dilate(blueMask, kernel, iterations = 1)

    contours, hierarchy = cv2.findContours(blueMask.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[0]
        ((x,y), radius) = cv2.minEnclosingCircle(contours)
        cv2.circle(img, (int(x), int(y)), int(radius), color_setting, 2)
        M = cv2.moments(contours)
        center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
        x2 = center[0]
        y2 = center[1]
        if 40 < x2 < 140 and 1 < y2 < 65:
            paintWindow = np.zeros((471,636,3)) + 255 
            paintWindow = cv2.rectangle(paintWindow, (40,10), (140,65), (0,0,0), 2)      #Button for Clear all   
            paintWindow = cv2.rectangle(paintWindow, (160,10), (255,65), colors[0], -1)  #Button for Blue Color
            paintWindow = cv2.rectangle(paintWindow, (275,10), (370,65), colors[1], -1)
            paintWindow = cv2.rectangle(paintWindow, (390,10), (485,65), colors[2], -1)
            paintWindow = cv2.rectangle(paintWindow, (505,10), (600,65), colors[3], -1)
            paintWindow = cv2.rectangle(paintWindow, (40,85), (140,140), (0,0,0), 2)
            paintWindow = cv2.rectangle(paintWindow, (40,160), (140,215), (0,0,0), 2)


            cv2.putText(paintWindow, "CLEAR ALL", (49,43), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons
            cv2.putText(paintWindow, "BLUE", (185, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(paintWindow, "GREEN", (298, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(paintWindow, "RED", (420, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(paintWindow, "YELLOW", (520, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 2, cv2.LINE_AA)
            cv2.putText(paintWindow, "MARKER", (57,117), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons
            cv2.putText(paintWindow, "PEN", (72,192), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0), 2, cv2.LINE_AA)   #Labels for buttons


        onbutton = False

        if 160 < x2 < 255 and 10 < y2 < 65:
            color_setting = (colors[0])
            onbutton = True
        elif 275 < x2 < 370 and 10 < y2 < 65:
            color_setting = (colors[1])
            onbutton = True
        elif 390 < x2 < 485 and 10 < y2 < 65:
            color_setting = (colors[2])
            onbutton = True
        elif 505 < x2 < 600 and 10 < y2 < 65:
            color_setting = (colors[3])
            onbutton = True
        
        
        if 40 < x2 < 140 and 85 < y2 < 140:
            utencilType = 10
            onbutton = True
        elif 40 < x2 < 140 and 160 < y2 < 215:
            utencilType = 4
            onbutton = True

        if  x1 == 0 and y1 == 0: 
            x1,y1 = x2,y2
        else:
            #print(x1,y1,x2,y2)
            if(not onbutton):
                cv2.line(paintWindow, (x1,y1), (x2,y2), color_setting, utencilType)
                cv2.line(img, (x1,y1), (x2,y2), color_setting, utencilType)

        x1,y1 = x2,y2
    else:
        x1,y1 = 0,0
                
    # Show the frame and the paintWindow image
    cv2.imshow("Tracking", img)
    cv2.imshow("Paint", paintWindow)

    # If the 'q' key is pressed, stop the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()







"""
myColors = [[7,44,222,255,100,190]]

def empty(val):
    pass

def get_color(): 
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars", 640,480)
    cv2.createTrackbar("Hue Min", "TrackBars", 0 , 179, empty)
    cv2.createTrackbar("Hue Max", "TrackBars", 179 , 179, empty)
    cv2.createTrackbar("Sat Min", "TrackBars", 0 , 255, empty)
    cv2.createTrackbar("Sat Max", "TrackBars", 255 , 255, empty)
    cv2.createTrackbar("Val Min", "TrackBars", 0 , 255, empty)
    cv2.createTrackbar("Val Max", "TrackBars", 255 , 255, empty)
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        success, img = cap.read()
        h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min,s_min,v_min]) 
        upper = np.array([h_max,s_max,v_max]) 
        mask = cv2.inRange(imgHSV,lower,upper)
        cv2.imshow("Result", img)
        cv2.imshow("Mask", mask)
    cv2.destroyAllWindows()
    return lower,upper 




def findColor(img, myColors, lower, upper):
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(imgHSV,lower,upper)
    cv2.imshow("findColor", mask)
    
def getContours(img,imgResult):
    contours, hierarchy = cv2.findCotours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            cv2.drawContours(imgResult,cnt,-1,(255,0,0), 3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            
            

def main():
    lower,upper = get_color()
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        success, img = cap.read()
        imgResult = img.copy()
        cv2.imshow("res", img)
        findColor(img,myColors,lower,upper)
        getContours(img,imgResult)

main()

"""