import cv2
import numpy as np
from time import time
import math
import serial.tools.list_ports
from openpyxl import Workbook
ports=serial.tools.list_ports.comports()
serialinst= serial.Serial()
portsList=[]
portvar="COM"
for oneport in ports:
    portsList.append(str(oneport))
    print(str(oneport))
val=input("Select Port:COM")
for x in range(0,len(portsList)):
    if portsList[x].startswith("COM"+str(val)):
        portvar+=str(val)
        print(portvar)

serialinst.baudrate=9600
serialinst.port=portvar
serialinst.open()

video = cv2.VideoCapture(1)

profile = {
    'blue': {
        'rgb': (255, 0, 0),
        'threshold': ((95, 80, 80), (135, 255, 255))  # Adjusted threshold for blue
    },
    'red': {
        'rgb': (0, 0, 255),
        'threshold': ((0, 100, 100), (20, 255, 255))
    },
    'purple': {
        'rgb': (128, 0, 128),  # Purple RGB values
        'threshold': ((110, 50, 50), (150, 255, 255))  # Adjusted threshold for purple
    }
}

def getBallCoord(frame, mode):
    lower, upper = profile[mode]['threshold']

    bright = cv2.convertScaleAbs(frame, alpha=1.5, beta=10)
    hsv = cv2.cvtColor(bright, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=4)
    mask = cv2.dilate(mask, None, iterations=4)

    greyBitmap = cv2.bitwise_and(frame, frame, mask=mask)
    greyBitmap = cv2.cvtColor(greyBitmap, cv2.COLOR_BGR2GRAY)
    greyBitmap = cv2.GaussianBlur(greyBitmap, (9, 9), 2)

    circleArr = cv2.HoughCircles(greyBitmap, cv2.HOUGH_GRADIENT, 1, greyBitmap.shape[0] // 8,
                                  param1=50, param2=30, minRadius=0, maxRadius=0)

    if circleArr is None:
        return False
    else:
        intCircleArr = np.uint16(np.around(circleArr))
        return intCircleArr[0, :]

delx = 0
dely = 0

def cartesian_to_polar(delx, dely):
    # Calculate the polar coordinates
    print(delx,dely)
    radius = math.sqrt(delx**2 + dely**2)
    theta = math.atan2(dely, delx)  # arctan2 returns the angle in radians

    # Convert the angle to degrees
    theta_degrees = math.degrees(theta)

    return radius, theta_degrees

stime = time()
mode = 'blue'
ERROR_THRESHOLD = 40  # in pixels
xavg=0
counter=0;

while True:
    check, frame = video.read()
    frame = cv2.resize(frame,(640,480))
    if not check:
        break

    ballArr = getBallCoord(frame, mode)

    xCentre, yCentre = frame.shape[1] // 2, frame.shape[0] // 2
    count = delx = dely = 0

    if ballArr is not False:
        count = len(ballArr)
        for xCircle, yCircle, radius in ballArr:
            delx, dely = xCircle - xCentre, yCircle - yCentre
            dely = -dely
            cv2.circle(frame, (xCircle, yCircle), radius, (255, 255, 255), 3)

            cv2.circle(frame, (ballArr[0, 0], ballArr[0, 1]), ballArr[0, 2], profile[mode]['rgb'], 3)
            xavg+=ballArr[0,0]
            counter+=1
            if(counter==5):
                counter = 0
                if(xavg/5<280):
                    command='L'
                    serialinst.write(command.encode('utf-8'))
                elif(xavg/5>360):
                    command='R'
                    serialinst.write(command.encode('utf-8'))
                else:
                    command='W'
                    serialinst.write(command.encode('utf-8'))
                xavg=0
    else:
        command='N'
        serialinst.write(command.encode('utf-8'))
        
                

    etime = time() - stime

    cv2.line(frame, (xCentre - 20, yCentre), (xCentre + 20, yCentre), (0, 255, 0), 1)
    cv2.line(frame, (xCentre, yCentre - 20), (xCentre, yCentre + 20), (0, 255, 0), 1)
    cv2.putText(frame, f"Elapsed {etime:.2f} seconds", (50, 50), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, f"{mode.upper()} MODE", (50, 100), cv2.FONT_HERSHEY_COMPLEX, 0.5, profile[mode]['rgb'], 2)
    cv2.putText(frame, "Balls in frame: {}".format(count), (50, 150), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, f"Del X: {delx}", (frame.shape[0] - 100, 50), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, f"Del Y: {dely}", (frame.shape[0] - 100, 100), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 2)
    radius, theta_degrees = cartesian_to_polar(delx, dely)
   # print(f"Polar Coordinates: Radius = {radius}, Theta = {theta_degrees} degrees")

    if count > 0 and abs(delx) < ERROR_THRESHOLD and abs(dely) < ERROR_THRESHOLD:
        cv2.putText(frame, "Aligned", (xCentre - 20, yCentre + 100), cv2.FONT_HERSHEY_COMPLEX, 0.6, profile['blue']['rgb'], 2)

    cv2.imshow("Balls Detector", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
