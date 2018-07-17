from collections import deque
import argparse
import imutils
import cv2
import threading
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math


# thanks to dronekit, arducopter and pyimage that become my reference for this program
def image():
    global cX,cY
    global radius
    global bawah
    global atas
    global luas
    global setpointx
    global setpointy
    global eX,eY
    global lex,ley
    global ts
    setpointx=320
    setpointy=240
    eX,eY=(0,0)
    cX,cY=(0,0)
    leX,leY=(0,0)
    jeX,jeY=(0,0)
    ts=0
    camera = cv2.VideoCapture(1)
    while True:
        (grabbed, frame) = camera.read()
           
        if not grabbed:
            break
        waktumulai=time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edged = cv2.Canny(blurred, 50, 150)
# find contours in the edge map
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

# loop over the contours
        for c in cnts:
# approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01 * peri, True)
# ensure that the approximated contour is "roughly" rectangular
            if len(approx) >= 4 and len(approx) <= 6:
# compute the bounding box of the approximated contour and
# use the bounding box to compute the aspect ratio
                (x, y, w, h) = cv2.boundingRect(approx)
                aspectRatio = w / float(h)
# compute the solidity of the original contour
                area = cv2.contourArea(c)
                hullArea = cv2.contourArea(cv2.convexHull(c))
                solidity = area / float(hullArea)

# compute whether or not the width and height, solidity, and
# aspect ratio of the contour falls within appropriate bounds
                keepDims = w > 25 and h > 25
                keepSolidity = solidity > 0.9
                keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2


# ensure that the contour passes all our tests
                if keepDims and keepSolidity and keepAspectRatio:
# draw an outline around the target and update the status
# text
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)

# compute the center of the contour region and draw the
# crosshairs
                    M = cv2.moments(approx)
                    (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
                    (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
                    cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
                    cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)
                    eX=cX-setpointx
                    eY=cY-setpointy
                    leX=eX
                    leY=eY
                    jeX=jeX+(eX*ts)
                    jeY=jeY+(eY*ts)

        cv2.putText(frame, "x: {}, y: {}".format(cX,cY),(10,40), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 3)
        cv2.putText(frame, "ex: {}, ey: {}".format(eX,eY),(300,40), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 3)
        ts=time.time()-waktumulai


        cv2.imshow("Frame", frame)
        cv2.imshow("edged", edged)    
#menunggu huruf q untuk ditekan sehingga keluar dari loop

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

# matiin camera matiin smua window
    camera.release()
    cv2.destroyAllWindows()
    print threading.currentThread().getName(), 'Sudah selesai'




	
def kontrol():
    global kpx
    global kpy
    global pembagi
    kpx=0.75
    kpy=1
    kix=0.42
    kiy=0.63
    kdx=0.935
    kdy=0.1205
    tinggiland=0.3
    connection_string = "127.0.0.1:14550"
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True,baud=57600)

    def precisionland():
        while vehicle.mode.name =='ALT_HOLD':
            outputx=(kpx*eX)+((kdX*(eX-leX))/ts)+(kix*jeX)
            outputy=(kpy*eY)+((kdY*(eY-leY))/ts)+(kiy*jeY)
#roll
            vehicle.channels.overrides['1'] = 1500+outputx
#pitch
            vehicle.channels.overrides['2'] = 1500+outputy
            time.sleep(0.1)
    print "mulai"
    precisionland()
        
        
        



    


kontrol = threading.Thread(name='kontrol', target=kontrol,args=())
image = threading.Thread(name='image', target=image,args=())


kontrol.start()
image.start()


 
