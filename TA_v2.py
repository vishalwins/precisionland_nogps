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
    setpointx=320
    setpointy=240
    eX,eY=(0,0)
    cX,cY=(0,0)
    camera = cv2.VideoCapture(1)
    while True:
        (grabbed, frame) = camera.read()
        status = "No Targets"
# check to see if we have reached the end of the
# video
        if not grabbed:
            break
# convert the frame to grayscale, blur it, and detect edges
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
                    eY=setpointy-cY
        cv2.putText(frame, "x: {}, y: {}".format(cX,cY),(10,40), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 3)
        cv2.putText(frame, "ex: {}, ey: {}".format(eX,eY),(300,40), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 3)


#roll + kanan - kiri | pitch - maju + mundur
#if error==9999:
#cv2.putText(image, "ERROR",(215,270 ), cv2.FONT_HERSHEY_SIMPLEX,2, (0, 0, 255), 5)
#cv2.putText(image, "RADIUS : {}".format(radius),(120, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 3)
   
    
#menampilkan frame dari img

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


def tampildata():
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    def animate(i):
        pullData = open("data.txt","r").read()
        dataArray = pullData.split('\n')
        xar = []
        yar = []
        for eachLine in dataArray:
            if len(eachLine)>1:
                x,y = eachLine.split(',')
                xar.append(int(x))
                yar.append(float(y))
        ax1.clear()
        ax1.plot(xar,yar)
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()


	
def kontrol():
    global kpx
    global kpy
    global pembagi
    kpx=0.525
    kpy=0.7
    pembagi=100
    tinggiland=0.3
    connection_string = "127.0.0.1:14550"
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True,baud=57600)
    def arm_and_takeoff_nogps(aTargetAltitude):
        """
        Arms vehicle dan terbang ke aTargetAltitude tanpa GPS data.
        """

    ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6

        print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
        #while not vehicle.is_armable:
         #   print(" Waiting for vehicle to initialise...")
          #  time.sleep(1)


        print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        vehicle.armed = True

        while not vehicle.armed:
            print(" Waiting for arming...")
            vehicle.armed = True
            time.sleep(1)

        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            print(" Altitude: %f  Desired: %f" %
                  (current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            set_attitude(thrust = thrust)
            time.sleep(0.2)

    def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5):
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000, # Type mask: bit 1 is LSB
            to_quaternion(roll_angle, pitch_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian
            thrust  # Thrust
        )
        vehicle.send_mavlink(msg)

#        start = time.time()
#        while time.time() - start < duration:
#            vehicle.send_mavlink(msg)
#            time.sleep(0.1)
    def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]
    def precisionland():
        set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5)
        while 1:
            cobawaktu=time.time()
            outputx=(kpx*eX)/pembagi
            outputy=(kpy*eY)/pembagi
            set_attitude(roll_angle = outputx, pitch_angle = outputy, yaw_rate = 0.0, thrust = 0.5)
            time.sleep(0.1)


    precisionland()
        
        
        


def jukukdata():
    waktuland=0
    connection_string = "127.0.0.1:14551"
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True,baud=57600)
    while True :
        time.sleep(1)
        while vehicle.mode.name =='GUIDED_NOGPS':
            file=open("data.txt","a")
            file.write("\n{},{}".format(waktuland,vehicle.location.global_relative_frame.alt))
            file.close()
 #           file=open("data2.txt","a")
  #          file.write("\n{}".format(waktuland))
    #        file.close()
            time.sleep(1)
            waktuland=waktuland + 1



    


kontrol = threading.Thread(name='kontrol', target=kontrol,args=())
image = threading.Thread(name='image', target=image,args=())
jukukdata = threading.Thread(name='jukukdata', target=jukukdata,args=())
tampildata = threading.Thread(name='tampildata', target=tampildata,args=())

kontrol.start()
image.start()
jukukdata.start()
tampildata.start()

 
