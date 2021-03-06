import time
from pyb import UART, LED

import sensor, image, time, struct
from math import *
import ulab

from ulab import numpy as np

np_dot = np.dot
print( "version", ulab.__version__ )
led = LED(2) # green led
led.off()

'''Extended Kalman Filter for smoother ball following'''

class KalmanFilter:
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")
        self.n = F.shape[1]
        self.m = H.shape[1]
        self.F = F
        self.H = H
        #self.B = np.zeros(1, dtype=np.float) if B is None else B
        self.Q = np.eye(self.n, dtype=np.float) if Q is None else Q
        self.R = np.eye(self.n, dtype=np.float) if R is None else R
        self.P = np.zeros((self.n, self.n)) if P is None else P
        self.x = [[0], [0], [0], [0], [0], [0]]
    def predict(self):
        self.x = np_dot( self.F, self.x ) #+ np_dot( self.B, u )
        self.P = np_dot(np_dot(self.F, self.P), self.F.transpose().copy()) + self.Q
        return self.x
    def update(self, z):
        y = z - np_dot( self.H, self.x )
        S = self.R + np_dot( self.H, np_dot( self.P, self.H.transpose().copy() ) )
        K = np_dot( np_dot( self.P, self.H.transpose().copy() ), np.linalg.inv(S) )
        self.x = self.x + np_dot( K, y )
        I = np.eye( self.n )
        self.P = np_dot( np_dot( I - np_dot( K, self.H ), self.P ),
            (I - np_dot( K, self.H ) ).transpose().copy() ) + np_dot( np_dot( K, self.R ), K.transpose().copy() )
        return self.x

# set dT at each processing step
F = np.eye(6, dtype=np.float)
B = 0

H = np.array([[1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]],  dtype=np.float)


Q = np.array([[1e-2, 0, 0, 0, 0, 0],
            [0, 1e-2, 0, 0, 0, 0],
            [0, 0, 5.0 , 0, 0, 0],
            [0, 0, 0, 5.0 , 0, 0],
            [0, 0, 0, 0, 1e-2, 0],
            [0, 0, 0, 0, 0, 1e-2]], dtype=np.float)

R = np.array([[1e-1, 0, 0, 0],
            [0, 1e-1, 0, 0,],
            [0, 0, 1e-1, 0],
            [0, 0, 0, 1e-1]], dtype=np.float)

kf = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

curr_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, gain_db=15)
# === EXPOSURE ===
curr_exposure = sensor.get_exposure_us()
print(curr_exposure * 0.21)

# if tuning exposure
sensor.set_auto_exposure(False, exposure_us = 1580)

LAB_EXPOSURE = 1775
SC_EXPOSURE = 2000

#sensor.set_auto_exposure(False, exposure_us = LAB_EXPOSURE)

sensor.skip_frames(time = 1000)
# === WHITE BAL ===
sensor.set_auto_whitebal(False, rgb_gain_db = (-5.55645, -6.52073, -0.504)) #Must remain false for blob tracking

sensor.set_brightness(0)
sensor.set_contrast(3)
#sensor.set_saturation(3)

sensor.skip_frames(time=1000)

clock = time.clock()
uart = UART(1, 2000000)


ID = 'whitebot'
ID = 'blackbot'

if ID == 'blackbot':
    centreY = 114
    centreX = 158
    startX = 20
    endX = 300
    ROI = (0, 0, 298, 240)

    # LAB thresholds
    # lab field values
    #red_thresh = [(35, 55, 20, 53, 15, 40)]

    red_thresh = [(43, 71, 41, 65, -9, 45)]
    blue_thresh = [(30, 74, -22, 14, -57, -30)]
    yellow_thresh = [(77, 100, -50, 127, 19, 61)]



    # SCIENCE CENTRE VALS

    # PRAC FIELD
    #red_thresh = [(43, 71, 41, 65, -9, 45)]
    #blue_thresh = [(5, 29, -20, 29, -31, -3)]
    #yellow_thresh = [(45, 100, 0, 40, 18, 127)]

    # REAL VALS

    red_thresh = [(43, 71, 41, 65, -9, 45)]
    blue_thresh = [(5, 29, -20, 29, -31, -3)]
    yellow_thresh = [(51, 93, -18, 66, 23, 107)]
else:
    centreY = 144
    centreX = 155
    ROI = (0, 0, 297, 240)
    startX = 20
    endX = 300

    # LAB thresholds
    # lab field values.
    #red_thresh = [(43, 71, 41, 65, -9, 45)]
    #blue_thresh = [(30, 74, -22, 14, -57, -30)]
    #yellow_thresh = [(77, 100, -50, 127, 19, 61)]


    # SCIENCE CENTRE VALS
    red_thresh = [(43, 71, 41, 65, -9, 45)]
    blue_thresh = [(0, 23, -35, 11, -24, -7)]
    yellow_thresh = [(51, 93, -18, 66, 23, 107)]

    # PRACTICE FIELD

dT = 0
ballFound = False
notFoundCount = 0
prevBall = None
prevBlue = None
prevYellow = None
ballROI = ROI
yellowROI = ROI
blueROI = ROI


def distanceMapper(pixel):
    # use polynomial regression to map pixel distance to centimetre distance
    polarity = pixel / abs(pixel) if pixel != 0 else 1
    # use absolute value since polynomial equation is different for negative numbers
    pixel = abs(pixel)

    return (-3.533887031 +
           (1.9393341134 ) * pixel +
           (-0.1056768120) * pixel**2 +
           (0.0026433076) * pixel**3  +
           (-0.0000275141) * pixel**4 +
           (0.0000001031) * pixel**5 ) * polarity

def distanceUnmapper(real):
    # use polynomial regression to map centimetre distance back to pixel distance
    polarity = real / abs(real) if real != 0 else 1
    # use absolute value since polynomial equation is different for negative numbers
    real = abs(real)
    return (-4.92497614 +
           (3.0761747478) * real +
           (-0.0238111740) * real**2 +
           (-0.0000109829) * real**3  +
           (0.0000008785) * real**4 +
           (-0.0000000027) * real**5 ) * polarity

class obj:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area = w * h
        self.angle = 500
        self.dist = 500
        self.unmappedDist = 0
        self.confidence = 0

    def centralise(self):
        self.x -= centreX
        self.y = centreY - self.y

    def process(self, mapDist = True):
        # currently ball is mapped before finding angle, goals are not
        # need to test to see if that makes a difference
        self.angle = degrees(atan2(self.y, self.x))
        if self.angle < 0: self.angle += 360
        self.unmappedDist = sqrt(self.x ** 2 + self.y ** 2)
        if mapDist:
            self.dist = distanceMapper(sqrt(self.x ** 2 + self.y ** 2))
        else:
            self.dist = sqrt(self.x ** 2 + self.y ** 2)

        # ratio of distance to area should be fixed, allowing us to see if goal is obstructed.
        self.confidence = (self.area / self.dist) / 0.1

    def getROI(self, sizeX, sizeY):
        tmpX = self.x
        tmpY = self.y
        x = int(max(startX, tmpX - sizeX // 2))
        y = int(max(0, tmpY - sizeY // 2))

        if sizeX // 2 + tmpX > endX:
            #print(tmpX + sizeX // 2)
            w = sizeX - (tmpX + sizeX // 2 - endX)
            #print(x + w)
        else:
            w = sizeX

        if sizeY // 2 + tmpY > 240:
            h = sizeY - (tmpY + sizeY // 2 - 240)
        else:
            h = sizeY
        return (x, y, int(w), int(h))



def track_obj(thresh, pixel_thresh, area_thresh, color = (255, 255, 255), debug=False, stride = 1, margin = 5, merge = False, roi = ROI):
    found_obj = None
    found_blob = None
    max_area = 0
    box = None
    # more blobs means more things blocking the goal
    num_blobs = 0
    blobs = img.find_blobs(thresh, merge=merge, roi = roi, x_stride = stride, y_stride = stride, pixels_threshold=pixel_thresh, area_threshold=area_thresh, margin = margin)
    for blob in blobs:
        num_blobs += 1
        if blob.area() > max_area:
            found_blob = blob
            max_area = blob.area()
            found_obj = obj(blob.cx(), blob.cy(), blob.w(), blob.h())

    if debug and found_obj:
        img.draw_rectangle(found_blob.rect(), color = color)
        img.draw_cross(found_blob.cx(), found_blob.cy(), color = color)
        #img.draw_line(centreX, found_blob.cy(), centreY, found_blob.cx(), color=color)
        found_obj.confidence = 1/num_blobs

    return found_obj


def find_objects(debug=False):
    global ballFound
    global notFoundCount
    global prevBall, prevYellow, prevBlue, ballROI, blueROI, yellowROI

    predBall = None


    if prevBall is None:
        ballROI = ROI
        # if ball previously seen, search for ball within a 100px x 100px window of prev ball location
    #else:
        #img.draw_rectangle(ballROI, color = (255, 0,0))
    # if goal previously seen, search for goal within 150 x 150 window of prev goal location
    if prevYellow is None:
        yellowROI = ROI
    #else:
        #img.draw_rectangle(yellowROI, color = (255, 255, 0))
    if prevBlue is None:
        blueROI = ROI
    #else:
        #img.draw_rectangle(blueROI, color = (0, 255, 255))


    img.draw_cross(centreX, centreY, color = (255, 0, 0))
    ball = track_obj(red_thresh, 3, 3, debug = debug, stride=2, roi = ballROI)
    blue = track_obj(blue_thresh, 5, 10, color = (0, 0, 255), stride = 10,  debug = debug, merge = False, margin = 0, roi = blueROI)
    yellow = track_obj(yellow_thresh, 5, 10, color = (0, 255, 0), stride = 10, debug =  debug, merge = False, margin = 0, roi = yellowROI)
    if ball:
        ballROI = ball.getROI(90, 90)
    if yellow:
        yellowROI = yellow.getROI(120,180)
    if blue:
        blueROI = blue.getROI(120,180)

    if ballFound:
        kf.F[0][2] = dT
        kf.F[1][3] = dT

        state = kf.predict()
        predW = state[4][0]
        predH = state[5][0]
        predX = state[0][0]
        predY = state[1][0]
        predBall = obj(predX, predY, predW, predH)
        predBall.process(mapDist = False)

        if debug:
            debugPredX = centreX + distanceUnmapper(predX)
            debugPredY = centreY - distanceUnmapper(predY)


            predRect = (int(debugPredX - predH / 2), int(debugPredY - predW / 2), int(predW), int(predH))
            img.draw_rectangle(predRect, color = (0, 255, 255))
            img.draw_cross(int(debugPredX), int(debugPredY), color = (0, 255, 255))


        # ball not detected but was recently seen

    if ball:
        notFoundCount = 0
        # map pixel dist to real dist so kalman filter can track ball more accurately
        #print(ball.x, ball.y)
        ball.centralise()
        ball.x = distanceMapper(ball.x)
        ball.y = distanceMapper(ball.y)

        z = np.array([[ball.x], [ball.y], [ball.w], [ball.h]], dtype=np.float)
        ball.process(mapDist = False)
        if not ballFound:
            # first detection!
            kf.P = np.eye(kf.F.shape[1])
            kf.x = np.array([z[0],z[1], [0], [0], z[2], z[3]], dtype=np.float)

            ballFound = True
        else:
            kf.update(z)
    else:
        # ball not detected
        notFoundCount += 1
        if notFoundCount >= 100:
            ballFound = False



    # create dummy objects if the object is not detected
    if yellow:
        yellow.centralise()
        yellow.process()
        prevYellow = yellow

        #print(f"Yellow pixel dist: {yellow.x} ")
        #if debug:
            #print(f"Yellow Goal: Angle: {yellow.angle} Distance: {yellow.dist} Pixel Distance: {yellow.unmappedDist}")
    else:
        yellow = obj(0, 0, 0, 0)
        prevYellow = None


    if blue:
        blue.centralise()
        blue.process()
        prevBlue = blue
        #print(f"blue pixel dist: {blue.y} ")

        #if debug:
            #print(f"Blue Goal: Angle: {blue.angle} Distance: {blue.dist} Pixel Distance: {blue.unmappedDist}")
    else:
        prevBlue = None
        blue = obj(0, 0, 0, 0)
        #if debug:
            #print(f"Blue Goal not detected!")

    if ball:
        prevBall = ball
        if debug:
            #print(f"Ball: Angle: {ball.angle} Distance: {ball.dist}")
            pass
    else:
        ball = obj(0, 0, 0, 0)
        prevBall = None

    if predBall:
        if debug:
            #print(f"Predicted Ball: Angle: {predBall.angle} Distance: {predBall.dist}")
            pass
    else:
        predBall = obj(0, 0, 0, 0)

    return [ball.angle, ball.dist, predBall.angle, predBall.dist, blue.angle, blue.dist, yellow.angle, yellow.dist]

def send(data):
    sendData = [42]

    for num in data:
        # convert from 2dp float to int
        num = int(round(num, 2) * 100)

        sendData += list(num.to_bytes(2, 'little'))

    for num in sendData:
        try:
            uart.writechar(num)
        except:
            pass

while(True):


    clock.tick()
    debug = False
    debug = True


    img = sensor.snapshot()

    dT = 1/clock.fps()

    data = find_objects(debug=debug)

    send(data)
    print("FPS:", clock.fps())



