import time
from pyb import UART, LED

import sensor, image, time
from math import *
import ulab

from ulab import numpy as np

np_dot = np.dot
print( "version", ulab.__version__ )
led = LED(2) # green led
led.on()

'''Kalman Filter for smoother ball following'''

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
sensor.skip_frames(time=1000)

curr_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, gain_db=15)
# === EXPOSURE ===
curr_exposure = sensor.get_exposure_us()
print(curr_exposure)
sensor.set_auto_exposure(False, exposure_us = int(curr_exposure * 0.3))
#sensor.set_auto_exposure(False, exposure_us = 1000)

sensor.skip_frames(time = 2000)
# === WHITE BAL ===
sensor.set_auto_whitebal(False) #Must remain false for blob tracking

sensor.set_brightness(0)
sensor.set_contrast(3)
sensor.set_saturation(3)

#sensor.__write_reg(0x0E, 0b00000000) # Disable night mode
#sensor.__write_reg(0x3E, 0b00000000) # Disable BLC
sensor.skip_frames(time=1000)

clock = time.clock()
uart = UART(1, 1000000)

ID = 'whitebot'

if ID == 'whitebot':
    centreX = 120
    centreY = 160

    # LAB thresholds
    # lab field values
    red_thresh = [(50, 75, 23, 70, 40, 76)]
    blue_thresh = [(40, 59, -20, 20, -60, -20)]
    yellow_thresh = [(70, 90, -10, 10, 50, 80)]
    green_thresh = [(50, 75, -50, -20, -5, 15)]
    white_thresh = [(70, 93, -30, 10, -10, 20)]
    black_thresh = [(10, 16, -10, 10, -5, 15)]

else:
    centreX = 120
    centreY = 160

    # LAB thresholds
    # lab field values
    red_thresh = [(50, 75, 23, 70, 40, 76)]
    blue_thresh = [(40, 59, -20, 20, -60, -20)]
    yellow_thresh = [(70, 90, -10, 10, 50, 80)]
    green_thresh = [(50, 75, -50, -20, -5, 15)]
    white_thresh = [(70, 93, -30, 10, -10, 20)]
    black_thresh = [(10, 16, -10, 10, -5, 15)]

dT = 0
ballFound = False
notFoundCount = 0

def distanceMapper(pixel):
    # use polynomial regression to map pixel distance to centimetre distance
    polarity = pixel / abs(pixel)
    # use absolute value since polynomial equation is different for negative numbers
    pixel = abs(pixel)
    return ((-1.08267) +
           (1.21493) * pixel +
           (-0.0402618) * pixel**2 +
           (0.000964692) * pixel**3  +
           (-0.00000913121) * pixel**4 +
           (0.000000030743) * pixel**5) * polarity

def distanceUnmapper(real):
    # use polynomial regression to map centimetre distance back to pixel distance
    polarity = real / abs(real)
    # use absolute value since polynomial equation is different for negative numbers
    real = abs(real)
    return ((-1.08267) +
           (1.21493) * real +
           (-0.0402618) * real**2 +
           (0.000964692) * real**3  +
           (-0.00000913121) * real**4 +
           (0.000000030743) * real**5) * polarity

class obj:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area = w * h
        self.angle = 500
        self.dist = 500

    def centralise(self, selfX, selfY):
        self.x -= centreX
        self.y = centreY - self.y

    def process(self, mapDist = True):
        # currently ball is mapped before finding angle, goals are not
        # need to test to see if that makes a difference
        self.angle = degrees(atan2(self.x, self.y))
        if self.angle < 0: self.angle += 360
        if mapDist:
            self.dist = round(distanceMapper(sqrt(self.x ** 2 + self.y ** 2)), 2)
        else:
            self.dist = round(sqrt(self.x ** 2 + self.y ** 2), 2)


def track_field(thresh, pixel_thresh, area_thresh, color = (255, 255, 255), debug=False, roi = (15, 10, 280, 210), stride = 1, margin = 20):
    blobs = img.find_blobs(thresh, merge=True, roi = roi, x_stride = 5, y_stride = 5, pixels_threshold=pixel_thresh, area_threshold=area_thresh, margin = margin)
    for blob in blobs:
        X = blob.cx()
        Y = blob.cy()
        #height = blob.h()
        box = blob.rect()
        img.draw_rectangle(box, color = color)
        img.draw_cross(X, Y, color = color)


def track_obj(thresh, pixel_thresh, area_thresh, color = (255, 255, 255), debug=False, roi = (40, 10, 240, 200), stride = 1, margin = 5, merge = False):
    found_obj = None
    max_area = 0
    box = None
    img.draw_cross(160, 120, color = (255, 0, 0))
    blobs = img.find_blobs(thresh, merge=merge, roi = roi, x_stride = stride, y_stride = stride, pixels_threshold=pixel_thresh, area_threshold=area_thresh, margin = margin)
    for blob in blobs:
        if blob.area() > max_area:
            max_area = blob.area()
            found_obj = obj(blob.cy(), blob.cx(), blob.w(), blob.h())

    if debug and found_obj:
        img.draw_rectangle((found_obj.x, found_obj.y, found_obj.w, found_obj.h), color = color)
        img.draw_cross(found_obj.x, found_obj.y, color = color)

    return found_obj if found_obj else None



def find_objects(debug=False):
    global ballFound
    global notFoundCount
    ball = track_obj(red_thresh, 5, 5, debug = debug, stride=2)
    blue = track_obj(blue_thresh, 10, 10, color = (0, 0, 255), stride = 10,  debug = debug, merge = True, margin = 30)
    yellow = track_obj(yellow_thresh, 20, 20, color = (0, 255, 0), stride = 10, debug =  debug, merge = True, margin = 30)

    if ballFound:
        kf.F[0][2] = dT
        kf.F[1][3] = dT

        state = kf.predict()
        predW = state[4][0]
        predH = state[5][0]
        predX = state[0][0]
        predY = state[1][0]
        if debug:
            debugPredX = distanceUnmapper(predX) + centreX
            debugPredY = distanceUnmapper(predY) * 2 - centreY
            predRect = (int(debugPredX - predH / 2), int(debugPredY - predW / 2), int(predW), int(predH))

            img.draw_rectangle(predRect, color = (0, 255, 255))
            img.draw_cross(int(debugPredX), int(debugPredX), color = (0, 255, 255))

        # ball not detected but was recently seen
        if not ball:
            ball = obj(predY, predX, predW, predH)
            ball.angle = degrees(atan2(ball.x, ball.y))
            ball.dist = sqrt(ball.x ** 2 + ball.y ** 2)

    if ball:
        notFoundCount = 0
        # map pixel dist to real dist so kalman filter can track ball more accurately
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
            found = False

    # fuck forgot why i did this
    #if (blue):
        #if (blue.y > selfY): blue.y -= blue.h / 2
        #else: blue.y += blue.h / 2
    #if (yellow):
        #if (yellow.y > selfY): yellow.y -= yellow.h / 2
        #else: yellow.y += yellow.h / 2

    # create dummy objects if the object is not detected
    if yellow:
        yellow.centralise()
        yellow.process()
        if debug:
            print(f"Yellow Goal: Angle: {yellow.angle} Distance: {yellow.dist}")
    else:
        yellow = obj(0, 0, 0, 0)
        if debug:
            print(f"Yellow Goal not detected!")
    if blue:
        blue.centralise()
        blue.process()
        if debug:
            print(f"Blue Goal: Angle: {blue.angle} Distance: {blue.dist}")
    else:
        blue = obj(0, 0, 0, 0)
        if debug:
            print(f"Blue Goal not detected!")

    if ball:
        if debug:
            print(f"Ball: Angle: {ball.angle} Distance: {ball.dist}")
    else:
        ball = obj(0, 0, 0, 0)
        if debug:
            print(f"Ball not detected!")


    return [ball.angle, ball.dist, blue.angle, blue.dist, blue.area, yellow.angle, yellow.dist, yellow.area]

def send(data):
    sendData = [42]

    for num in data:
        num = round(num)
        sendData += list(num.to_bytes(2, 'little'))

    for num in sendData:
        try:
            uart.writechar(num)
        except:
            pass

while(True):
    debug = True
    clock.tick()
    img = sensor.snapshot()

    dT = 1/clock.fps()

    data = find_objects(debug=debug)

    send(data)

    if debug:
        print(clock.fps())

