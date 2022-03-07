import time
from pyb import UART, LED

import sensor, image, time, math
import ulab

from ulab import numpy as np

np_dot = np.dot
print( "version", ulab.__version__ )
led = LED(2) # green led
led.on()

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
F = np.eye(6,  dtype=np.float)
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


# LAB thresholds
# lab field values
#red_thresh = [(45, 60, 50, 80, 30, 60)]
#blue_thresh = [(20, 45, -10, 10, -40, -10)]
#yellow_thresh = [(50, 80, 0, 35, 55, 70)]
#green_thresh = [(0, 15, 0, 40, -80, -20)]
#black_thresh = [(10, 16, -10, 10, -5, 15)]

# home values
#red_thresh = [(58, 75, 25, 50, -10, 20)]
red_thresh = [(50, 75, 23, 70, 40, 76)]
blue_thresh = [(40, 59, -20, 20, -60, -20)]
yellow_thresh = [(70, 90, -10, 10, 50, 80)]
green_thresh = [(50, 75, -50, -20, -5, 15)]
white_thresh = [(70, 93, -30, 10, -10, 20)]
#black_thresh = [(10, 16, -10, 10, -5, 15)]
sensor.reset()

sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)

curr_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, gain_db=15)
# === EXPOSURE ===
curr_exposure = sensor.get_exposure_us()
print(curr_exposure)
#sensor.set_auto_exposure(False, exposure_us = int(curr_exposure * 0.3))
#sensor.set_auto_exposure(False, exposure_us = 1000)

sensor.skip_frames(time = 1000)
# === WHITE BAL ===
sensor.set_auto_whitebal(False) #Must remain false for blob tracking

sensor.set_brightness(0)
sensor.set_contrast(3)
sensor.set_saturation(3)


sensor.__write_reg(0x0E, 0b00000000) # Disable night mode
sensor.__write_reg(0x3E, 0b00000000) # Disable BLC
sensor.skip_frames(time=1000)

# UART 3, and baudrate.

clock = time.clock()
uart = UART(1, 115200)

IMG_WIDTH = 320
IMG_HEIGHT = 240

selfX = IMG_WIDTH // 2
selfY = IMG_HEIGHT // 2

dT = 0


ballFound = False
notFoundCount = 0

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
    X = 500
    Y = 500
    height = 0
    width = 0
    max_area = 0
    box = None
    img.draw_rectangle(roi, color = (255, 255, 255))
    blobs = img.find_blobs(thresh, merge=merge, roi = roi, x_stride = stride, y_stride = stride, pixels_threshold=pixel_thresh, area_threshold=area_thresh, margin = margin)
    for blob in blobs:
        if blob.area() > max_area:
            max_area = blob.area()
            X = blob.cx()
            Y = blob.cy()
            height = blob.h()
            width = blob.w()
            box = blob.rect()

    if debug and X < 500:
        img.draw_rectangle(box, color = color)
        img.draw_cross(X, Y, color = color)

    return X, Y, width, height

def centralise(x, y, selfX, selfY):
    x = selfX * 2 - x
    x -= selfX
    y = -y + selfY
    return x, y

def processObj(X, Y, selfX, selfY):
    if X < 500:
        X, Y = centralise(X, Y, selfX, selfY)
        Angle = math.degrees(math.atan2(X, Y))
        if Angle < 0: Angle += 360
        Distance =  math.sqrt(X ** 2 + Y ** 2)
        return round(Angle), round(Distance)
    return 500, 500


def find_objects(debug=False):
    global ballFound
    global notFoundCount
    ballX, ballY, ballW, ballH = track_obj(red_thresh, 5, 5, debug = debug, stride=2)
    blueX, blueY, blueW, blueH = track_obj(blue_thresh, 10, 10, color = (0, 0, 255), stride = 10,  debug = debug, merge = True, margin = 30)
    yellowX, yellowY, yellowW, yellowH = track_obj(yellow_thresh, 20, 20, color = (0, 255, 0), stride = 10, debug =  debug, merge = True, margin = 30)

    if ballFound:
        kf.F[0][2] = dT
        kf.F[1][3] = dT

        state = kf.predict()
        predW = state[4][0]
        predH = state[5][0]
        predX = state[0][0]
        predY = state[1][0]
        print("Predicted:", predX, predY, "Actual:", ballX, ballY)
        predRect = (int(predX- predH / 2), int(predY- predW / 2), int(predW), int(predH))


        img.draw_rectangle(predRect, color = (0, 255, 255))
        img.draw_cross(int(predX), int(predY), color = (0, 255, 255))



    if ballX == 500 and ballY == 500:
        # ball not detected
        notFoundCount += 1
        if notFoundCount >= 100:
            found = False
    else:
        notFoundCount = 0
        z = np.array([[ballX], [ballY], [ballW], [ballH]], dtype=np.float)
        if not ballFound:
            # first detection!
            kf.P = np.eye(kf.F.shape[1])

            kf.x = np.array([z[0],z[1], [0], [0], z[2], z[3]], dtype=np.float)

            ballFound = True
        else:
            kf.update(z)


    if (blueY > selfY): blueY -= blueH / 2
    else: blueY += blueH / 2
    if (yellowY > selfY): yellowY -= yellowH / 2
    else: yellowY += yellowH / 2


    yellowAngle, yellowDist = processObj(yellowX, yellowY, selfX, selfY)
    blueAngle, blueDist = processObj(blueX, blueY, selfX, selfY)
    ballAngle, ballDist = processObj(ballX, ballY, selfX, selfY)
    #print(ballAngle, ballDist)
    #return ballAngle #, int(ballDist)]
    return [ballAngle, ballDist, blueAngle, blueDist, yellowAngle, yellowDist]

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
    ids = [sensor.OV5640, sensor.OV5640, sensor.OV7725, sensor.MT9M114, sensor.MT9V034]
    for i in range(len(ids)):
        if sensor.get_id() == ids[i]:
            print(i)

    if debug:
        print(clock.fps())

