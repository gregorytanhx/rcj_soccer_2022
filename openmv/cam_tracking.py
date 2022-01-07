import time
from pyb import UART

import sensor, image, time, math

from ulab import numpy as np

from ulab import numpy as np

class KalmanFilter(object):
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        """
        # Define sampling time
        self.dt = dt
        # Define the  control input variables
        self.u = np.matrix([[u_x],[u_y]])
        # Intial State
        self.x = np.matrix([[0], [0], [0], [0]])
        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt,0],
                            [0,self.dt]])
        # Define Measurement Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        #Initial Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2
        #Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2,0],
                           [0, y_std_meas**2]])
        #Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])

     def predict(self):
        # Refer to :Eq.(9) and Eq.(10)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
        # Update time state
        #x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]

     def update(self, z):
        # Refer to :Eq.(11), Eq.(12) and Eq.(13)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #Eq.(11)
        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))   #Eq.(12)
        I = np.eye(self.H.shape[1])
        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P   #Eq.(13)
        return self.x[0:2]


# LAB thresholds
# lab field values
#red_thresh = [(45, 60, 50, 80, 30, 60)]
#blue_thresh = [(20, 45, -10, 10, -40, -10)]
#yellow_thresh = [(50, 80, 0, 35, 55, 70)]
#green_thresh = [(0, 15, 0, 40, -80, -20)]
#black_thresh = [(10, 16, -10, 10, -5, 15)]

# home values
red_thresh = [(58, 75, 25, 50, -5, 20)]
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
sensor.set_auto_exposure(False, exposure_us = int(curr_exposure * 0.3))
#sensor.set_auto_exposure(False, exposure_us = 200)

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
uart = UART(3, 115200)

IMG_WIDTH = 320
IMG_HEIGHT = 240

selfX = IMG_WIDTH // 2
selfY = IMG_HEIGHT // 2

FPS = 50.9

ballTracker = KalmanFilter(1/FPS, 1, 1, 1, 0.1, 0.1)

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
            box = blob.rect()

    if debug and X < 500:
        img.draw_rectangle(box, color = color)
        img.draw_cross(X, Y, color = color)

    return X, Y, height

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

    ballX, ballY, _ = track_obj(red_thresh, 5, 5, debug = debug, stride=2)
    blueX, blueY, blueH = track_obj(blue_thresh, 10, 10, color = (0, 0, 255), stride = 10,  debug = debug, merge = True, margin = 30)
    yellowX, yellowY, yellowH = track_obj(yellow_thresh, 20, 20, color = (0, 255, 0), stride = 10, debug =  debug, merge = True, margin = 30)

    predX, predY = ballTracker.predict()
    ballTracker.update(np.array([[ballX], [ballY]]))

    if (blueY > selfY): blueY -= blueH / 2
    else: blueY += blueH / 2
    if (yellowY > selfY): yellowY -= yellowH / 2
    else: yellowY += yellowH / 2


    yellowAngle, yellowDist = processObj(yellowX, yellowY, selfX, selfY)
    blueAngle, blueDist = processObj(blueX, blueY, selfX, selfY)
    ballAngle, ballDist = processObj(ballX, ballY, selfX, selfY)
    #print(ballAngle, ballDist)
    if blueY > yellowY: side = 1
    else: side = 0
    #return ballAngle #, int(ballDist)]
    return [ballAngle, ballDist, blueAngle, blueDist, yellowAngle, yellowDist, side]

#data = [53, 47]

#import pyb
#pyb.hard_reset()

while(True):
    clock.tick()
    img = sensor.snapshot()




    #print(sensor.get_exposure_us())
    data = find_objects(debug=True)

    #for i in range(len(data)):
        #if data[i] == 42:
            #data[i] = 43

    #uart.writechar(42)

    #for i in data:
        #uart.write("%d\n"%i)


    print(clock.fps())
