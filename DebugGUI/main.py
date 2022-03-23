import pygame
import math
import sys
import os
import time
import serial
import pickle
import struct
from background import draw_bg
import tkinter as tk
from tkinter import *
import serial.tools.list_ports



bt_working = False

ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
try:
    bluetooth = serial.Serial(port="COM30", baudrate=115200, timeout=1)
    bt_working = True
    print("BLUETOOTH ENABLED")
except:
    print("BLUETOOTH NOT ENABLED")
# field dimensions
OUTER_WIDTH = 182
OUTER_HEIGHT = 243
INNER_WIDTH = 132
INNER_HEIGHT = 193
SCALE = 4
BUFFER_SIZE = 12

mode = {
    "NormalProgram": 1,
    "TestLocalisation": 2,
    "TuneBallTrack": 3,
    "RemoteControl": 4,
}

# TODO: add user interaction for moving robot to point on field
# TODO: add GUI for tuning ball track (can copy from simulator)
# TODO: add GUI for switching debug modes

# root = tk.Tk()
# root.title("DEBUG")
# root.geometry("350x300")


# currMode = StringVar(root)
# currMode.set("NormalProgram") # default value

# modeMenu = OptionMenu(root, currMode, "NormalProgram", "TestLocalisation", "TuneBallTrack", "RemoteControl")

# # ball tuning stuff
# k = tk.Scale(root, from_= 0, to = 1.2, resolution=0.02, orient=HORIZONTAL, length = 300)
# a = tk.Scale(root, from_= 0, to = 2, resolution=0.001, orient=HORIZONTAL, length = 300)
# b = tk.Scale(root, from_= 0, to = 8, resolution=0.1, orient=HORIZONTAL, length = 300)
# c = tk.Scale(root, from_= 200, to= 1000, resolution=10, orient=HORIZONTAL, length = 300)
# k_label = tk.Label(root, text = 'OFFSET_K', font=('calibre',10, 'bold'))
# a_label = tk.Label(root, text = 'BALL_MULT_A', font=('calibre',10, 'bold'))
# b_label = tk.Label(root, text = 'BALL_MULT_B', font=('calibre',10, 'bold'))
# c_label = tk.Label(root, text = 'MAX_DIST', font=('calibre',10, 'bold'))

# # load previously saved values
# try:
#     with open("settings.pkl", "rb") as f:
#         tmp = pickle.load(f)
#         OFFSET_K, BALL_MULT_A, BALL_MULT_B, MAX_DIST = tmp
#         k.set(OFFSET_K)
#         a.set(BALL_MULT_A)
#         b.set(BALL_MULT_B)
#         c.set(MAX_DIST)
#         vals = {"OFFSET_K": OFFSET_K, "BALL_MULT_A": BALL_MULT_A, "BALL_MULT_B": BALL_MULT_B, "MAX_DIST": MAX_DIST}
#         print(vals)
# except:
#     pass


# save_btn=tk.Button(root,text = 'Save', command = save)
# k_label.grid(row=0, column=0)
# k.grid(row=1, column=0)
# a_label.grid(row=2, column=0)
# a.grid(row=3, column=0)
# b_label.grid(row=4, column=0)
# b.grid(row=5, column=0)
# c_label.grid(row=6, column=0)
# c.grid(row=7, column=0)

# save_btn.grid(row=8, column=0)

# modeMenu.grid(row=9, column=1)


def chooseMode():
    variable = StringVar(master)
    variable.set("one")  # default value

    w = OptionMenu(master, variable, "one", "two", "three")
    w.pack()


def save():
    vals = [k.get(), a.get(), b.get(), c.get()]

    with open("settings.pkl", "wb+") as f:
        pickle.dump(vals, f)


def shiftCoords(x, y):
    # convert from centre origin to top left origin
    x += OUTER_WIDTH // 2
    y = OUTER_HEIGHT // 2 - y
    return x, y


def getCoords(angle, distance):
    # convert mm dist to cm
    distance /= 10
    # convert coordinates from polar to cartesian
    x = distance * math.sin(math.radians(angle))
    y = distance * math.cos(math.radians(angle))
    # print(f"Coordinates X: {int(x)} Y: {int(y)}")

    return x, y


class SerialRecData:
    def __init__(self):
        # position of robot in polar coordinates
        self.botAngle = 0
        self.botDistance = 0
        # ball data
        self.ballAngle = 0
        self.ballDist = 0
        # bbox
        self.width = 0
        self.height = 0


class Ball(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 4 * SCALE
        self.img = img = pygame.image.load("ball.png")
        self.img = pygame.transform.scale(img, (self.radius, self.radius))

    def setPosition(self, coords):
        self.x = int(coords[0]) * SCALE
        self.y = int(coords[1]) * SCALE

    def draw(self, screen):
        screen.blit(self.img, (self.x - self.radius // 2, self.y - self.radius // 2))


class Bot(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 9 * SCALE
        self.w = 0
        self.h = 0

    def setBox(self, width, height):
        self.w = (width / 10) * SCALE
        self.h = (height / 10) * SCALE

    def setPosition(self, coords):
        self.x = int(coords[0]) * SCALE
        self.y = int(coords[1]) * SCALE

    def draw(self, screen):
        # draw bot
        pygame.draw.circle(screen, (40, 40, 40), (self.x, self.y), self.radius)
        # draw bounding box
        bbox = pygame.Rect(self.x - self.w / 2, self.y - self.h / 2, self.w, self.h)
        pygame.draw.rect(screen, (40, 40, 40), bbox, 4, border_radius=10)


def getItem(buffer, i):
    return int.from_bytes(buffer[2 * i : 2 * (i + 1)], byteorder="little")


def getSerialData(data):
    print(bluetooth.in_waiting)
    while bluetooth.in_waiting >= BUFFER_SIZE + 1:
        buffer = bytearray(bluetooth.readline())

        data.botAngle = getItem(buffer, 0)
        data.botDistance = getItem(buffer, 1)
        data.ballAngle = getItem(buffer, 2)
        data.ballDist = getItem(buffer, 3)
        data.width = getItem(buffer, 4)
        data.height = getItem(buffer, 5)


def sendSerialData(data):
    OFFSET_K = k.get()
    BALL_MULT_A = a.get()
    BALL_MULT_B = b.get()
    MAX_DIST = c.get()

    vals = [OFFSET_K, BALL_MULT_A, BALL_MULT_B, MAX_DIST]
    sendData = []
    for i in range(len(vals)):
        # convert data to bytes
        tmp = "f" if type(vals[i]) == float else "i"
        sendData += list(struct.pack(f"<{tmp}", vals[i]))
    bluetooth.write(bytearray(sendData))


def main():
    pygame.init()
    screen = pygame.display.set_mode((OUTER_WIDTH * SCALE, OUTER_HEIGHT * SCALE))

    ball = Ball(OUTER_WIDTH // 2, OUTER_HEIGHT // 2)
    bot = Bot(OUTER_WIDTH // 2, OUTER_HEIGHT // 2)
    run = True
    data = SerialRecData()

    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                bluetooth.close()
                pygame.quit()

        getSerialData(data)
        # dummy test data


        # display position of robot and ball on field

        botX, botY = getCoords(data.botAngle, data.botDistance)
        relBallX, relBallY = getCoords(data.ballAngle, data.ballDist)
        ballX = relBallX + botX
        ballY = relBallY + botY

        bot.setPosition(shiftCoords(botX, botY))
        bot.setBox(data.width, data.height)
        ball.setPosition(shiftCoords(ballX, ballY))

        print(f"X confidence: {180 / max(data.width, 1) }")
        print(f"Y confidence: {180 / max(data.height, 1) }")

        draw_bg(screen)
        ball.draw(screen)
        bot.draw(screen)

        pygame.display.update()
        # root.update()


main()

