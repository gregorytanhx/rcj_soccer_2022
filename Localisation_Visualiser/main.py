
import pygame
import math
import sys
import os
import time
import serial
from background import draw_bg

bluetooth = serial.Serial(port="COM10", baudrate=9600, timeout=0)

# field dimensions
OUTER_WIDTH = 182 
OUTER_HEIGHT = 243
INNER_WIDTH = 132
INNER_HEIGHT = 193
SCALE = 4
BUFFER_SIZE = 8


def shiftCoords(x, y):
    x += OUTER_WIDTH // 2
    y = OUTER_HEIGHT // 2 - y
    return x, y


def getCoords(angle, distance):
    # convert mm dist to cm
    distance /= 10
    # convert coordinates from polar to cartesian
    x = distance * math.sin(math.radians(angle)) 
    y = distance * math.cos(math.radians(angle)) 
    #print(f"Coordinates X: {int(x)} Y: {int(y)}")
    # convert from centre origin to top left origin
    
    return x, y

class SerialData:
    def __init__(self):
        self.angle = 0
        self.distance = 0
        self.ballAngle = 0
        self.ballDist = 0       
    
class Ball(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 4 * SCALE
        self.img = img = pygame.image.load("ball.png")
        self.img = pygame.transform.scale(img, (self.radius, self.radius))
        
    def setPosition (self, coords):
        self.x = int(coords[0]) * SCALE
        self.y = int(coords[1]) * SCALE

    def draw(self, screen):
        screen.blit(self.img, (self.x-self.radius // 2, self.y - self.radius // 2))
        
class Bot(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 9 * SCALE
        
    def setPosition(self, coords):
        self.x = int(coords[0]) * SCALE
        self.y = int(coords[1]) * SCALE
            
    def draw(self, screen):
        pygame.draw.circle(screen, (40, 40, 40), (self.x, self.y), self.radius)
        
        
def getSerialData(data):
    while bluetooth.in_waiting >= BUFFER_SIZE + 1:
        buffer = bytearray(bluetooth.readline())
    
        data.angle = int.from_bytes(buffer[0:2], byteorder = 'little')       
        data.distance = int.from_bytes(buffer[2:4], byteorder = 'little')  
        data.ballAngle = int.from_bytes(buffer[4:6], byteorder = 'little')
        data.ballDist = int.from_bytes(buffer[6:8], byteorder = 'little')
    
       
def main():   
    pygame.init()
    screen = pygame.display.set_mode((OUTER_WIDTH * SCALE, OUTER_HEIGHT * SCALE))
    
    ball = Ball(OUTER_WIDTH // 2, OUTER_HEIGHT // 2)
    bot = Bot(OUTER_WIDTH // 2, OUTER_HEIGHT // 2)
    run = True
    data = SerialData()
   
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                bluetooth.close()
                pygame.quit()     
                   
        getSerialData(data)
        botX, botY = getCoords(data.angle, data.distance)
        relBallX, relBallY = getCoords(data.ballAngle, data.ballDist)
        ballX = relBallX + botX
        ballY = relBallY + botY
        
        bot.setPosition(shiftCoords(botX, botY))
        ball.setPosition(shiftCoords(ballX, ballY))
        
        draw_bg(screen)
        ball.draw(screen)   
        bot.draw(screen)
        pygame.display.update()
    
try: 
    main()
except:
    bluetooth.close()
