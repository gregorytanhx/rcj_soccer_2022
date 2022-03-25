from math import sin, cos, pi
import pygame, time

pygame.init()
screen = pygame.display.set_mode((1000, 1000))


last = time.time()
def degtorad(angle):
	return angle * pi/180

angle = 0
x = 20
y = 400
x1 = 400
y1 = 400
while angle < 360:
    x = 200
    y = 400
    x1 = 200
    y1 = 400
    while time.time() - last < 0.2:
        #pygame.draw.rect(screen, (0,0,0), pygame.Rect(0, 0, 800, 800))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

    
        a = sin(degtorad(50+angle)) * (1/sin(degtorad(80)))
        b = sin(degtorad(50-angle)) * (1/sin(degtorad(80)))

        x_co = sin(degtorad(angle)) * 0.7778619134302062
        y_co = cos(degtorad(angle)) * 0.7778619134302062
        # x_co = sin(degtorad(angle)) * 0.7071067811865476
        # y_co = cos(degtorad(angle)) * 0.7071067811865476
        a1 = x_co + y_co
        b1 = -x_co + y_co


        oldX = x
        oldY = y
        oldX1 = x1
        oldY1 = y1
        # front left, back right
        x += a*cos(degtorad(40))
        y -= a*sin(degtorad(40))

        # front right, back left
        x -= b*cos(degtorad(40))
        y -= b*sin(degtorad(40))

        # front left, back right
        x1 += a1*cos(degtorad(40))
        y1 -= a1*sin(degtorad(40))

        # front right, back left
        x1 -= b1*cos(degtorad(40))
        y1 -= b1*sin(degtorad(40))
        # # front left, back right
        # x1 += a1*cos(degtorad(45))
        # y1 -= a1*sin(degtorad(45))

        # # front right, back left
        # x1 -= b1*cos(degtorad(45))
        # y1 -= b1*sin(degtorad(45))

        pygame.draw.line(screen, (255, 0, 0), (oldX, oldY), (x, y), 2)
        pygame.draw.line(screen, (0, 255, 0), (oldX1, oldY1), (x1, y1), 2)
        pygame.display.update()
    #print(time.time(), last)
    last = time.time()
    angle += 1
    if angle == 360:
        angle = 0
