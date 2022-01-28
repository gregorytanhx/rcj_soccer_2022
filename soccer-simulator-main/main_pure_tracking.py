
import pygame, sys, os, math, time
from background import draw_bg
from ball import Ball
pygame.init()
OUTER_WIDTH = 182 
OUTER_HEIGHT = 243
INNER_WIDTH = 132
INNER_HEIGHT = 193
SCALE = 4
ATTACKER_START_X = (OUTER_WIDTH * SCALE) // 2
ATTACKER_START_Y = (OUTER_HEIGHT - 80) * SCALE
OPP_GOALIE_START_X = (OUTER_WIDTH * SCALE) // 2
OPP_GOALIE_START_Y = 180
BLUE_GOAL_LEFT_X = 61 * SCALE
BLUE_GOAL_RIGHT_X = (OUTER_WIDTH - 61) * SCALE
BLUE_GOAL_Y = round((25-7.4) * SCALE)

screen = pygame.display.set_mode((OUTER_WIDTH * SCALE, OUTER_HEIGHT * SCALE))
def degtorad(angle):
	return angle * math.pi/180

def sigmoid(x):
    return 1/(1+math.exp(-x))

def map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def rotate(im, angle, pivot):
    # rotate the leg image around the pivot
    image = pygame.transform.rotate(im, angle)
    rect = image.get_rect()
    rect.center = pivot
    return image, rect

def check_collision(x1, y1, x2, y2, r1, r2):
    if abs(x1 - r1//2 - x2) <= r2 or abs(x1 + r1//2 - x2) <= r2:
        if abs(y1 - r1//2 - y2) <= r2 or abs(y1 + r1//2 - y2) <= r2:
            return True
    return False

class Bot(object):
    #arrow_img = pygame.image.load("red-up arrow.png")
    #arrow_img = pygame.transform.scale(arrow_img, (5 * SCALE, 10 * SCALE))
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 10 * SCALE
        self.stop = False
        self.has_ball = False
        self.angle = 0
    
    def move(self, angle, speed):
        if not self.stop:
            a = math.sin(degtorad(50+angle)) * (1/math.sin(degtorad(80)))
            b = math.sin(degtorad(50-angle)) * (1/math.sin(degtorad(80)))

            if abs(a) > abs(b):
                b = b * speed / abs(a) 
                a = speed * a/abs(a)
            else:
                a = a * speed / abs(b)
                b = speed * b/abs(b)
            #print(round(a, 2), end=' ');
            #print(round(b, 2));
            # front left, back right
            self.x += a*math.cos(degtorad(40))
            self.y -= a*math.sin(degtorad(40))

            # front right, back left
            self.x -= b*math.cos(degtorad(40))
            self.y -= b*math.sin(degtorad(40))  
            
    def goto(self, x, y):
        self.x = x
        self.y = y
        
    def kick(self, ball):
        ball.kick = True
        ball.caught = False
        self.has_ball = False
        ball.angle = math.radians(self.angle)
        ball.vel = 3 * self.speed
    
    def get_ball_data(self, ball):
        self.abs_ball_angle = math.degrees(math.pi/2 - math.atan2(self.y - ball.y, ball.x - self.x))
        if self.abs_ball_angle > 180:
            self.abs_ball_angle -= 360
        
        self.ball_dist = math.sqrt((self.y-ball.y)**2 + (self.x-ball.x)**2)

    def draw(self, screen):
        pygame.draw.circle(screen, (40, 40, 40), (self.x, self.y), 10 * SCALE)
        

class Attacker(Bot):
    speed = 2
    def check_ball(self, ball):
        # check collision with ball
        if check_collision(self.x, self.y, ball.x, ball.y, self.radius, ball.diameter):
            if abs(self.rel_ball_angle) <= 2:
                    ball.caught = True
                    self.has_ball = True
                    ball.hit = False
            else:                
                ball.hit = True
                ball.angle = math.radians(self.abs_ball_angle)
                ball.vel = self.speed * 4            
            return 
        
        ball.caught = False
        self.has_ball = False
        
    '''def get_goal_data(self, opp):
        if abs(BLUE_GOAL_LEFT_X - opp.x) > abs(BLUE_GOAL_RIGHT_X - opp.x):
            goal_centre = (BLUE_GOAL_LEFT_X + opp.x - opp.radius//2) // 2
            goal_extreme_point = BLUE_GOAL_LEFT_X + 10
        else:
            goal_centre = (BLUE_GOAL_RIGHT_X + opp.x + opp.radius//2) // 2
            goal_extreme_point = BLUE_GOAL_RIGHT_X - 10
        central_goal_angle = math.pi/2 - math.atan2(self.y - BLUE_GOAL_Y, goal_centre - self.x)
        extreme_goal_angle = math.pi/2 - math.atan2(self.y - BLUE_GOAL_Y, goal_extreme_point - self.x)
        goal_dist = math.sqrt((self.y - BLUE_GOAL_Y) ** 2 + (goal_centre - self.x) ** 2)
        
        return central_goal_angle, extreme_goal_angle, goal_dist'''

    def update_move(self, ball):   
        self.get_ball_data(ball)
        #goal_angle, extreme_goal_angle, goal_dist = self.get_goal_data(opp) 
        #self.angle =  math.degrees(extreme_goal_angle)
        self.rel_ball_angle = self.abs_ball_angle #- self.angle 
        
        self.check_ball(ball)
        '''if self.has_ball and goal_dist < 300:
            ball.hit = False
            self.kick(ball)'''
        dist_thresh = 100
        ball_mult = 1.2 + math.exp((dist_thresh - self.ball_dist)/20)
        move_angle = self.rel_ball_angle * ball_mult
 
        if move_angle > 360: move_angle -= 360
        
        print(f"absolute ball angle: {round(self.abs_ball_angle)}, relative ball angle: {round(self.rel_ball_angle)},  move angle: {round(move_angle)}")
        print(f"ball distance: {self.ball_dist}, ball value: {ball_mult}")
        if self.x > 25 * SCALE + INNER_WIDTH * SCALE:
            self.x = 25 * SCALE + INNER_WIDTH * SCALE
        elif self.x < 25 * SCALE:
            self.x = 25 * SCALE
        
        else:
            self.move(move_angle, self.speed) 
    
class Opp_Goalie(Bot):
    speed = 0.04
    def block(self, ball):
        self.get_ball_data(ball)      
        self.abs_ball_angle = self.abs_ball_angle - 180 if self.abs_ball_angle >= 180 else self.abs_ball_angle + 180
        if ball.x < self.x:
            self.move(270, self.speed * abs(self.x-ball.x))
        else:
            self.move(90, self.speed * abs(self.x-ball.x))
        if self.x > BLUE_GOAL_RIGHT_X:
            self.x = BLUE_GOAL_RIGHT_X
        if self.x < BLUE_GOAL_LEFT_X:
            self.x = BLUE_GOAL_LEFT_X
        
        if check_collision(self.x, self.y, ball.x, ball.y, self.radius, ball.diameter):
            ball.stop = True
            

def main():
    ball = Ball((OUTER_WIDTH * SCALE) // 2, (OUTER_HEIGHT * SCALE) // 2)
    attacker = Attacker(ATTACKER_START_X, ATTACKER_START_Y)
    #opp_goalie = Opp_Goalie(OPP_GOALIE_START_X + 100, OPP_GOALIE_START_Y)
    run = True
    last = time.time()
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()
                sys.exit()
        
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_SPACE]:
            attacker.goto(ATTACKER_START_X, ATTACKER_START_Y)
            ball.caught = False 
            attacker.stop = True          
             
        buttons = pygame.mouse.get_pressed()
        if sum(buttons): 
            attacker.stop = False
            
        attacker.update_move(ball)
        ball.update_pos(attacker)
        #opp_goalie.block(ball)
        #if check_collision(attacker.x, attacker.y, opp_goalie.x, opp_goalie.y, attacker.radius, opp_goalie.radius):
        #    attacker.stop = True
            #opp_goalie.stop = True
            
       # else:
        #    attacker.stop = False
            #opp_goalie.stop = False
            
        #if ball.hit:
        #    time.sleep(10000)
        #    run = False
            
        draw_bg(screen)
        ball.draw(screen)   
        attacker.draw(screen)
        #opp_goalie.draw(screen)   
        pygame.display.update()
        
main()
