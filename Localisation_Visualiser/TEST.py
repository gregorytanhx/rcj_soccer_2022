
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
OPP_GOALIE_START_Y = 160
BLUE_GOAL_LEFT_X = 61 * SCALE
BLUE_GOAL_RIGHT_X = (OUTER_WIDTH - 61) * SCALE
BLUE_GOAL_Y = round((25-7.4) * SCALE)

screen = pygame.display.set_mode((OUTER_WIDTH * SCALE, OUTER_HEIGHT * SCALE))

def check_collision(x1, y1, x2, y2, r1, r2):
    if abs(x1 - r1//2 - x2) <= r2 or abs(x1 + r1//2 - x2) <= r2:
        if abs(y1 - r1//2 - y2) <= r2 or abs(y1 + r1//2 - y2) <= r2:
            return True
    return False

class Bot(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 10 * SCALE
        self.speed = 3
        self.stop = False
        self.has_ball = False
        self.angle = 0
        
    def move(self, angle, speed):
        if not self.stop:
            self.x += math.sin(angle * math.pi/180) * speed
            self.y -= math.cos(angle * math.pi/180) * speed       
        
    def goto(self, x, y):
        self.x = x
        self.y = y
        
    def check_ball(self, ball):
        # check collision with ball
        if check_collision(self.x, self.y, ball.x, ball.y, self.radius, ball.diameter):
            if abs(self.ball_deg_angle) <= 15 or self.ball_deg_angle >= 345:
                    ball.caught = True
                    self.has_ball = True
                    ball.hit = False
            else:
                ball.caught = True
                self.has_ball = True
                ball.hit = False
                ball.hit = True
                ball.angle = self.ball_rad_angle
                ball.vel = self.speed * 4
            return 
        
        ball.caught = False
        self.has_ball = False

    def draw(self, screen):
        pygame.draw.circle(screen, (40, 40, 40), (self.x, self.y), 10 * SCALE)

class Attacker(Bot):
  
    def goal_data(self, opp):
        if abs(BLUE_GOAL_LEFT_X - opp.x) > abs(BLUE_GOAL_RIGHT_X - opp.x):
            goal_centre = (BLUE_GOAL_LEFT_X + opp.x - opp.radius//2) // 2
        else:
            goal_centre = (BLUE_GOAL_RIGHT_X + opp.x + opp.radius//2) // 2
        goal_angle = math.pi/2 - math.atan2(self.y - BLUE_GOAL_Y, goal_centre - self.x)
        goal_dist = math.sqrt((self.y - BLUE_GOAL_Y) ** 2 + (goal_centre - self.x) ** 2)
        
        return goal_angle, goal_dist
    
    
    def kick(self, ball):
        ball.kick = True
        ball.caught = False
        self.has_ball = False
        ball.angle = self.angle * math.pi/180
        ball.vel = 2 * self.speed
        
    def update_move(self, ball, opp):   
        goal_angle, goal_dist = self.goal_data(opp) 
        self.angle =  goal_angle * 180/math.pi     
           
        self.ball_rad_angle = math.pi/2 - math.atan2(self.y - ball.y, ball.x - self.x)
        self.ball_deg_angle = self.ball_rad_angle * 180/math.pi 
        
        if self.ball_deg_angle < 0:
            self.ball_deg_angle += 360
        
        self.ball_deg_angle -= self.angle 
        self.check_ball(ball)
        if self.has_ball and goal_dist < 250:
            print('kick')
            self.kick(ball)
        self.ball_dist = math.sqrt((self.y-ball.y)**2 + (self.x-ball.x)**2)
        
        if self.ball_dist < 100:
            if abs(self.ball_deg_angle) < 90:
                move_angle = self.ball_deg_angle*2.5
            else:
                if abs(self.ball_deg_angle) >= 180:
                    move_angle = self.ball_deg_angle - 90
                else: 
                    move_angle = self.ball_deg_angle + 90
        else: 
            move_angle = self.ball_deg_angle

        if self.has_ball:
           move_angle = self.angle
        #print(ball.x, ball.y)
        print(f"goal angle: {round(self.angle)}, ball angle: {round(self.ball_deg_angle)} move angle: {round(move_angle)}, ball distance: {self.ball_dist}, has ball: {self.has_ball}")
        self.move(move_angle, self.speed) 
    
class Opp_Goalie(Bot):
    def block(self, ball):
        if ball.x < self.x:
            self.move(270, 1.2 * abs(self.x-ball.x))
        else:
            self.move(90, 1.2 * abs(self.x-ball.x))
            

def main():
    ball = Ball((OUTER_WIDTH * SCALE) // 2, (OUTER_HEIGHT * SCALE) // 2)
    attacker = Attacker(ATTACKER_START_X, ATTACKER_START_Y)
    opp_goalie = Opp_Goalie(OPP_GOALIE_START_X + 100, OPP_GOALIE_START_Y)
    run = True
    angle = 0 
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
            
        attacker.update_move(ball, opp_goalie)
        ball.update_pos(attacker)
        opp_goalie.block(ball)
        if check_collision(attacker.x, attacker.y, opp_goalie.x, opp_goalie.y, attacker.radius, opp_goalie.radius):
            attacker.stop = True
            opp_goalie.stop = True
            
        else:
            attacker.stop = False
            opp_goalie.stop = False
            
        draw_bg(screen)
        ball.draw(screen)   
        attacker.draw(screen)
        opp_goalie.draw(screen)   
          
        
        
        pygame.display.update()
        
main()