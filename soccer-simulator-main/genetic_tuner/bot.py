import math, pygame, time, random
from ball import Ball

OUTER_WIDTH = 182 
OUTER_HEIGHT = 243
INNER_WIDTH = 132
INNER_HEIGHT = 193
SCALE = 4
ATTACKER_START_X = (OUTER_WIDTH * SCALE) // 2
# ATTACKER_START_Y = (OUTER_HEIGHT - 80) * SCALE
ATTACKER_START_Y = OUTER_HEIGHT // 2 * SCALE
OPP_GOALIE_START_X = (OUTER_WIDTH * SCALE) // 2
OPP_GOALIE_START_Y = 180
BLUE_GOAL_LEFT_X = 61 * SCALE
BLUE_GOAL_RIGHT_X = (OUTER_WIDTH - 61) * SCALE
BLUE_GOAL_Y = round((25-7.4) * SCALE)


def degtorad(angle):
	return angle * math.pi/180

def sigmoid(x):
    return 1/(1+math.exp(-x))

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2-y1) ** 2)

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
        self.ball = Ball((OUTER_WIDTH * SCALE) // 2, (OUTER_HEIGHT * SCALE) // 2)
        self.x = x
        self.y = y
        self.radius = 9 * SCALE
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
    
    def get_ball_data(self):
        self.ball_angle = math.degrees(math.pi/2 - math.atan2(self.y - self.ball.y, self.ball.x - self.x))        
        self.ball_dist = math.sqrt((self.y-self.ball.y)**2 + (self.x-self.ball.x)**2)

    def draw(self, screen):
        self.ball.draw(screen)
        pygame.draw.circle(screen, (40, 40, 40), (self.x, self.y), self.radius)
        
    
    def update_ball_pos(self):
        self.ball.update_pos(self)
        

class Attacker(Bot):
    speed = 2
    def __init__(self, offset_k = 1, mult_a = 0.2, mult_b = 0.7):
        super().__init__(ATTACKER_START_X, ATTACKER_START_Y)
        self.MAX_DIST = 800
        self.k = offset_k
        self.a = mult_a
        self.b = mult_b
        self.fitness = 1e-3
        
    def check_ball(self):
        # check collision with ball
        tmp = self.ball_angle
        if tmp > 180: 
            tmp -= 360
        if check_collision(self.x, self.y, self.ball.x, self.ball.y, self.radius, self.ball.diameter):
            if abs(tmp) <= 5:
                    self.ball.caught = True
                    self.has_ball = True
                    self.ball.hit = False
            else:                
                self.ball.hit = True
                self.ball.angle = math.radians(self.ball_angle)
                self.ball.vel = self.speed * 4            
             
        else:
            self.ball.caught = False
            self.has_ball = False
        

    def update_move(self):   
        self.get_ball_data()
        #goal_angle, extreme_goal_angle, goal_dist = self.get_goal_data(opp) 
        #self.angle =  math.degrees(extreme_goal_angle)
    
        self.check_ball()
        if self.has_ball: 
            self.ballCaughtTime = time.time() 
        else:
            self.ballCaughtTime = None

        if self.ball_angle <= 180:
            offset = min(self.ball_angle * self.k, 90)
        else:
            offset = max((360 - self.ball_angle) * self.k, -90)
            
        
        factor = 1 - self.ball_dist/self.MAX_DIST
        ball_mult = self.a * math.exp(self.b * factor)
        move_angle = self.ball_angle + ball_mult * offset
 
        if move_angle > 360: move_angle -= 360
        
        # print(f"ball angle: {round(self.ball_angle)}, move angle: {round(move_angle)}")
        # print(f"ball distance: {self.ball_dist}, ball value: {ball_mult}")
        self.move(move_angle, self.speed) 
            
    def update_fitness(self):

        if self.has_ball:
            self.fitness += self.initialBallDist * 1000 / (self.ballCaughtTime - self.lastBallTime) 
        elif self.ball.hit:
            self.fitness += ((1000 - self.ball_dist) + abs(self.ball_angle)) * 0.7
        else:
            self.fitness += ((1000 - self.ball_dist) + abs(360 - self.ball_angle)) * 0.2
            
        