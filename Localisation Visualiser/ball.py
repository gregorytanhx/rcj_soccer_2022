import pygame, math

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


class Ball(object):
    img = pygame.image.load("ball.png")
    img = pygame.transform.scale(img, (7 * SCALE, 7 * SCALE))
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.caught = False
        self.hit = False
        self.kick = False
        self.vel = 0
        self.angle = 0
        self.diameter = self.img.get_height()
        self.stop = False
        
    def move(self, angle, speed):
        if self.stop:
            self.hit = False
        else:
            self.x += math.sin(angle) * speed
            self.y -= math.cos(angle) * speed  
        
    def check_collision(self):
        if self.y - self.diameter // 2 < BLUE_GOAL_Y:
            if self.x > BLUE_GOAL_LEFT_X and self.x < BLUE_GOAL_RIGHT_X:
                return True
        return False
        
    def update_pos(self, bot):
        pygame.event.get()
        buttons = pygame.mouse.get_pressed()
        if sum(buttons):
            new_pos = pygame.mouse.get_pos()
            self.x, self.y = new_pos        
            self.hit = False
            self.kick = False
            self.stop = False
            
        elif not self.check_collision():
            
            if self.caught:
                self.x = bot.x + (bot.radius //2 + self.diameter) * math.sin(bot.angle * math.pi/180)
                self.y = bot.y - (self.diameter + bot.radius //2) * math.cos(bot.angle * math.pi/180)

                
            elif self.hit or self.kick:       
                if self.vel < 0:
                     self.vel = 0 
                     self.kick = False
                     self.hit = False
                self.move(self.angle, self.vel)
                self.vel -= 0.1
            
    def draw(self, screen):
        screen.blit(self.img, (self.x-(7*SCALE//2), self.y-(7*SCALE//2)))