import pygame

OUTER_WIDTH = 182 
OUTER_HEIGHT = 243
INNER_WIDTH = 132
INNER_HEIGHT = 193
SCALE = 4

def draw_bg(screen):
    # green field
    pygame.draw.rect(screen, (23, 138, 79), pygame.Rect(0, 0, OUTER_WIDTH * SCALE, OUTER_HEIGHT * SCALE))
    # white line
    pygame.draw.rect(screen, (255, 255, 255), 
                        pygame.Rect(25 * SCALE, 25 * SCALE, INNER_WIDTH * SCALE, INNER_HEIGHT * SCALE), 2 * SCALE)
    # penalty area

    pygame.draw.rect(screen, (255, 255, 255), 
                     pygame.Rect(56 * SCALE, 24 * SCALE, 70 * SCALE, 25 * SCALE), 2 * SCALE, 
                     0, 0, 0, 15 * SCALE, 15 * SCALE)
    pygame.draw.rect(screen, (255, 255, 255), 
                     pygame.Rect(56 * SCALE, (OUTER_HEIGHT - 49) * SCALE, 70 * SCALE, 25 * SCALE), 2 * SCALE,
                     0, 15 * SCALE, 15 * SCALE, 0, 0)
    
    # goals
    pygame.draw.line(screen, (0, 0, 255), (61 * SCALE, round((25-7.4) * SCALE)), (61 * SCALE, 26 * SCALE), 2 * SCALE)
    pygame.draw.line(screen, (0, 0, 255), ((OUTER_WIDTH - 61) * SCALE, round((25-7.4) * SCALE)), ((OUTER_WIDTH - 61) * SCALE, 26 * SCALE), 2 * SCALE)     
    pygame.draw.line(screen, (0, 0, 255), (61 * SCALE, round((25-7.4) * SCALE)), ((OUTER_WIDTH - 61) * SCALE, round((25-7.4) * SCALE)), 2 * SCALE)
    
    pygame.draw.line(screen, (255, 255, 0), (61 * SCALE, (OUTER_HEIGHT-round((25-7.4))) * SCALE), (61 * SCALE, (OUTER_HEIGHT-26) * SCALE), 2 * SCALE)
    pygame.draw.line(screen, (255, 255, 0), ((OUTER_WIDTH - 61) * SCALE, (OUTER_HEIGHT-round((25-7.4))) * SCALE), ((OUTER_WIDTH - 61) * SCALE, (OUTER_HEIGHT-26) * SCALE), 2 * SCALE)     
    pygame.draw.line(screen, (255, 255, 0), (61 * SCALE, (OUTER_HEIGHT-round((25-7.4))) * SCALE), ((OUTER_WIDTH - 61) * SCALE, (OUTER_HEIGHT-round((25-7.4))) * SCALE), 2 * SCALE)
    # centre circle
    pygame.draw.circle(screen, (0, 0, 0), ((OUTER_WIDTH * SCALE) // 2, (OUTER_HEIGHT * SCALE) // 2), 30 * SCALE, 2)
    
    
    neutral_spots = [(50 * SCALE, 65 * SCALE), ((OUTER_WIDTH - 50) * SCALE, 65 * SCALE),
                 (50 * SCALE, (OUTER_HEIGHT - 65) * SCALE), ((OUTER_WIDTH - 50) * SCALE, (OUTER_HEIGHT - 65) * SCALE),
                 ((OUTER_WIDTH * SCALE) // 2, (OUTER_HEIGHT * SCALE) // 2)]

    # neutral spots
    for i in neutral_spots:
        pygame.draw.circle(screen, (0, 0, 0), i, 4)