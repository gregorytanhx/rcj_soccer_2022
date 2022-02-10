

# importing the sys module
import sys         
  
# appending the directory of mod.py 
# in the sys.path list
sys.path.append("C:/Users/gtanh/Documents/thiccWIRES-2022/soccer-simulator-main")   


import pygame, sys, os, math, time, random

from background import draw_bg

import listtools
from algorithm import GeneticAlgorithm
from config import config
from bot import *

# Bring your packages onto the path


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

screen = pygame.display.set_mode((OUTER_WIDTH * SCALE, OUTER_HEIGHT * SCALE))


random.seed(0)
pygame.init()


class Simulation:
    def __init__(self, config):
        self.config = config
        self.algorithm = GeneticAlgorithm(self.config)
        self.generate_initial_population()
        self.bots = []

    def generate_initial_population(self):
        """
        Generate random population of n chromosomes (suitable solutions for the problem)
        Creates a random genome
        """
        random.seed()
        self.population = []
        for _ in range(self.config['population_size']):
            # create a random chromosome with a random gain value
            tmp_k = random.random() * self.config['max_gain_value']
            tmp_a = random.random() * self.config['max_gain_value']
            tmp_b = random.random() * self.config['max_gain_value']
            self.population.append(Attacker(tmp_k, tmp_a, tmp_b))
            

    def generate_new_population(self):
        """
        Generate a new population by repeating following steps until the new population is complete
        """
        new_population = []
        self.fitness_values = []

        # find fitness values of the entire population
        for bot in self.population:
            self.fitness_values.append(bot.fitness)

        # generate a new population based on fitness values
        for chromosomeIndex in range(self.config['population_size']):
            # selection - find two parents of new chromosome
            parentIndices = self.algorithm.selection(self.fitness_values)

            # crossover - generate a child based on
            chromosome = self.algorithm.crossover(self.population[parentIndices[0]], self.population[parentIndices[1]])

            # mutation
            chromosome = self.algorithm.mutation(chromosome)
            new_population.append(chromosome)

        self.population = new_population

    def run(self):
        """
        Run simulation for a specific chromosome c.
        Returns the fitness function value of the simulation
        """

        lastBallTime = 0
        start = True
        startTime = time.time()
        # run for 30 secs each time
        while time.time() - startTime < 10:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False
                    pygame.quit()
                    sys.exit()
        
            
           
                
            
            # new ball position every 2 seconds
            if start or time.time() - lastBallTime >= 2:
                newPos= (random.randint(OUTER_WIDTH - INNER_WIDTH, INNER_WIDTH) * SCALE, random.randint(OUTER_HEIGHT - INNER_HEIGHT, INNER_HEIGHT)* SCALE)
                lastBallTime = time.time()
                for attacker in self.population:
                    attacker.x = ATTACKER_START_X
                    attacker.y = ATTACKER_START_Y
                    attacker.ball.update_pos(attacker, newPos)
                    attacker.initialBallDist = distance(attacker.x, attacker.y, newPos[0], newPos[1])
                    attacker.lastBallTime = time.time()
                
                if start: 
                    start = False
                else:
                    attacker.update_fitness()
            
            else:
                for attacker in self.population:
                    attacker.ball.update_pos(attacker)
                    
            for attacker in self.population:            
                attacker.update_move()
                
            draw_bg(screen)  
            for attacker in self.population:
                attacker.draw(screen)
 
            pygame.display.update()
            

simulation = Simulation(config)

simulation.generate_initial_population()


max_values = []
avg_values = []
k_values = []
a_values = []
b_values = []

# perform simulation

for i in range(config['max_runs']):

    
    simulation.run()
    simulation.generate_new_population()
    fitness_values = simulation.fitness_values
    population = simulation.population

    # add the champion chromosome to a list of champions for plotting
    index_of_champion = listtools.max_index_in_list(fitness_values)
    k_values.append(population[index_of_champion].k)
    a_values.append(population[index_of_champion].a)
    b_values.append(population[index_of_champion].b)


    # add the max/average values to lists for plotting
    max_values.append(listtools.max_value_in_list(fitness_values))
    avg_values.append(listtools.avgList(fitness_values))

    simulation.generate_new_population()
    print ("Run " + str(i) + ": max value " + str(max_values[i]) + ", avg value " + str(avg_values[i]))