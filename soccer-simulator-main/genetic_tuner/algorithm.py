import random
import math
from bot import Attacker
import listtools

class GeneticAlgorithm:
    def __init__(self, config):
        self.config = config

    def mutation(self, chromosome):
        """
        With a mutation probability mutate new offspring at each locus (position in chromosome).
        """
        random.seed()
        #very small real valued mutation
        max_gain_value = self.config['max_gain_value']

        if random.random() < self.config['mutation_probability'] / 3:
            chromosome.k = chromosome.k + random.random()/max_gain_value
        if chromosome.k < 0:
            chromosome.k = random.random() * max_gain_value

        elif random.random() < self.config['mutation_probability'] * 2/3:
            chromosome.a = chromosome.a + random.random()/max_gain_value
        if chromosome.a < 0:
            chromosome.a = random.random() * max_gain_value

        elif random.random() < self.config['mutation_probability']:
            chromosome.b = chromosome.b + random.random()/max_gain_value
        if chromosome.b < 0:
            chromosome.b = random.random() * max_gain_value

        return chromosome

    def crossover(self, parent1, parent2):
        """
        3b[Crossover] With a crossover probability cross over the parents to form a new offspring (children).
        If no crossover was performed, offspring is an exact copy of parents.
        """
        random.seed()

        # if we dont crossover, offspring is a copy of parents
        if random.random() > self.config['crossover_rate']:
            return parent1
        else:
            # random combination crossover
            number = random.random()
            if number < 1.0/6:
                return Attacker(parent1.k, parent1.a, parent1.b)
            elif number < 2.0/6:
                return Attacker(parent2.k, parent1.a, parent1.b)
            elif number < 3.0/6:
                return Attacker(parent1.k, parent2.a, parent1.b)
            elif number < 4.0/6:
                return Attacker(parent1.k, parent1.a, parent2.b)
            elif number < 5.0/6:
                return Attacker(parent2.k,parent1.a, parent2.b)
            else:
                return Attacker(parent2.k,parent2.a, parent2.b)

    def selection(self, fitness_values):
        """
        Pick two parents according to probability represented by normalized fitness values
        3a[Selection] Select two parent chromosomes from a population according to their fitness (the better fitness, the bigger chance to be selected)
        """
        # normalize the list so we have probabilities to pick parents
        fitness_values = listtools.normListSumTo(fitness_values, 1)
        parentIndices = []
        random.seed()
        parent1_probability = random.random()
        parent2_probability = random.random()

        sum = 0
        for i in range(self.config['population_size']):
            if len(parentIndices) == 2:
                break
            next_sum = sum + fitness_values[i]
            if parent1_probability <= next_sum and parent1_probability >= sum:
                parentIndices.append(i)
            if parent2_probability <= next_sum and parent2_probability >= sum:
                parentIndices.append(i)
            sum = next_sum
        return parentIndices

    def fitness(self):
        """
        Evaluate the fitness f(x) of each chromosome x in the population
        returns the fitness value according to the fitness function

        f(x): takes in a list of distances, finds the sum of their squares and returns the inverse of it.
        """
        sum = 0
        for i in range(len(distance_list)):
            sum = sum + distance_list[i]**2
        return 1 /  math.sqrt(sum)

