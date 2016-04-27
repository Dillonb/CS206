#!/usr/bin/env python3

# Constants
GENERATIONS = 100000

INITIAL_POPULATION_SIZE = 100
NUM_MOST_FIT = 10
NUM_CHILDREN_PER = 10

THREADS = 9

ADD_BAR_RATE = 0.1
REMOVE_BAR_RATE = 0.05
CHANGE_SYNAPTIC_WEIGHT_RATE=0.03

import random
import copy
from subprocess import Popen, PIPE, STDOUT
from multiprocessing import Pool

class Node:
    def __init__(self, children=None):
        self.children = []

        if children is not None:
            for child in children:
                child.parent = self
                self.children.append(child)

    def is_leaf(self):
        return len(self.children) == 0

    # Recursively encode the tree as a string by DFS
    def encode(self):
        # Number of children plus one for this node
        totalNodes = self.num_children_deep() + 1
        # Height
        height = self.height()
        # Encoding of this node and all children
        encoding = self.encodeRecurse().strip()
        return "%d %d %s"%(totalNodes, height, encoding)

    def encodeRecurse(self):
        ret = str(len(self.children)) + " "

        for child in self.children:
            ret += child.encodeRecurse()

        return ret

    def num_children(self):
        return len(self.children)

    def num_children_deep(self):
        total = self.num_children()
        for child in self.children:
            total += child.num_children_deep()

        return total

    def height(self):
        if self.num_children() == 0:
            return 1
        # Find the height of the biggest subtree
        return 1 + max([child.height() for child in self.children])

    def random_leaf(self):
        if self.is_leaf():
            return self
        else:
            return random.choice(self.children).random_leaf()

    def unsaturated_children(self):
        unsaturatedChildren = []
        for child in self.children:
            if len(child.children) <= 4:
                unsaturatedChildren.append(child)

            unsaturatedChildren += child.unsaturated_children()
        return unsaturatedChildren

    def random_unsaturated_node(self):
        nodes = self.unsaturated_children()
        if len(self.children) <= 4:
            nodes.append(self)

        return random.choice(nodes)

class Robot:
    def __init__(self):
        self.root = Node([Node()])
        self.randomizeANN()
        self.fitnessMemo = None

    def __eq__(self, other):
        return self.minimalencode() == other.minimalencode()

    def __hash__(self):
        return self.minimalencode().__hash__()

    def get_active_ann(self):
        active_ann = []
        numnodes = self.root.num_children_deep() + 1
        for i in range(0, numnodes - 1):
            for j in range(0, numnodes):
                index = i * 98 + j
                active_ann.append(self.ann[index])
        return active_ann

    def minimalencode(self):
        return "%s %s"%(self.root.encode(), " ".join([str(x) for x in self.get_active_ann()]))

    def randomizeANN(self):
        self.ann = []
        for i in range(0, 9900):
            self.ann.append(random.random() * 2 - 1)

    def encode(self):
        return self.root.encode() + " " + " ".join([str(x) for x in self.ann])

    def addDanglingBar(self):
        parent = self.root.random_unsaturated_node()
        child = Node()
        child.parent = parent
        parent.children.append(child)

    def removeDanglingBar(self):
        if self.root.is_leaf():
            # Root is leaf node, can't remove anything
            return
        toRemove = self.root.random_leaf()
        toRemove.parent.children.remove(toRemove)

    def mutate(self):
        new = copy.deepcopy(self)
        new.fitnessMemo = None

        if random.random() <= ADD_BAR_RATE:
            #print("Adding a dangling bar")
            new.addDanglingBar()

        if random.random() <= REMOVE_BAR_RATE:
            #print("Removing a dangling bar")
            new.removeDanglingBar()

        # Always have a 3% chance to change each synaptic weight
        weightsChanged = 0

        for i in range(0, len(new.ann)):
            if random.random() <= CHANGE_SYNAPTIC_WEIGHT_RATE:
                weightsChanged+=1
                new.ann[i] = random.random()

        #print("Changed %d syaptic weights"%weightsChanged)

        return new

    def getChildren(self, numChildren):
        children = []
        for i in range(0, numChildren):
            children.append(self.mutate())
        return children

    def fitness(self):
        if self.fitnessMemo is None:
            cmd = ['./simulator', '-q']

            p = Popen(cmd, stdout=PIPE, stdin=PIPE, stderr=PIPE)
            fit, errors = p.communicate(input=str.encode(self.encode()))

            self.fitnessMemo = float(fit)

        return self.fitnessMemo


population = []
for i in range(0, INITIAL_POPULATION_SIZE):
    population.append(Robot())

population = list(set(population))

top_fitness = 0

def calculateFitnessMemo(robot):
    robot.fitness()
    return robot

for generation in range(1, GENERATIONS + 1):
    print("====================GENERATION %d=========================="%generation)

    print("Finding fitness values of population...")

    # Create memoization results of fitness() in parallel
    with Pool(processes=THREADS) as pool:
        population = pool.map(calculateFitnessMemo, population)

    # Sort the population by fitness, highest first
    population.sort(key=lambda robot: robot.fitness(), reverse=True)
    most_fit = population[:10]

    if most_fit[0].fitness() > top_fitness:
        print("New best fitness! %f => %f"%(top_fitness, most_fit[0].fitness()))
        top_fitness = most_fit[0].fitness()

        f = open("best.robot", "w")
        f.write(most_fit[0].encode())
        f.close()
    else:
        print("No new best.")

    print("Best robot has %d body parts."%(1 + most_fit[0].root.num_children_deep()))

    print("Top 10 fitness values: " + " ".join([str(robot.fitness()) for robot in most_fit]))

    new_population = copy.copy(most_fit)

    print ("Mutating new unique children for next generation...")
    while len(new_population) < len(most_fit) * NUM_CHILDREN_PER:
        for robot in most_fit:
            new_population += robot.getChildren(NUM_CHILDREN_PER)
            #print("len(new_population): %d"%len(new_population))
            # Remove duplicates the fun way
            new_population = list(set(new_population))
            #print("len(new_population): %d"%len(new_population))
    print("Population size: %d"%len(new_population))

    population = new_population
