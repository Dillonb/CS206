#!/usr/bin/env python
import numpy as np
import random
import copy
from numpy import abs, cos, sin, pi, zeros, transpose
from subprocess import Popen, PIPE, STDOUT
import sys

NUM_SENSORS = 4
NUM_MOTORS = 8

NUM_SYNAPSES = NUM_SENSORS * NUM_MOTORS

NUM_RUNS = 10
NUM_GENERATIONS = 50000

# Number of children to test at once
GENERATION_SIZE=10

MUTATION_RATE = 0.3

def MatrixCreate(x, y):
    return zeros((x, y))

def MatrixPerturb(matrix, prob):
    matrix = copy.deepcopy(matrix)
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if prob >= 1 or prob > random.random():
                matrix[i][j] = (2 * random.random()) - 1
    return matrix

def SynapsesToString(synapses):
    rows = []
    for row in synapses:
        rows.append(' '.join(str(synapse) for synapse in row))
    return ' '.join(rows)

def SaveANN(synapses, filename):
    f = open(filename, 'w')
    f.write(SynapsesToString(synapses))
    f.close()

def Demonstrate(synapses, quiet = False):
    cmd = ['./simulator']
    if quiet:
        cmd.append('-q')
    p = Popen(cmd, stdout=PIPE, stdin=PIPE, stderr=PIPE)
    fitness, errors = p.communicate(input=str.encode(SynapsesToString(synapses)))
    return float(fitness)

def Fitness(synapses):
    """Simulate robot and return fitness"""
    return Demonstrate(synapses, True)

def FitnessList(synapses_list):
    cmd = ['./simulator', '-q']

    processes = []
    results = []

    for synapses in synapses_list:
        processes.append(Popen(cmd, stdout=PIPE, stdin=PIPE, stderr=PIPE))

    for i in range(len(synapses_list)):
        fitness, errors = processes[i].communicate(
            input=str.encode(SynapsesToString(synapses_list[i])))
        results.append(float(fitness))
    return results

def SetTerminalTitle(title):
    sys.stdout.write("\x1b]2;" + title + "\x07")

#neuronValues    = MatrixCreate(NUM_RUNS, NUM_SYNAPSES)

synapses        = MatrixCreate(NUM_SENSORS, NUM_MOTORS)
parentSynapses  = MatrixCreate(NUM_SENSORS, NUM_MOTORS)

for i in range(0, NUM_SENSORS):
    for j in range(0, NUM_MOTORS):
        synapses[i][j] = (random.random() * 2) - 1

# Initial neuron values and positions
#for i in range(0, NUM_NEURONS):
    #neuronValues[0][i] = random.random()

parent = MatrixCreate(NUM_SENSORS, NUM_MOTORS)

parent = MatrixPerturb(parent, 2)

generations = []

parentFitness = Fitness(parent)
generations.append([parent, parentFitness])

for currentGeneration in range(0,NUM_GENERATIONS):
    children = []
    for i in range(GENERATION_SIZE):
        children.append(MatrixPerturb(parent, MUTATION_RATE))

    childFitnesses = FitnessList(children)
    childFitness = childFitnesses[0]

    child = children[0]

    # Don't need to check 0, that's our default
    for i in range(1, GENERATION_SIZE):
        if childFitnesses[i] > childFitness:
            child = children[i]
            childFitness = childFitnesses[i]

    if (childFitness > parentFitness ):
        parent = child
        parentFitness = childFitness
        #Demonstrate(parent)
        SaveANN(parent, "best.ann")
        SaveANN(parent, "fitness_" + str(parentFitness) + "_generation_" + str(currentGeneration) +  ".ann")


    print("%d %f %f"
            %(currentGeneration, parentFitness, childFitness))
    SetTerminalTitle("Evolving: Gen " + str(currentGeneration) + " Fitness: " + str(childFitness) + " Best: " + str(parentFitness))

    generations.append([parent, parentFitness])

