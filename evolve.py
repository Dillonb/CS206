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

MUTATION_RATE = 0.05

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
    ret = ''
    for row in synapses:
        for synapse in row:
            ret += str(synapse) + ' '
    return ret.strip()

def SaveANN(synapses, filename):
    f = open(filename, 'w')
    f.write(SynapsesToString(synapses))
    f.close()

def Demonstrate(synapses, quiet = False):
    cmd = ['./AppRagdollDemo']
    if quiet:
        cmd.append('-q')
    p = Popen(cmd, stdout=PIPE, stdin=PIPE, stderr=PIPE)
    fitness, errors = p.communicate(input=str.encode(SynapsesToString(synapses)))
    return float(fitness)

def Fitness(synapses):
    """Simulate robot and return fitness"""
    return Demonstrate(synapses, True)

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
    child = MatrixPerturb(parent, MUTATION_RATE)
    childFitness = Fitness(child)
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

