#!/usr/bin/env python3

import random
import copy
from subprocess import Popen, PIPE, STDOUT

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
        self.root = Node()
        self.randomizeANN()

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

        # 5% chance to add a dangling bar
        if random.random() <= 0.05:
            print("Adding a dangling bar")
            new.addDanglingBar()

        # 5% chance to remove a dangling bar
        if random.random() <= 0.05:
            print("Removing a dangling bar")
            new.removeDanglingBar()

        # Always have a 3% chance to change each synaptic weight
        weightsChanged = 0

        for i in range(0, len(new.ann)):
            if random.random() <= 0.03:
                weightsChanged+=1
                new.ann[i] = random.random()

        print("Changed %d syaptic weights"%weightsChanged)

        return new

    fitnessMemo = None
    def fitness(self):
        if self.fitnessMemo is None:
            print("Calculating fitness....")
            cmd = ['./simulator', '-q']

            p = Popen(cmd, stdout=PIPE, stdin=PIPE, stderr=PIPE)
            fit, errors = p.communicate(input=str.encode(self.encode()))

            self.fitnessMemo = float(fit)

        return self.fitnessMemo


GENERATIONS = 100000

parent = Robot()
parentFitness = parent.fitness()

for generation in range(1, GENERATIONS + 1):
    child = parent.mutate()
    childFitness = child.fitness()
    print("====================GENERATION %d=========================="%generation)
    print("%f => %f"%(parentFitness, childFitness))
    if childFitness > parentFitness:
        parent = child
        parentFitness = childFitness
        f = open("best.robot", "w")
        f.write(parent.encode())
        f.close()

