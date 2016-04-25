#!/usr/bin/env python3


class Node:
    def __init__(self, children=None):
        self.children = []

        if children is not None:
            for child in children:
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




n = Node(
    [
        Node(
            [
                Node(),
                Node(),
                Node()
            ]),
        Node([
            Node(),
            Node()
        ])
    ])

print(n.encode())

#print(Node([Node(), Node(), Node()]).encode())
