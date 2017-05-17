import numpy as np
import pandas as pd
import random
import math


def distance(node_1, node_2):
    t_node = node_1 - node_2
    return t_node * np.transpose(t_node)


def norm(node_1, node_2):
    return node_1 * np.transpose(node_2)


def maxDistanceNode(nodeSet, target):
    destNode = target
    maxDis = 0
    for node in nodeSet:
        dis = distance(node, target)
        if dis > maxDis:
            maxDis = dis
            destNode = node
    return destNode


class QueryNode:
    def __init__(self, value):
        self.value = np.matrix(value)
        self.MIP = 0
        self.neighbor = 0


class TreeNode:
    def __init__(self, nodeSet):
        self.nodeSet = nodeSet
        self.center = np.mean(nodeSet, 0)
        self.radius = 0
        self.maxRadius()
        self.leftChild = None
        self.rightChild = None

    def maxRadius(self):
        for node in self.nodeSet:
            self.radius = max(self.radius, distance(self.center, node))

    def size(self):
        return self.nodeSet.shape[0]

    def isLeaf(self):
        if self.leftChild and self.rightChild:
            return False
        else:
            return True


class SingleBallTree:
    def __init__(self, nodeSet, N0):
        self.N0 = N0
        self.root = self.MakeBallTree(nodeSet)

    def MakeBallTree(self, nodeSet):
        tree = TreeNode(nodeSet)
        if tree.size() <= self.N0:
            return tree
        else:
            [A, B] = self.MakeBallTreeSplit(nodeSet)
            leftSet = np.matrix(np.zeros(nodeSet.shape[1]))
            rightSet = np.matrix(np.zeros(nodeSet.shape[1]))
            for node in nodeSet:
                if distance(node, A) <= distance(node, B):
                    leftSet = np.insert(leftSet, 0, node, axis=0)
                else:
                    rightSet = np.insert(rightSet, 0, node, axis=0)
            leftSet = np.delete(leftSet, leftSet.shape[0] - 1, axis=0)
            rightSet = np.delete(rightSet, rightSet.shape[0] - 1, axis=0)
            tree.leftChild = self.MakeBallTree(leftSet)
            tree.rightChild = self.MakeBallTree(rightSet)
            return tree

    def MakeBallTreeSplit(self, nodeSet):
        size = nodeSet.shape[0]
        rand = random.randint(0, size - 1)
        randomNode = nodeSet[rand, :]
        A = maxDistanceNode(nodeSet, randomNode)
        B = maxDistanceNode(nodeSet, A)
        return [A, B]

    def LinearSearch(self, query, nodeSet):
        for node in nodeSet:
            dis = norm(query.value, node)
            if dis > query.MIP:
                query.MIP = dis
                query.neighbor = node

    def Search(self, query):
        self.TreeSearch(query, self.root)

    def TreeSearch(self, query, node):
        if query.MIP < self.MIP(query.value, node):
            if node.isLeaf():
                self.LinearSearch(query, node.nodeSet)
            else:
                left_Len = self.MIP(query.value, node.leftChild)
                right_len = self.MIP(query.value, node.rightChild)
                if left_Len <= right_len:
                    self.TreeSearch(query, node.rightChild)
                    self.TreeSearch(query, node.leftChild)
                else:
                    self.TreeSearch(query, node.leftChild)
                    self.TreeSearch(query, node.rightChild)

    def MIP(self, query, node):
        return norm(query, node.center) + node.radius * math.sqrt(distance(query, query))


def validation(query, nodeSet):
    maxDis = 0
    maxNeight = None
    for node in nodeSet:
        dis = norm(node, query)
        if dis > maxDis:
            maxDis = dis
            maxNeight = node
    return [maxNeight, maxDis]

if __name__ == '__main__':
    data = np.loadtxt('./singBallTree_data')
    data = np.matrix(data)
    tree = SingleBallTree(data, 10)
    query = QueryNode(value=[5.775, 9.251])
    tree.Search(query)
    print query.neighbor
    print query.MIP
    [maxNegiht, maxDis] = validation(query.value, data)
    print maxNegiht
    print maxDis
