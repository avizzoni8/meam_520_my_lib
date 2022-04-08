import numpy as np
from numpy import linalg as la

class Tree: #Really just a list of the leafs of the tree - real action is in the nodes themselves
    #loc =
    def __init__(self,seed, goal,):
        self.seed = seed
        self.goal = goal
        self.leafs = [self.seed]
        self.path = np.array(self.goal.data) #add node itself to list of nodes
        self.delta = 0.05

    def addleaf(self, newnode):
        #New Node is node object
        print('in add leaf')
        #print("finding closest")
        parentnode, dist = self.closestnode(newnode)
        print(dist)
        if dist > self.delta:
            print("dist too large")
            newnode.data = self.delta*newnode.data/la.norm(newnode.data)
        #print("Closest parent data is", parentnode.data)
        newnode.makeparent(parentnode)
        #print("new node has parent", newnode.parent!=None)
        self.leafs = self.leafs + [newnode]
        if parentnode.child == None:#TODO add child check to determine if it is a leaf or not
            #print("closest was a leaf")
            self.leafs.remove(newnode.parent)
            parentnode.child = 1
            return
        else:
            #print("closest was not a leaf")
            return

    def closestnode(self, node):
        q = node.data
        i = 0
        lf = self.leafs
        mindist = abs(la.norm(q - lf[0].data))
        minnode = lf[0]
        while i < len(self.leafs):
            curnode = lf[i]
            while curnode != None:
                # find nearest node based off of norm
                if abs(la.norm(q -curnode.data)) < mindist:
                    mindist = abs(la.norm(q -curnode.data))
                    minnode = curnode
                    curnode = curnode.parent
                else:
                    curnode = curnode.parent
            i+=1
        return minnode, mindist

    def render_path(self,leaf): #only will be called when we have reached the goal
        #last element in leafs will be the end of that path.
        #path = self.path
        #print("in render path")
        if leaf.parent == None:
            #print("current has no parent, end")
            #print("Seed?", self.seed.data.all() == leaf.data.all())
            self.path = np.append(self.seed.data, self.path, axis=0)
            return self.path
        else:
            #print("data to add", leaf.data)
            #print("leaf size",np.shape(leaf.data))
            #print('path size', np.shape(self.path))
            self.path = np.append(leaf.data, self.path, axis=0)
            self.render_path(leaf.parent)
        return self.path

class Node:

    def __init__(self, data, child, parent):
        self.data = data #np.array of joint configurations
        self.child = child #Each new node will start out with boolean False showing it has no children
        self.parent = parent #another node, None if first node

    def makeparent(self, parentnode):
        #new node: random node TREE, parentnode: old node that will be parent
        #both are tree objects
        self.parent = parentnode
        return

#Starting Node
seed = np.random.rand(1,7)
goal = np.zeros([1,7])
seednode = Node(seed, None, None)
goalnode = Node(goal, None, None)
print('seed is', seednode.data)
Tr = Tree(seednode, goalnode)

i = 0
while i < 10:
    new =Node(np.random.rand(1, 7), None, None)
    #print('new is', new.data)
    #print("find closest node \n")
    #print(Tr.closestnode(new).data)
    Tr.addleaf(new)
    #Tr.removeleaf(new)
    #print("# of leafs", len(Tr.leafs))
    #print("Render path", Tr.render_path(new))
    #Tr.path = np.array(Tr.goal.data)
    #print("Render path", len(Tr.render_path(new)))
    #Tr.path = np.array(Tr.goal.data)
    i += 1

print('number of leafs', len(Tr.leafs))
print('Last leaf is', Tr.leafs[-1].data)
print("The path", Tr.render_path(Tr.leafs[-1]))





