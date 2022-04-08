import numpy as np
#from planningtree import Tree
#from planningtree import Node
from detectCollision import detectCollision
from loadmap import loadmap
from copy import deepcopy
from calculateFK import FK
from numpy import linalg as la

class Tree: #Really just a list of the leafs of the tree - real action is in the nodes themselves
    #loc =
    def __init__(self,seed, goal,):
        self.seed = seed
        self.goal = goal
        self.leafs = [self.seed]
        self.path = np.array(self.goal.data) #add node itself to list of nodes
        self.delta = 1.5

    def addleaf(self, newnode):
        #New Node is node object
        #print('in add leaf')
        #print("finding closest")
        parentnode, dist = self.closestnode(newnode)
        #print(dist)
        if dist > self.delta:
            #print("dist too large")
            newnode.data = parentnode.data+self.delta*(newnode.data-parentnode.data)/la.norm(newnode.data-parentnode.data)
            #print("new data", newnode.data)
        #print("Closest parent data is", parentnode.data)
        newnode.makeparent(parentnode)
        #print("new node has parent", newnode.parent!=None)
        self.leafs = self.leafs + [newnode]
        if parentnode.child == None:
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

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # initialize path
    path = []

    #print('start input',start)
    start.resize(1, 7)
    #print("start resized", start)

    #print(type(start))
    goal.resize(1, 7)
    #print('goal resized',goal)
    seednode = Node(start, None, None)
    goalnode = Node(goal, None, None)
    #initialize tree
    Tr = Tree(seednode, goalnode)

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    #goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])

    # Initialize FK Class
    fk = FK()

    Path = []
    jointsfound = []

    seed = np.random.rand(1, 7)*(upperLim - lowerLim) + lowerLim

    for j in [0,1,2,3,4,5,6]:
        maxsteps = 500
        i = 0
        print("Looking for joint", j)
        print("Goal is", goal[0][j])
        print("Random of each interate", random(j, seed, upperLim, lowerLim))
        success = False
        while i < maxsteps: #TODO:
            q_rand_config = random(j, seed, upperLim, lowerLim)
            #print("random q", q_rand_config)# Generate random config
            #print("the joint loc", q_rand_work)
            newnode = Node(q_rand_config, None, None)
            Tr.addleaf(newnode)
            # Convert random config to 3d workspace
            q_rand_work = fk.forward(newnode.data[0])[0]

            for obstacle in map.obstacles:
                #print("checking for collisions")
                obs = obstacle + [-0.15, -0.15, -0.15, 0.15, 0.15, 0.15] #enlarge to account for link thickness
                if detectCollision(q_rand_work[:-1], q_rand_work[1:], obs)[0]:
                    print("We have a collision")
                    i+=1
                    Tr.leafs.remove(newnode)
                    return
                else:
                    #print("no collision")
                    pass
                #Configuration is not valid because of collision, get new one no need to continue checking

            #Now point is okay, so add it to tree as node
            #print("create new node")

            if abs(goal[0][j] - newnode.data[0][j]) < 1e-1:
                print("Looks like we are at solution")
                #Get path and return it
                #Path = Path + Tr.render_path(newnode)
                jointsfound = jointsfound + [newnode.data[0][j]]
                seed = jointsfound
                success = True
                break
            else:
                #print("No, keep going")
                pass
            i+=1

        print(success)
        print("end loop")
        '''for i in Tr.leafs:
            print(i.data)'''

        #print(Tr.render_path(Tr.leafs[-1]))
        #print(np.shape(Tr.path))
    return jointsfound


def random(i,seed,uplim,lowlim):
    if i == 0:
        return seed
    else:
        rand = np.random.rand(1, 7 - i)*(uplim[i:] - lowlim[i:]) + lowlim[i:]
        return np.append(seed, rand)

def dist(x, y):
    return None
   # TODO: Implement

if __name__ == '__main__':
    map_struct = loadmap("maps/map3.txt")

    start = np.array([0,-1,0,-2,0,1.57,0])

    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    '''lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    goal = np.random.rand(1, 7) * (upperLim - lowerLim) + lowerLim'''
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print(path)
