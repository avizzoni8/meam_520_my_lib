import numpy as np
#from planningtree import Tree
#from planningtree import Node
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
from lib.calculateFK import FK
from numpy import linalg as la

class Node: #The random points

    def __init__(self, data, child, parent):
        self.data = data #np.array of joint configurations
        self.child = child  #Each new node will start out with boolean False showing it has no children
        self.parent = parent #another node, None if first node

    def makeparent(self, parentnode): #Attach it to a parent node in the tree
        self.parent = parentnode
        return


class Tree: #Really just a list of the leafs of the tree - real action is in the nodes themselves

    def __init__(self,seed, goal,):
        self.seed = seed
        self.goal = goal
        self.leafs = [self.seed] #list of end leafs
        self.path = np.array(self.goal.data) #add node itself to list of nodes
        self.delta = 1.5 #make distance between points

    def addleaf(self, newnode):
        #New Node is node object
        parentnode, dist = self.closestnode(newnode)

        #Check to see if the distance between new node and closest node is larger than delta
        if dist > self.delta:
            #Normalize the vector pointing between the points, multiply by delta, add it to parent
            #Update the data of the new node with that new location
            newnode.data = parentnode.data+self.delta*(newnode.data-parentnode.data)/la.norm(newnode.data-parentnode.data)

        #Make the closestnode the parent of the newnode
        newnode.makeparent(parentnode)

        #Add it to the list of lefs
        self.leafs = self.leafs + [newnode]

        #check is the parent is a leaf or now
        if parentnode.child == None:
            #Is a leaf so remove it from list of leafs and update child boolean
            self.leafs.remove(newnode.parent)
            parentnode.child = 1
            return
        else:
            return

    def closestnode(self, node):
        #Iterate through the paths of each leaf back to the seed.

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
                    #Current node is closer than current closest - update
                    mindist = abs(la.norm(q -curnode.data))
                    minnode = curnode
                    #Move down the branch - update current node to current's parent
                    curnode = curnode.parent
                else:
                    #Current isn't closest so keep going down the branch
                    curnode = curnode.parent
            i+=1
        return minnode, mindist

    def render_path(self,leaf):
        #Leaf is leaf node you want the path of - will call this when we get to the end
        if leaf.parent == None:
            #at the end so must be at the seed. Add it to the top of the path
            self.path = np.append(self.seed.data, self.path, axis=0)
            return self.path
        else:
            #Add data to top of the path
            self.path = np.append(leaf.data, self.path, axis=0)
            self.render_path(leaf.parent)
        return self.path

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

    start.resize(1, 7)
    goal.resize(1, 7)

    #Create those nodes
    seednode = Node(start, None, None)
    goalnode = Node(goal, None, None)

    #initialize tree
    Tr = Tree(seednode, goalnode)

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    # Initialize FK Class
    fk = FK()

    maxsteps = 100
    i = 0
    while i < maxsteps: #TODO:

        #q_rand_config = np.random.rand(1,7) * (upperLim - lowerLim) + lowerLim
        q_rand_config = np.random.uniform(lowerLim, upperLim)

        #Add node first because it may update the configuration
        newnode = Node(q_rand_config, None, None)
        Tr.addleaf(newnode)

        # Convert random config to 3d workspace after it has been added and potentially updated
        q_rand_work = fk.forward(newnode.data[0])[0]

        #Check if I'm in obstacle
        for obstacle in map.obstacles:
            obs = obstacle + [-0.15, -0.15, -0.15, 0.15, 0.15, 0.15] #enlarge to account for link thickness
            if detectCollision(q_rand_work[:-1], q_rand_work[1:], obs)[0]:
                print("We have a collision")
                i+=1
                Tr.leafs.remove(newnode) #Remove it from set of leafs since its not a viable configuration
                return
            else:
                pass

        #TODO Check if i can move between successive nodes
        for obstacle in map.obstacles:
            obs = obstacle + [-0.15, -0.15, -0.15, 0.15, 0.15, 0.15]# enlarge to account for link thickness
            previous, _ = fk.forward(newnode.parent.data[0])
            #print('Previous node', previous)
            #print('current node', q_rand_work)
            if detectCollision(q_rand_work, previous, obs)[0]:
                print("We can't a move here")
                Tr.leafs.remove(newnode)
                i += 1
                return
            else:
                print("We can move here")
                pass

        #TODO Check if I can get to goal from here
        for obstacle in map.obstacles:
            obs = obstacle + [-0.15, -0.15, -0.15, 0.15, 0.15, 0.15]# enlarge to account for link thickness
            goal_jl, _ = fk.forward(goal[0])
            reach = False
            if detectCollision(q_rand_work, goal_jl, obs)[0]:
                print("Cant reach goal from here")
                pass
            else:
                #we can reach the goal from here
                print("Looks like we are at solution")
                # Get path and return it
                path = Tr.render_path(newnode)
                return path

        i+=1

    print("end loop")

    return path


if __name__ == '__main__':

    map_struct = loadmap("..maps/map3.txt")
    start = np.array([0, -1, 0, -2, 0, 1.57, 0])
    goal = np.array([-1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

    '''

    lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    success = []
    i = 0
    while i < 10:

        start = np.random.uniform(lowerLim,upperLim)
        goal = np.random.uniform(lowerLim,upperLim)
        path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
        if path[0]!=[]:
            success=success+[1]
        else:
            success=success+[0]
        print(path[1])
        i+=1

    print("success rate", np.mean(success))'''