import sys
import numpy as np
from copy import deepcopy
from lib import solveIK

import rospy
import roslib

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# The library you implemented over the course of this semester!
from lib.calculateFK import FK
from lib.calcJacobian import FK
from lib.solveIK import IK
from lib.rrt import rrt
from lib.loadmap import loadmap

if __name__ == "__main__":

    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    arm.safe_move_to_position(arm.neutral_position()) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE
    def rotx(theta):
        mat = np.array([[1,0,0,0],[0,np.cos(theta), - np.sin(theta),0],[0,np.sin(theta),np.cos(theta),0],[0,0,0,1])
        return mat

    def roty(theta):
        mat = np.array([[np.cos(theta),0,np.sin(theta),0],[0,1,0,0],[-np.sin(theta),0,np.cos(theta),0],[0,0,0,1])
        return mat

    def rotz(theta):
        mat = np.array([[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
        return mat

    def trans(x,y,z):
        mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[x,y,z,1]])
        return mat

    # Detect some tags...
    for (name, pose) in detector.get_detections():
         print(name,'\n',pose)

         if name == "tag0": #do nothing - this is the ground
             continue
         elif name == "tag1": #need to rotate and transate to get to the center
             rot = pose[0:3,0:3] #get rotation
             new_rot = roty(-np.pi/2)@(rotx(-np.pi/2)@rot) #transform rotation to correct orientation
             disp = pose[0:3,3] #get displacement
             new_disp = disp + (-25)*(new_rot[0,:]) #translate to center of block in certain direction
             #unsure on units this is mm

        #All the tag code is identical to tag 1 apart from the different transformations and rotations
         elif name == "tag2":
             rot = pose[0:3,0:3]
             new_rot = roty(np.pi)@(rotx(-np.pi/2)@rot)
             disp = pose[0:3,3]
             new_disp = disp + (-25)*(new_rot[2,:])

         elif name == "tag3":
             rot = pose[0:3,0:3]
             new_rot = roty(np.pi/2)@(rotx(-np.pi/2)@rot)
             disp = pose[0:3,3]
             new_disp = disp + (25)*(new_rot[0,:])

         elif name == "tag4":
             rot = pose[0:3,0:3]
             new_rot = rotx(-np.pi/2)@rot
             disp = pose[0:3,3]
             new_disp = disp + (25)*(new_rot[2,:])

         elif name == "tag5":
             rot = pose[0:3,0:3]
             new_rot = roty(-np.pi/2)@(rotx(np.pi)@rot)
             disp = pose[0:3,3]
             new_disp = disp + (25)*(new_rot[1,:])

         elif name == "tag6":
             rot = pose[0:3,0:3]
             new_rot = roty(np.pi/2)@rot
             disp = pose[0:3,3]
             new_disp = disp + (-25)*(new_rot[1,:])

         new_transform = np.hstack([new_rot[0,:],new_rot[1,:],new_rot[2,:],new_disp]) #creating the homogeous transformation
         #final_row = np.array([0,0,0,1])
         #new_transform = np.vstack([new_transform,final_row]) #I can't seem to get the final row added...
         print(new_transform)
         #unsure how we should save these new homogenous transforms...

    neutral = arm.neutral_position()
    neutral_3d = fk.forward(arm.neutral_position())[1]
    #TODO define drop pose and grab pose
    grabpose_3D = np.array([
        [ 1,  0, 0, 1.147 ],
        [ 0,  1, 0, 0.562 ],
        [ 0,  0, -1, 0.528 ],
        [ 0,  0, 0, 1 ],
    ])

    droppose_3D = np.array([
        [ 1,  0,  0, 0.884 ],
        [ 0,  1,  0, 0.487 ],
        [ 0,  0, -1, 0.528 ],
        [ 0,  0, 0, 1 ],
    ])

    #now get them in configuration space:

    grabpose = solveIK(grabpose_3D,neutral)[0]
    droppose = solveIK(droppose_3D, neutral)[0]
    #TODO Drop down & Grab
    #TODO Move to drop pose and place block
    #TODO reset

    # Move around...
    arm.safe_move_to_position(arm.neutral_position())

    arm.safe_move_to_position(grabpose)
    arm.safe_move_to_position(droppose)

    # END STUDENT CODE
