import sys
import numpy as np
from copy import deepcopy



import rospy
import roslib

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
from core.utils import transform

# The library you implemented over the course of this semester!
from calculateFK import FK
from calcJacobian import FK
from solveIK import IK
#from rrt import rrt
from loadmap import loadmap

def get_robo_frame(tag):
    pose0 = detector.get_detections()[0][1]
    h = np.linalg.inv(pose0)
    print("tag0 inv \n", h)
    #h = pose#@transform([-0.5,0,0],[0,0,0])
    #print("tag0 in robo frame \n", h)
    return tag@h


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

    # Detect some tags...
    '''for (name, pose) in detector.get_detections():
         print(name,'\n',pose)
         if name != 'tag0':'''
    (name, pose) = detector.get_detections()[0]
    print(name,'\n',pose)
    (name, pose) = detector.get_detections()[1]
    print(name,'\n',pose)
    print("robo frame \n", get_robo_frame(pose))


    #TODO use inverse and matrix composition to get tags in robot frame


    # Move around...

    '''ik = IK()
    fk = FK()
    #neutral = arm.neutral_position()
    #neutral_3d = fk.forward(arm.neutral_position())[1]
    neutral = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4])
    neutral_3d = fk.forward(neutral)[1]
    print('neutral', neutral)
    print('neutral 3d',neutral_3d)
    # TODO define drop pose and grab pose
    grabpose_3D = np.array([
        [1, 0, 0, 0.562],
        [0, -1, 0, 0.169],
        [0, 0, -1, 0.3],
        [0, 0, 0, 1],
    ])

    droppose_3D = np.array([
        [1, 0, 0, 0.562],
        [0, -1, 0, -0.169],
        [0, 0, -1, 0.3], #Starts at 0.2+0.005*X
        [0, 0, 0, 1],
    ])

    table_pose3d = np.array([
        [1, 0, 0, .2],
        [0, -1, 0, .3],
        [0, 0, -1, 0.5],
        [0, 0, 0, 1],
    ])

    # now get them in configuration space:
    grabpose = ik.inverse(grabpose_3D, neutral)[0]
    droppose = ik.inverse(droppose_3D, neutral)[0]
    table_pose = ik.inverse(table_pose3d, neutral)[0]



    # TODO Drop down & Grab
    # TODO Move to drop pose and place block
    # TODO reset


    # Move around...

    print('pose 3d',table_pose3d)
    print('pose 7d',table_pose)
    print('bring it back',fk.forward(droppose)[1])
    #arm.safe_move_to_position(arm.neutral_position())

    #arm.safe_move_to_position(grabpose)
    #arm.safe_move_to_position(droppose)'''

    # END STUDENT CODE

    # END STUDENT CODE
