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


# TODO Drop down & Grab
# TODO Move to drop pose and place block
# TODO reset:drop - go to neutral and reorientate
# TODO reset:grab - go to neutral and reorientate


def get_robo_frame(tag):
    pose0 = detector.get_detections()[0][1]
    h = pose0@transform([0.5, 0, 0], [0, 0, 0])
    h = h@tag
    return h

def stack(i,cur_q):
    droppose_3D = np.array([
        [1, 0, 0, 0.562],
        [0, -1, 0, -0.169],
        [0, 0, -1, 0.2+0.005*i], #Starts at 0.2+0.005*X
        [0, 0, 0, 1],
    ])

    q = ik.inverse(droppose_3D, cur_q)[0]
    arm.safe_move_to_position(q)
    arm.open_gripper()

def reset():
    arm.safe_move_to_position(arm.neutral_position())
    arm.open_gripper()

def go_grab(q):
    arm.safe_move_to_position(q)
    arm.exec_gripper_command(0.045,10)




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

    # Move around...

    ik = IK()
    fk = FK()
    #neutral = arm.neutral_position()
    #neutral_3d = fk.forward(arm.neutral_position())[1]
    neutral = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4])
    neutral_3d = fk.forward(neutral)[1]
    #print('neutral', neutral)
    #print('neutral 3d',neutral_3d)
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

    # now get them in configuration space:
    grabpose = ik.inverse(grabpose_3D, neutral)[0]
    #print('grabpose', grabpose)
    #print('grab  3d',grabpose_3D)
    droppose = ik.inverse(droppose_3D, neutral)[0]
    #print('droppose', droppose)
    #print('drop  3d',droppose_3D)

    blocks_q = []
    for i in [1,2,3,4]:
        tag_rf = get_robo_frame(detector.get_detections()[i][1])
        tag_rf = tag_rf@np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        blocks_q += [ik.inverse(tag_rf, grabpose)[0]]



    # Master Loop

    #Create
    reset()
    for i in [1,2,3,4]:
        print("go get block")
        go_grab(blocks_q[i])
        """grab function - need to fix orientation"""
        print("neutral")
        arm.safe_move_to_position(arm.neutral_position())
        print("go to drop")
        stack(i,arm.neutral_position()) #will stack block

    """Dynamic Loop?"""

    # END STUDENT CODE

    # END STUDENT CODE
