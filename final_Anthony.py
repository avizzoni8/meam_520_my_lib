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
from core.utils import transform, roll, pitch, yaw

# The library you implemented over the course of this semester!
from calculateFK import FK
from calcJacobian import FK
from solveIK import IK
#from rrt import rrt
from loadmap import loadmap


#TODO make sure we can do this for blue AND red - its just flipping grab and drop

#TODO tag6 drop and tag5 drop

#TODO: Get tag6 function
#TODO: rotate end effector to tag6 orientation
# TODO reset:grab - go to neutral and reorientate


def get_robo_frame(tag):
    pose0 = detector.get_detections()[0][1]
    h = pose0@transform([0.5, 0, 0], [0, 0, 0])
    h = h@tag
    return h

grabpose_3D = np.array([
        [1, 0, 0, 0.562],
        [0, -1, 0, 0.169],
        [0, 0, -1, 0.3],
        [0, 0, 0, 1],
    ])

droppose_3D = np.array([
        [0, 1, 0, 0.562-0.075],
        [0, 0, 1, -0.169+0.075],
        [1, 0, 0, 0.45], #Starts at 0.2+0.005*X
        [0, 0, 0, 1],
    ])

def stack(i,cur_q):
    droppose_3D = np.array([
        [0, 1, 0, 0.562-0.075],
        [0, 0, 1, -0.169+0.075],
        [1, 0, 0, 0.2+0.05*i], #Starts at 0.2+0.005*X
        [0, 0, 0, 1],
    ])

    q = ik.inverse(droppose_3D, cur_q)[0]
    arm.safe_move_to_position(q)
    arm.exec_gripper_cmd(0.1)

def stack_badangle(i,cur_q):
    droppose_3D = np.array([
        [0, 1, 0, 0.562-0.075],
        [0, 0, -1, -0.169+0.075],
        [-1, 0, 0, 0.2+0.05*i], #Starts at 0.2+0.005*X
        [0, 0, 0, 1],
    ])

    q = ik.inverse(droppose_3D, cur_q)[0]
    arm.safe_move_to_position(q)
    arm.exec_gripper_cmd(0.1)

def stack_6up(i,cur_q):
    droppose_3D = np.array([
        [1, 0, 0, 0.562-0.075],
        [0, -1, 1, -0.169+0.075],
        [1, 0, -1, 0.2+0.05*i], #Starts at 0.2+0.005*X
        [0, 0, 0, 1],
    ])

    q = ik.inverse(droppose_3D, cur_q)[0]
    arm.safe_move_to_position(q)
    arm.exec_gripper_cmd(0.1)

def reset():
    arm.safe_move_to_position(arm.neutral_position())
    arm.exec_gripper_cmd(0.08)

def go_grab(q):
    arm.safe_move_to_position(q)
    arm.exec_gripper_cmd(0.045,10)



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

    # now get them in configuration space:
    grabpose = ik.inverse(grabpose_3D, neutral)[0]
    #print('grabpose', grabpose)
    #print('grab  3d',grabpose_3D)
    droppose = ik.inverse(droppose_3D, neutral)[0]
    #print('droppose', droppose)
    #print('drop  3d',droppose_3D)
    #neutral_t6 = ik.inverse(neutral_3d, neutral)[0]

    block_grab = []
    block_hover =[]
    case =[]
    for i in [0,1,2,3]:
        print('tag \n', detector.get_detections()[i+1][1])
        (name, pose) = detector.get_detections()[i+1]

        tag_rf = get_robo_frame(detector.get_detections()[i+1][1])
        print('robo frame \n', tag_rf)

        tag_rf = tag_rf@transform([0,0,0],[0,np.pi,0])
        tag_rf = tag_rf @ transform([0, 0, 0], [0, 0, np.pi/2])
        print("point z down \n", tag_rf)

        tag_rf = tag_rf@transform([0,0,-0.025],[0,0,0])
        print("hover \n", tag_rf)

        if np.arccos(tag_rf[0, 0]) < np.pi / 4 + 0.01:
            if np.arccos(tag_rf[0, 0]) > np.pi / 4 - 0.01:
                case += ['badangle']
                tag_rf = tag_rf @ transform([0, 0, 0], [0, 0, -np.pi])

        #turn it into Q space
        block_hover += [ik.inverse(tag_rf, grabpose)[0]]

        tag_rf = tag_rf@transform([0,0,0.035],[0,0,0])
        print("down to grab \n", tag_rf)
        #Turn it into Q space
        block_grab += [ik.inverse(tag_rf,block_hover[i])[0]]
        case += [None]



    # Master Loop

    #Create
    reset()
    for i in [0,1,2,3]:
        (name, pose) = detector.get_detections()[i+1]

        '''if name == 'tag5':
            run tag5 reset function - Alex
            pass'''


        print("go get block")
        arm.safe_move_to_position(block_hover[i])
        go_grab(block_grab[i])
        print("neutral")
        arm.safe_move_to_position(neutral)

        if name == 'tag6':
            print("tag 6 up")
            print("go to drop")
            stack_6up(i + 1, arm.neutral_position())  # will stack block
            arm.safe_move_to_position(droppose)
            continue

        elif case[i] == 'badangle':
            stack_badangle(i+1, arm.neutral_position())  # will stack block
            arm.safe_move_to_position(droppose)
            continue

        else:
            print("go to drop")
            stack(i+1,arm.neutral_position()) #will stack block
            arm.safe_move_to_position(droppose)

    """Dynamic Loop?"""

    # END STUDENT CODE

    # END STUDENT CODE
