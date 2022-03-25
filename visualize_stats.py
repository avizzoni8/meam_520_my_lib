import sys
from math import pi, sin, cos
import numpy as np
from time import perf_counter
import random

import rospy
import roslib
import tf
import geometry_msgs.msg

from core.interfaces import ArmController

from lib.solveIK import IK

rospy.init_node("visualizer")

# Using your solution code
ik = IK()

#########################
##  RViz Communication ##
#########################

tf_broad  = tf.TransformBroadcaster()

# Broadcasts a frame using the transform from given frame to world frame
def show_pose(H,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(H),
        tf.transformations.quaternion_from_matrix(H),
        rospy.Time.now(),
        frame,
        "world"
    )

#############################
##  Transformation Helpers ##
#############################

def trans(d):
    """
    Compute pure translation homogenous transformation
    """
    return np.array([
        [ 1, 0, 0, d[0] ],
        [ 0, 1, 0, d[1] ],
        [ 0, 0, 1, d[2] ],
        [ 0, 0, 0, 1    ],
    ])

def roll(a):
    """
    Compute homogenous transformation for rotation around x axis by angle a
    """
    return np.array([
        [ 1,     0,       0,  0 ],
        [ 0, cos(a), -sin(a), 0 ],
        [ 0, sin(a),  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def pitch(a):
    """
    Compute homogenous transformation for rotation around y axis by angle a
    """
    return np.array([
        [ cos(a), 0, -sin(a), 0 ],
        [      0, 1,       0, 0 ],
        [ sin(a), 0,  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def yaw(a):
    """
    Compute homogenous transformation for rotation around z axis by angle a
    """
    return np.array([
        [ cos(a), -sin(a), 0, 0 ],
        [ sin(a),  cos(a), 0, 0 ],
        [      0,       0, 1, 0 ],
        [      0,       0, 0, 1 ],
    ])

def transform(d,rpy):
    """
    Helper function to compute a homogenous transform of a translation by d and
    rotation corresponding to roll-pitch-yaw euler angles
    """
    return trans(d) @ roll(rpy[0]) @ pitch(rpy[1]) @ yaw(rpy[2])

#################
##  IK Targets ##
#################

# TODO: Try testing your own targets!

# Note: below we are using some helper functions which make it easier to generate
# valid transformation matrices from a translation vector and Euler angles, or a
# sequence of successive rotations around z, y, and x. You are free to use these
# to generate your own tests, or directly write out transforms you wish to test.

"""targets = [
    transform( np.array([-.2, -.3, .5]), np.array([0,pi,pi])            ),
    transform( np.array([-.2, .3, .5]),  np.array([pi/6,5/6*pi,7/6*pi]) ),
    transform( np.array([.5, 0, .5]),    np.array([0,pi,pi])            ),
    transform( np.array([.7, 0, .5]),    np.array([0,pi,pi])            ),
    transform( np.array([.2, .6, 0.5]),  np.array([0,pi,pi])            ),
    transform( np.array([.2, .6, 0.5]),  np.array([0,pi,pi-pi/2])       ),
    transform( np.array([.2, -.6, 0.5]), np.array([0,pi-pi/2,pi])       ),
    transform( np.array([.2, -.6, 0.5]), np.array([pi/4,pi-pi/2,pi])    ),
    transform( np.array([.5, 0, 0.2]),   np.array([0,pi-pi/2,pi])       ),
    transform( np.array([.4, 0, 0.2]),   np.array([pi/2,pi-pi/2,pi])    ),
    transform( np.array([.4, 0, 0]),     np.array([pi/2,pi-pi/2,pi])    ),
]"""

def rand_xy():
    x = np.random.rand()
    y = np.random.rand()
    if x<0.5: #make is negative
        return -0.75*y
    else:
        return 0.75*y
def rand_z():
    return 0.75*np.random.rand()

def rand_th():
    x = np.random.rand()
    y = np.random.rand()
    if x<0.5: #make is negative
        return -pi*y
    else:
        return pi*y

targets = [
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            ),
    transform( np.array([rand_xy(), rand_xy(), rand_z()]), np.array([rand_th(),rand_th(),rand_th()])            )
]


####################
## Test Execution ##
####################

np.set_printoptions(suppress=True)

if __name__ == "__main__":

    arm = ArmController()

    time_array = np.zeros(10)
    it_array = np.zeros(10)
    success_array = np.zeros(10)

    # Iterates through the given targets, using your IK solution
    # Try editing the targets list above to do more testing!
    for i, target in enumerate(targets):
        print("Target " + str(i) + " located at:")
        print(target)
        print("Solving... ")
        show_pose(target,"target")

        seed = arm.neutral_position() # use neutral configuration as seed

        start = perf_counter()
        q, success, rollout = ik.inverse(target, seed)
        stop = perf_counter()
        dt = stop - start

        time_array[i-1] = dt
        it_array[i-1] = len(rollout)

        if success:
            print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt,it=len(rollout)))
            success_array[i-1]=1
            arm.safe_move_to_position(q)
        else:
            success_array[i-1]=1
            print('IK Failed for this target using this seed.')


        if i < len(targets) - 1:
            input("Press Enter to move to next target...")

    print("Performance Statistics \n")
    print("Time:")
    print("Mean:", np.mean(time_array))
    print("Median:", np.median(time_array))
    #print("Mode:", np.mode(time_array))
    print("Iterations:")
    print("Mean:", np.mean(it_array))
    print("Median:", np.median(it_array))
    #print("Mode:", np.mode(it_array))
    print("Success Rate:", np.mean(success_array))
