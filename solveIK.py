import numpy as np
from math import pi, acos
from scipy.linalg import null_space

from calcJacobian import calcJacobian
from calculateFK import FK
from IK_velocity import IK_velocity


class IK:
    # JOINT LIMITS
    lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    center = lower + (upper - lower) / 2  # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self, linear_tol=1e-4, angular_tol=1e-3, max_steps=500, min_step_size=1e-5):
        """
        Constructs an optimization-based IK solver with given solver parameters.
        Default parameters are tuned to reasonable values.

        PARAMETERS:
        linear_tol - the maximum distance in meters between the target end
        effector origin and actual end effector origin for a solution to be
        considered successful
        angular_tol - the maximum angle of rotation in radians between the target
        end effector frame and actual end effector frame for a solution to be
        considered successful
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # solver parameters
        self.linear_tol = linear_tol
        self.angular_tol = angular_tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size

    ######################
    ## Helper Functions ##
    ######################

    @staticmethod
    def displacement_and_axis(target, current):
        """
        Helper function for the End Effector Task. Computes the displacement
        vector and axis of rotation from the current frame to the target frame

        This data can also be interpreted as an end effector velocity which will
        bring the end effector closer to the target position and orientation.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        current - 4x4 numpy array representing the "current" end effector orientation

        OUTPUTS:
        displacement - a 3-element numpy array containing the displacement from
        the current frame to the target frame, expressed in the world frame
        axis - a 3-element numpy array containing the axis of the rotation from
        the current frame to the end effector frame. The magnitude of this vector
        must be sin(angle), where angle is the angle of rotation around this axis
        """

        ## STUDENT CODE STARTS HERE
        displacement = target - current
        displacement = np.transpose(displacement[0:3, 3])
        current_rot = current[0:3, 0:3]
        # print(current_rot)
        target_rot = target[0:3, 0:3]
        # print(target_rot)
        rotation_matrix_current_to_end = np.transpose(current_rot) @ target_rot
        # print(rotation_matrix_current_to_end)
        skew_matrix = 1 / 2 * (rotation_matrix_current_to_end - np.transpose(rotation_matrix_current_to_end))
        # print(skew_matrix)
        axis = np.zeros(3)
        axis[0] = skew_matrix[2, 1]  # correspond to a1
        axis[1] = skew_matrix[0, 2]  # correspond to a2
        axis[2] = skew_matrix[1, 0]  # correspond to a3
        axis = current[0:3,0:3] @ axis

        ## END STUDENT CODE

        return displacement, axis

    @staticmethod
    def distance_and_angle(G, H):
        """
        Helper function which computes the distance and angle between any two
        transforms.

        This data can be used to decide whether two transforms can be
        considered equal within a certain linear and angular tolerance.

        Be careful! Using the axis output of displacement_and_axis to compute
        the angle will result in incorrect results when |angle| > pi/2

        INPUTS:
        G - a 4x4 numpy array representing some homogenous transformation
        H - a 4x4 numpy array representing some homogenous transformation

        OUTPUTS:
        distance - the distance in meters between the origins of G & H
        angle - the angle in radians between the orientations of G & H


        """

        ## STUDENT CODE STARTS HERE
        distance = G - H  # G denoted as target and H is current
        distance = np.linalg.norm(np.transpose(distance[0:3, 3]))
        G_rotation_matrix = G[0:3, 0:3]
        H_rotation_matrix = H[0:3, 0:3]
        rotation_matrix = np.transpose(H_rotation_matrix) @ G_rotation_matrix
        input_value = (np.trace(rotation_matrix) - 1) / 2
        # normalize 3 vector to remove noise
        if abs(input_value) > 1:
            a1 = rotation_matrix[0:3, 0]
            a2 = rotation_matrix[0:3, 1]
            a3 = rotation_matrix[0:3, 2]
            normalized_a1 = a1 / np.sqrt(np.sum(a1 ** 2))
            normalized_a2 = a2 / np.sqrt(np.sum(a2 ** 2))
            normalized_a3 = a3 / np.sqrt(np.sum(a3 ** 2))
            rotation_matrix = np.vstack((normalized_a1, normalized_a2))
            rotation_matrix = np.vstack((rotation_matrix, normalized_a3))
            rotation_matrix = np.transpose(rotation_matrix)

            #print("inside of distance_and_angle, input>1")
            #print(rotation_matrix)
            #print(a1,a2,a3)

        input_value = (np.trace(rotation_matrix) - 1) / 2
        angle = acos(input_value)
        ## END STUDENT CODE

        return distance, angle

    def is_valid_solution(self, q, target):
        """
        Given a candidate solution, determine if it achieves the primary task
        and also respects the joint limits.

        INPUTS
        q - the candidate solution, namely the joint angles
        target - 4x4 numpy array representing the desired transformation from
        end effector to world

        OUTPUTS:
        success - a Boolean which is True if and only if the candidate solution
        produces an end effector pose which is within the given linear and
        angular tolerances of the target pose, and also respects the joint
        limits.
        """

        ## STUDENT CODE STARTS HERE

        success = False
        print("in valid solution")
        fk = FK()
        ik = IK()
        end_pose = fk.forward(q)[1]
        print("end pos is", end_pose)
        error = ik.distance_and_angle(end_pose,target)
        d_er = error[0]
        ang_er = error[1]

        if d_er < ik.linear_tol and ang_er < ik.angular_tol:
            for i in np.arange(len(ik.upper)):
                if q[i] < ik.upper[i] and q[i] > ik.lower[i]:
                    success = True
                else:
                    success = False
                return success

        else:
            success = False

        """fk = FK();
        end_pose = fk.forward(q)[1][0:3,0:3]
        end_loc = fk.forward(q)[1][0:3,3]
        target_pose = target[0:3,0:3]
        target_loc = target[0:3,3]

        if abs(np.linalg.norm(end_loc-target_loc))<= self.linear_tol:
            #Good, check angular tolerence:
            ang_error = abs(end_pose-target_pose)
            for i in [0,1,2]:
                for j in [0,1,2]:
                    if ang_error[i,j]<=self.angular_tol:
                        success = True
                    else:
                        success = False

        else:
            success = False"""


        ## END STUDENT CODE

        return success

    ####################
    ## Task Functions ##
    ####################

    @staticmethod
    def end_effector_task(q, target):
        """
        Primary task for IK solver. Computes a joint velocity which will reduce
        the error between the target end effector pose and the current end
        effector pose (corresponding to configuration q).

        INPUTS:
        q - the current joint configuration, a "best guess" so far for the final answer
        target - a 4x4 numpy array containing the desired end effector pose

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        ## STUDENT CODE STARTS HERE

        ik = IK()
        fk = FK()
        #print("in end effector pose")
        #print("q is", q)
        end_pose = fk.forward(q)[1]
        lin_vel, ang_vel = ik.displacement_and_axis(target,end_pose)

        dq = IK_velocity(q, lin_vel,ang_vel)

        ## END STUDENT CODE

        return dq

    @staticmethod
    def joint_centering_task(q, rate=5e-1):
        """
        Secondary task for IK solver. Computes a joint velocity which will
        reduce the offset between each joint's angle and the center of its range
        of motion. This secondary task acts as a "soft constraint" which
        encourages the solver to choose solutions within the allowed range of
        motion for the joints.

        INPUTS:
        q - the joint angles
        rate - a tunable parameter dictating how quickly to try to center the
        joints. Turning this parameter improves convergence behavior for the
        primary task, but also requires more solver iterations.

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # normalize the offsets of all joints to range from -1 to 1 within the allowed range
        offset = 2 * (q - IK.center) / (IK.upper - IK.lower)
        dq = rate * -offset  # proportional term (implied quadratic cost)

        return dq

    ###############################
    ## Inverse Kinematics Solver ##
    ###############################

    def inverse(self, target, seed):
        """
        Uses gradient descent to solve the full inverse kinematics of the Panda robot.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        seed - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with optimization

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.
        success - True if the IK algorithm successfully found a configuration
        which achieves the target within the given tolerance. Otherwise False
        rollout - a list containing the guess for q at each iteration of the algorithm
        """

        q = seed
        i = 0 #step counter
        rollout = []

        while True:

            rollout.append(q)

            # Primary Task - Achieve End Effector Pose
            dq_ik = self.end_effector_task(q, target)

            # Secondary Task - Center Joints
            dq_center = self.joint_centering_task(q)

            ## STUDENT CODE STARTS HERE

            # Task Prioritization
            dq = np.zeros(7)  # TODO: implement me!

            J = calcJacobian(q)
            a = np.transpose(null_space(J))
            dq_prime = a*np.dot(a,np.transpose(dq_center))/np.dot(a,np.transpose(a))
            dq = dq_ik + dq_prime

            # Termination Conditions
            if np.linalg.norm(dq)<= self.min_step_size or i==self.max_steps:  # TODO: check termination conditions
                break  # exit the while loop if conditions are met!

            ## END STUDENT CODE

            q = q + dq
            q = q[0]
            i=i+1
            rollout = rollout + [q]


        success = self.is_valid_solution(q, target)
        return q, success, rollout


################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True, precision=5)

    ik = IK()
    fk = FK()

    # matches figure in the handout
    seed = np.array([0, 0, 0, -pi / 2, 0, pi / 2, pi / 4])
    print("q is type", type(seed))

    target = np.array([
        [0, -1, 0, 0.3],
        [-1, 0, 0, 0],
        [0, 0, -1, .5],
        [0, 0, 0, 1],
    ])

    print(ik.is_valid_solution(seed, target))

    q, success, rollout = ik.inverse(target, seed)

    for i, q in enumerate(rollout):
        joints, pose = ik.fk.forward(q)
        d, ang = IK.distance_and_angle(target, pose)
        #print('iteration:', i, ' q =', q, ' d={d:3.4f}  ang={ang:3.3f}'.format(d=d, ang=ang))

    print("Success: ", success)
    print("Solution: ", q)
    print("Iterations:", len(rollout))
