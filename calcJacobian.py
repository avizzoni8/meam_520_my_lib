import numpy as np
from calculateFK import FK

fk = FK()

def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q: 0 x 7 configuration vector (of joint angles) [q0,q1,q2,q3,q4,q5,q6]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    fk = FK()
    J = np.zeros((6, 7))
    joint_positions, T0e = fk.forward(q)
    Ai = fk.compute_Ai(q)
    z_axes = fk.get_axis_of_rotation(q)
    #print("Z axes are \n", z_axes)

    for i in [0,1,2,3,4,5,6]:
        J[0:3,i]=np.cross(z_axes[i,:],joint_positions[-1]-joint_positions[i])
        J[3:6,i]=Ai[i][0:3,0:3] @ [0,0,1]
        #print("J",str(i+1)," Joint location \n", joint_positions[i])
        #print("J",str(i+1)," r vector \n", joint_positions[-1]-joint_positions[i])
        #print("J",str(i+1)," previous z axis \n", z_axes[i,:])
        #print("J",str(i+1)," Linear Velocity \n", J[0:3,i])

        lv = J[0:3,i]=np.cross(z_axes[i,:],joint_positions[-1]-joint_positions[i])
        wv = J[3:6,i]=Ai[i][0:3,0:3] @ [0,0,1]
        """check1 = np.dot(lv,z_axes[i,:])
        check2 = np.dot(lv,wv)
        check3 = np.dot(lv, joint_positions[-1]-joint_positions[i])
        #print([check1,check2,check3])
        #print("Checking dot products...")
        #print("Linear Velocity dotted with Axis of Rot\n", check1)
        #print("Linear Velocity dotted with Angular Velocity\n", check2)
        if check1 < 0.00001 and check2 < 0.00001 and check3 < 0.00001:
            print("Good!")
        else:
            print("Not Good!")"""

    return J

if __name__ == '__main__':
    q=np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print("New configuration is \n", q)
    print(np.round(calcJacobian(q),3))
    q=np.array([np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0, np.pi/2, np.pi/4])
    print("New configuration is \n", q)
    print(np.round(calcJacobian(q),3))
    q=np.array([np.pi/3, 0, -np.pi/4, -np.pi/2, 0, 0, 0])
    print("New configuration is \n", q)
    print(np.round(calcJacobian(q),3))
