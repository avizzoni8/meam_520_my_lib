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
from lib.calculateFK import FK
from lib.calcJacobian import FK
from lib.solveIK import IK
#from rrt import rrt


#TODO make sure we can do this for blue AND red - its just flipping grab and drop


grabpose_3D = np.array([
		[1, 0, 0, 0.562],
		[0, -1, 0, 0.169],
		[0, 0, -1, 0.3],
		[0, 0, 0, 1],
	])

droppose_3D = np.array([
		[0, 1, 0, 0.562],
		[0, 0, 1, -0.169],
		[1, 0, 0, 0.45], #Starts at 0.2+0.005*X
		[0, 0, 0, 1],
	])


def get_robo_frame(pose0,tag):
	#pose0 = detector.get_detections()[0][1]
	h = pose0@transform([0.5, 0, 0], [0, 0, 0])
	h = h@tag
	return h

def stack(i,cur_q):
	droppose_3D = np.array([
		[0, 1, 0, 0.562],
		[0, 0, 1, -0.169],
		[1, 0, 0, 0.230+0.05*i], #Starts at 0.2+0.005*X
		[0, 0, 0, 1],
	])
	release = np.array([
		[0, 1, 0, 0.562],
		[0, 0, 1, -0.169],
		[1, 0, 0, 0.230+0.05*(i+1)],
		[0, 0, 0, 1],
	])

	q = ik.inverse(droppose_3D, cur_q)[0]
	qr = ik.inverse(release, q)[0]
	arm.safe_move_to_position(q)
	arm.exec_gripper_cmd(0.1)
	arm.safe_move_to_position(qr)

def stack_badangle(i,cur_q):
	droppose_3D = np.array([
		[0, 1, 0, 0.562],
		[0, 0, -1, -0.169],
		[-1, 0, 0, 0.230+0.05*i], #Starts at 0.2+0.005*X
		[0, 0, 0, 1],
	])
	release = np.array([
		[0, 1, 0, 0.562],
		[0, 0, -1, -0.169],
		[-1, 0, 0, 0.230 + 0.05 *(i+1)],  # Starts at 0.2+0.005*X
		[0, 0, 0, 1],
	])

	if i == 0:
		droppose_3D = droppose_3D@transform([0,0,0],[0,0,np.pi/2])

	q = ik.inverse(droppose_3D, cur_q)[0]
	qr = ik.inverse(release, q)[0]
	arm.safe_move_to_position(q)
	arm.exec_gripper_cmd(0.1)
	arm.safe_move_to_position(qr)

def stack_6up(i,cur_q):
	droppose_3D = np.array([
		[1, 0, 0, 0.562],
		[0, -1, 0, -0.169],
		[0, 0, -1, 0.230+0.05*i], #Starts at 0.2+0.005*X
		[0, 0, 0, 1],
	])
	release = np.array([
		[1, 0, 0, 0.562],
		[0, -1, 0, -0.169],
		[0, 0, -1, 0.230 + 0.05 * (i+1)],  # Starts at 0.2+0.005*X
		[0, 0, 0, 1],
	])

	q = ik.inverse(droppose_3D, cur_q)[0]
	qr = ik.inverse(release, q)[0]
	arm.safe_move_to_position(q)
	arm.exec_gripper_cmd(0.1)
	arm.safe_move_to_position(qr)

def tag5_function(i,bh):
	#recreate hover location in 3D then rotate
	'''	tag_rf = get_robo_frame(pose0,staticblocks[i][1])
	tag_rf = tag_rf@transform([0,0,0],[0,np.pi,0])
	tag_rf = tag_rf @ transform([0, 0, 0], [0, 0, np.pi/2])
	original_hover = tag_rf@transform([0,0,-0.025],[0,0,0])'''

	#come in at an angle
	hover_rotated_1 = bh @ transform([0, 0, 0], [0, -np.pi/4, 0])
	hover_rotated_Q_1 = ik.inverse(hover_rotated_1, block_hover[i])[0]

	#now move down
	grab_rotated = bh @transform([0,0,0.05],[0,0,0])
	grab_rotated = grab_rotated @ transform([0, 0, 0], [0, -np.pi/4, 0])
	grab_rotated_Q = ik.inverse(grab_rotated, hover_rotated_Q_1)[0]

	arm.safe_move_to_position(hover_rotated_Q_1)
	go_grab(grab_rotated_Q)
	#now grabbed at 45 degree angle

	#now move up - still at an angle:
	hover_rotated_2 = bh @ transform([0, 0, -0.025], [0, 0, 0])
	hover_rotated_2 = hover_rotated_2 @ transform([0, 0, 0], [0, -np.pi / 4, 0])
	hover_rotated_Q_2 = ik.inverse(hover_rotated_2, block_hover[i])[0]
	arm.safe_move_to_position(hover_rotated_Q_2)

	#now rotate forward to show Tag6
	hover_rotated_3 = bh @ transform([0, 0, -0.025], [0, 0, 0])
	hover_rotated_3 = hover_rotated_3 @ transform([0, 0, 0], [0, np.pi/4, 0])
	hover_rotated_Q_3 = ik.inverse(hover_rotated_3, hover_rotated_Q_2)[0]
	arm.safe_move_to_position(hover_rotated_Q_3)

	#now move down and drop
	drop_rotated = bh @ transform([0, 0, 0.045], [0, 0, 0])
	drop_rotated = drop_rotated @ transform([0, 0, 0], [0, np.pi / 4, 0])
	drop_rotated_Q = ik.inverse(drop_rotated, grab_rotated_Q)[0]
	arm.safe_move_to_position(drop_rotated_Q)
	arm.exec_gripper_cmd(0.1)

def reset():
	arm.safe_move_to_position(arm.neutral_position())
	arm.exec_gripper_cmd(0.08)

def go_grab(q):
	arm.safe_move_to_position(q)
	arm.exec_gripper_cmd(0.045,10)

def static_tags():
	tags = []
	for (name, pose) in detector.get_detections():
		if name != 'tag0':
			if name != 'tag7':
				if name != 'tag8':
					if name != 'tag9':
						if name != 'tag10':
							if name != 'tag11':
								if name != 'tag12':
									tags += [(name, pose)]
	return tags


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



	ik = IK()
	fk = FK()

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

	#Get static blocks
	staticblocks = static_tags()

	#Find tag0
	for (name, pose) in detector.get_detections():
		if name == 'tag0':
			print('found tag0')
			pose0 = pose

	#PREPROCESSING
	block_grab = []
	block_hover =[]
	block_hover_3D = []
	block_grab_3D = []
	case =[]
	for i in [0,1,2,3]:
		(name, pose) = staticblocks[i]
		print(name, "\n", pose)
		tag_rf = get_robo_frame(pose0,pose)
		#print('robo frame \n', tag_rf)
		tag_rf = tag_rf@transform([0,0,0],[0,np.pi,0])
		tag_rf = tag_rf @ transform([0, 0, 0], [0, 0, np.pi/2])
		#print("point z down \n", tag_rf)
		tag_rf = tag_rf@transform([0,0,-0.025],[0,0,0])
		#print("hover \n", tag_rf)

		if abs(np.arccos(tag_rf[0, 0])) > 2:
			print("preprocessing - T6 pointed at robot ")
			print("hover pose is\n", tag_rf)
			case += ['badangle']
			tag_rf = tag_rf @ transform([0, 0, 0], [0, 0, np.pi])
		else:
			case += [None]

		block_hover_3D += [tag_rf]
		#turn it into Q space
		block_hover += [ik.inverse(tag_rf, grabpose)[0]]

		#make grab pose
		tag_rf = tag_rf@transform([0,0,0.05],[0,0,0])
		block_grab_3D += [tag_rf]
		print("grab pose \n", tag_rf)
		#Turn it into Q space
		block_grab += [ik.inverse(tag_rf,block_hover[i])[0]]



	# Master Loop

	#Create
	reset()
	for i in [0,1,2,3]:
		(name, pose) = staticblocks[i]

		#if name == 'tag5':
		tag5_function(i,block_hover_3D[i])
		print("tag6 should be pointing at robot now")
		print("go to drop")
		stack_badangle(i, arm.neutral_position())  # will stack block
		arm.safe_move_to_position(neutral)
		continue


		print("go get block")
		arm.safe_move_to_position(block_hover[i])
		go_grab(block_grab[i])
		print("neutral")
		arm.safe_move_to_position(neutral)

		if name == 'tag6':
			print("tag 6 up")
			print("go to drop")
			stack_6up(i, arm.neutral_position())  # will stack block
			arm.safe_move_to_position(neutral)
			continue

		elif case[i] == 'badangle':
			print("Tag 6 pointed at robot")
			print("go to drop")
			stack_badangle(i, arm.neutral_position())  # will stack block
			arm.safe_move_to_position(neutral)
			continue

		else:
			print("go to drop")
			stack(i,arm.neutral_position()) #will stack block
			arm.safe_move_to_position(neutral)

	"""Dynamic Loop?"""

	# END STUDENT CODE

	# END STUDENT CODE

	#test
