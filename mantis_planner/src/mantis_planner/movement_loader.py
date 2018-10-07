import rospy

from mantis_planner.movement import BaseMovement, Movement
from geometry_msgs.msg import Vector3

def get_joint_names():
	joint_names = []
	count = 0
	while ( rospy.has_param("~movements/joint_names/j%i" % (count) ) ):
		joint_names.append(str(rospy.get_param("~movements/joint_names/j%i" % (count) ) ) )
		count += 1

	return joint_names

def count_joint_params(move):
	count = 0
	while ( rospy.has_param("~movements/m%i/j%i" % (move,count) ) ):
		count += 1

	return count

def check_joint_params(move, num_joints):
	return num_joints == count_joint_params(move)

def check_base_params(move):
	return rospy.has_param("~movements/m%i/base/x" % (move) ) and \
		   rospy.has_param("~movements/m%i/base/y" % (move) ) and \
		   rospy.has_param("~movements/m%i/base/z" % (move) ) and \
		   rospy.has_param("~movements/m%i/base/yaw" % (move) )

def load_movements():
	if( check_base_params(0) ):
		joint_names = get_joint_names()
		num_joints = len(joint_names)
		rospy.loginfo("Detected %i joints" % num_joints)

		i = 0
		movements = []
		while( check_base_params(i) and check_joint_params(i, num_joints) ):
			movements.append(Movement())
			movements[i].base.position.x = rospy.get_param("~movements/m%i/base/x" % (i) )
			movements[i].base.position.y = rospy.get_param("~movements/m%i/base/y" % (i) )
			movements[i].base.position.z = rospy.get_param("~movements/m%i/base/z" % (i) )
			movements[i].base.yaw = rospy.get_param("~movements/m%i/base/yaw" % (i) )

			movements[i].joints = []
			for j in range(num_joints):
				movements[i].joint_positions.append(rospy.get_param( "~movements/m%i/j%i" % (i,j) ) )

			rospy.loginfo(movements[i])

			i += 1

		return (True, movements, joint_names)

	else:
		return (False, None, None)
