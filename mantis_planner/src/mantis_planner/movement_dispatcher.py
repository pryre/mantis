from math import *

import roslib
roslib.load_manifest('contrail')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from contrail.msg import TrajectoryAction, TrajectoryGoal
from mantis_router_joints.msg import JointMovementAction, JointMovementGoal
from mantis_msgs.msg import Manoeuvre, Movement

def configure_clients(joint_names):
	action_topic_contrail = rospy.get_param("~action_topic_contrail", 'contrail')
	action_topic_joints = rospy.get_param("~action_topic_joint_prefix", "router_joints/")

	client_base = actionlib.SimpleActionClient(action_topic_contrail, TrajectoryAction)
	rospy.loginfo("Waiting for base action client")
	client_base.wait_for_server()

	action_client_joints = []
	for j in range(len(joint_names)):
		if not rospy.is_shutdown():
			action_client_joints.append(actionlib.SimpleActionClient(action_topic_joints + joint_names[j], JointMovementAction))
			rospy.loginfo("Waiting for %s action client" % (action_topic_joints + joint_names[j]))
			action_client_joints[j].wait_for_server()

	return (client_base, action_client_joints)

def do_dispatch_continuous(plan):
	success = True

	(client_base, action_client_joints) = configure_clients(plan.joint_names)

	# Prepare the spline interpolation points
	rospy.loginfo("Continuous manoeuvre:")

	goal_base = TrajectoryGoal()
	goal_base.duration = plan.c_duration
	goal_base.positions = []
	goal_base.yaws = []
	for i in range(len(plan.movements)):
		goal_base.positions.append(plan.movements[i].base.position)
		goal_base.yaws.append(plan.movements[i].base.yaw)

	# Do some formatted logging to console
	datalog_names = ["Base_Px","Base_Py","Base_Pz","Base_Rz"]
	datalog = []
	datalog_px = []
	datalog_py = []
	datalog_pz = []

	for i in range(len(goal_base.positions)):
		datalog_px.append(goal_base.positions[i].x)
		datalog_py.append(goal_base.positions[i].y)
		datalog_pz.append(goal_base.positions[i].z)

	datalog.append(datalog_px)
	datalog.append(datalog_py)
	datalog.append(datalog_pz)
	datalog.append(goal_base.yaws)

	goal_joints = []
	for j in range(len(plan.joint_names)):
		goal_joints.append(JointMovementGoal())
		goal_joints[j].duration = plan.c_duration

		goal_joints[j].positions = []
		for i in range(len(plan.movements)):
			goal_joints[j].positions.append(plan.movements[i].joints[j])

		datalog.append(goal_joints[j].positions)
		datalog_names.append(plan.joint_names[j])

	format_title = ""
	format_row = ""
	for i in range(len(datalog_names)):
		format_title += '%' + str(len(datalog_names[i])) + 's '
		format_row += '%' + str(len(datalog_names[i])) + '.4f '

	rospy.loginfo(format_title % tuple(datalog_names))
	for i in range(len(plan.movements)):
		row = []
		for d in range(len(datalog)):
			row.append(datalog[d][i])
		rospy.loginfo(format_row % tuple(row))

	# Prepare the starting time of the action to syncronize clients
	time_start = rospy.Time.now()
	goal_base.start = time_start + rospy.Duration.from_sec(2)
	for j in range(len(plan.joint_names)):
		goal_joints[j].start = time_start + rospy.Duration.from_sec(2)

	# Send out all of the goals in a batch
	client_base.send_goal(goal_base)
	for j in range(len(plan.joint_names)):
		action_client_joints[j].send_goal(goal_joints[j])

	# Wait for a return from the clients
	client_base.wait_for_result()
	success = (client_base.get_state() == GoalStatus.SUCCEEDED)
	for j in range(len(plan.joint_names)):
		if success:
			action_client_joints[j].wait_for_result()
			success = success and (action_client_joints[j].get_state() == GoalStatus.SUCCEEDED)
		#else:
			# If there is a goal failure, cancel all goals for remaining joints
			#action_client_joints[j].cancel_all_goals()

	if not success:
		rospy.logwarn("Continuous movement externally aborted!")

	return success

def do_dispatch_discrete(plan):
	success = True

	(client_base, action_client_joints) = configure_clients(plan.joint_names)

	nom_lvel = plan.nominal_velocity
	nom_rvel = plan.nominal_rate
	nom_jvel = plan.nominal_joint_rate

	for i in range(len(plan.movements) - 1):
		rospy.loginfo("manoeuvre %i:" % i)

		goal_base = TrajectoryGoal()
		goal_base.positions = [plan.movements[i].base.position, plan.movements[i+1].base.position]
		goal_base.yaws = [plan.movements[i].base.yaw, plan.movements[i+1].base.yaw]

		rospy.loginfo("\tbase: [%0.4f,%0.4f,%0.4f;%0.4f] -> [%0.4f,%0.4f,%0.4f;%0.4f]" %
					   (plan.movements[i].base.position.x, plan.movements[i].base.position.y, plan.movements[i].base.position.z, plan.movements[i].base.yaw,
						plan.movements[i+1].base.position.x, plan.movements[i+1].base.position.y, plan.movements[i+1].base.position.z, plan.movements[i+1].base.yaw))

		dx = plan.movements[i+1].base.position.x - plan.movements[i].base.position.x
		dy = plan.movements[i+1].base.position.y - plan.movements[i].base.position.y
		dz = plan.movements[i+1].base.position.z - plan.movements[i].base.position.z

		lt = sqrt((dx*dx)+(dy*dy)+(dz*dz)) / nom_lvel
		rt = 0.0
		if plan.movements[i+1].base.yaw > plan.movements[i].base.yaw:
			yawd = (plan.movements[i+1].base.yaw - plan.movements[i].base.yaw) % pi
			rt = yawd / nom_rvel
		else:
			yawd = (plan.movements[i].base.yaw - plan.movements[i+1].base.yaw) % pi
			rt = yawd / nom_rvel

		jt = 0
		for j in range(len(plan.joint_names)):
			rospy.loginfo("\tj%i: %0.4f -> %0.4f" % (j,plan.movements[i].joints[j], plan.movements[i+1].joints[j]))
			jt = max([jt, abs(plan.movements[i+1].joints[j] - plan.movements[i].joints[j] ) / nom_jvel])

		move_time = max([lt,rt,jt])
		goal_base.duration = rospy.Duration.from_sec(move_time)
		rospy.loginfo("\ttime: %0.4f" % (move_time))

		goal_joints = []
		for j in range(len(plan.joint_names)):
			goal_joints.append(JointMovementGoal())
			goal_joints[j].positions = [plan.movements[i].joints[j], plan.movements[i+1].joints[j]]
			goal_joints[j].duration = rospy.Duration.from_sec(move_time)

		time_start = rospy.Time.now()
		goal_base.start = time_start + rospy.Duration.from_sec(2)
		for j in range(len(plan.joint_names)):
			goal_joints[j].start = time_start + rospy.Duration.from_sec(2)

		client_base.send_goal(goal_base)
		for j in range(len(plan.joint_names)):
			action_client_joints[j].send_goal(goal_joints[j])

		client_base.wait_for_result()
		success = (client_base.get_state() == GoalStatus.SUCCEEDED)
		for j in range(len(plan.joint_names)):
			if success:
				action_client_joints[j].wait_for_result()
				success = success and (action_client_joints[j].get_state() == GoalStatus.SUCCEEDED)
			#else:
				# If there is a goal failure, cancel all goals for remaining joints
				#action_client_joints[j].cancel_all_goals()

		if not success:
			rospy.logwarn("Discrete movement externally aborted!")
			break

	return success

def movement_dispatcher(plan):
	success = False

	if not isinstance(plan, Manoeuvre):
		raise TypeError("Error: plan variable not mantis_msgs/Manoeuvre")

	if len(plan.movements) >= 2:
		rospy.loginfo("Loaded %i movements for the base and %i joints" % (len(plan.movements), len(plan.joint_names)))

		if plan.mode == Manoeuvre.MODE_DISCRETE:
			if (plan.nominal_velocity > 0) and (plan.nominal_rate > 0) and (plan.nominal_joint_rate > 0):
				rospy.loginfo("Dispatcher seting up for discrete plan")
				success = do_dispatch_discrete(plan)
			else:
				raise ValueError("Error: one or more nominal rates are invalid (<=0)")
		elif plan.mode == Manoeuvre.MODE_CONTINUOUS:
			if (plan.c_duration > rospy.Duration(0)):
				rospy.loginfo("Dispatcher seting up for continuous plan")
				success = do_dispatch_continuous(plan)
			else:
				raise ValueError("Error: duration is invalid (<=0)")
		else:
			rospy.logerr("Invalid movement mode set (%s)" % plan.mode)
			raise ValueError("Error: invalid mode")
	else:
		raise ValueError("Error: <2 movements")

	return success
