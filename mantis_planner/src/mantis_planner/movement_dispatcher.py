from math import *

import roslib
roslib.load_manifest('mantis_planner')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from contrail.msg import TrajectoryAction, TrajectoryGoal
from mantis_router_joints.msg import JointMovementAction, JointMovementGoal
from mantis_planner.msg import ManoeuvreAction, ManoeuvreGoal

class DispatcherFull:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('manoeuvre', ManoeuvreAction, self.execute, False)
		self.server.register_preempt_callback(self.cancel)
		self.server.start()

	def execute(self, goal):
		self.configure_clients(goal.joint_names)

		try:
			if( self.movement_dispatcher(goal) ):
				rospy.loginfo("Movement plan complete!")
				self.server.set_succeeded()
		except (TypeError, ValueError) as e:
			rospy.logerr(e)
			rospy.loginfo("Movement plan aborted!")
			self.server.set_aborted()

	def cancel(self):
		if self.client_base is not None:
			self.client_base.cancel_all_goals()

		for j in self.clients_joints:
			j.cancel_all_goals()

	def configure_clients(self, joint_names):
		action_topic_contrail = rospy.get_param("~action_topic_contrail", 'contrail')
		action_topic_joints = rospy.get_param("~action_topic_joint_prefix", "router_joints/")

		self.client_base = actionlib.SimpleActionClient(action_topic_contrail, TrajectoryAction)
		rospy.loginfo("Waiting for base action client")
		self.client_base.wait_for_server()

		self.clients_joints = []
		for j in range(len(joint_names)):
			if not rospy.is_shutdown():
				self.clients_joints.append(actionlib.SimpleActionClient(action_topic_joints + joint_names[j], JointMovementAction))
				rospy.loginfo("Waiting for %s action client" % (action_topic_joints + joint_names[j]))
				self.clients_joints[j].wait_for_server()

	def do_dispatch_continuous(self, plan):
		success = True

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
		self.client_base.send_goal(goal_base)
		for j in range(len(plan.joint_names)):
			self.clients_joints[j].send_goal(goal_joints[j])

		# Wait for a return from the clients
		self.client_base.wait_for_result()
		success = (self.client_base.get_state() == GoalStatus.SUCCEEDED)
		for j in range(len(plan.joint_names)):
			if success:
				self.clients_joints[j].wait_for_result()
				success = success and (self.clients_joints[j].get_state() == GoalStatus.SUCCEEDED)
			#else:
				# If there is a goal failure, cancel all goals for remaining joints
				#clients_joints[j].cancel_all_goals()

		if not success:
			rospy.logwarn("Continuous movement externally aborted!")

		return success

	def do_dispatch_discrete(self, plan):
		success = True

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

			self.client_base.send_goal(goal_base)
			for j in range(len(plan.joint_names)):
				self.clients_joints[j].send_goal(goal_joints[j])

			self.client_base.wait_for_result()
			success = (self.client_base.get_state() == GoalStatus.SUCCEEDED)
			for j in range(len(plan.joint_names)):
				if success:
					self.clients_joints[j].wait_for_result()
					success = success and (self.clients_joints[j].get_state() == GoalStatus.SUCCEEDED)
				#else:
					# If there is a goal failure, cancel all goals for remaining joints
					#clients_joints[j].cancel_all_goals()

			if not success:
				rospy.logwarn("Discrete movement externally aborted!")
				break

		return success

	def movement_dispatcher(self, plan):
		success = False

		if not isinstance(plan, ManoeuvreGoal):
			raise TypeError("Error: plan variable not mantis_planner/Manoeuvre action")

		if len(plan.movements) >= 2:
			rospy.loginfo("Loaded %i movements for the base and %i joints" % (len(plan.movements), len(plan.joint_names)))

			if plan.mode == ManoeuvreGoal.MODE_DISCRETE:
				if (plan.nominal_velocity > 0) and (plan.nominal_rate > 0) and (plan.nominal_joint_rate > 0):
					rospy.loginfo("Dispatcher seting up for discrete plan")
					success = self.do_dispatch_discrete(plan)
				else:
					raise ValueError("Error: one or more nominal rates are invalid (<=0)")
			elif plan.mode == ManoeuvreGoal.MODE_CONTINUOUS:
				if (plan.c_duration > rospy.Duration(0)):
					rospy.loginfo("Dispatcher seting up for continuous plan")
					success = self.do_dispatch_continuous(plan)
				else:
					raise ValueError("Error: duration is invalid (<=0)")
			else:
				rospy.logerr("Invalid movement mode set (%s)" % plan.mode)
				raise ValueError("Error: invalid mode")
		else:
			raise ValueError("Error: <2 movements")

		return success
