from math import *

import roslib
roslib.load_manifest('mantis_planner')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import numpy as np

from mantis_msgs.msg import Movement
from mantis_planner.msg import ManoeuvreAction, ManoeuvreGoal

from breadcrumb.srv import RequestPath
from breadcrumb.srv import RequestPathRequest

class PlannerAStar:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('manoeuvre_astar', ManoeuvreAction, self.execute, False)
		self.server.register_preempt_callback(self.cancel)

		rospy.loginfo("[PlannerA*] Started manoeuvre_astar server")

		manoeuvre_topic = rospy.get_param('~manoeuvre_topic', 'manoeuvre')
		self.client = actionlib.SimpleActionClient(manoeuvre_topic, ManoeuvreAction)
		self.client.wait_for_server()

		rospy.loginfo("[PlannerA*] Connected to guidance router")

		rospy.wait_for_service('~request_path')
		self.srvc_bc = rospy.ServiceProxy('~request_path', RequestPath)

		rospy.loginfo("[PlannerA*] Connected to manoeuvre and astar servers, starting astar planner")
		self.server.start()


	def execute(self, goal):
		try:
			if( self.movement_dispatcher(goal) ):
				rospy.loginfo("Movement plan complete!")
				self.server.set_succeeded()
		except (TypeError, ValueError) as e:
			rospy.logerr(e)
			rospy.loginfo("Movement plan aborted!")
			self.server.set_aborted()

	def cancel(self):
		if self.client is not None:
			self.client.cancel_all_goals()

	def do_dispatch_discrete(self, plan):
		success = True

		for i in range(len(plan.movements) - 1):
			rospy.loginfo("manoeuvre %i:" % i)

			req = RequestPathRequest()
			req.start = plan.movements[i].base.position
			req.end = plan.movements[i+1].base.position

			res = self.srvc_bc(req)

			if len(res.path.poses) > 0:
				rospy.loginfo("[NAV] Path planned, preparing to transmit")

				move_astar = ManoeuvreGoal()
				move_astar.mode = move_astar.MODE_CONTINUOUS
				move_astar.joint_names = plan.joint_names

				#Set up variables for timing and interpolation points
				lt = 0.0
				linspace_yaws = np.linspace(plan.movements[i].base.yaw, plan.movements[i+1].base.yaw, num=len(res.path.poses))
				linspace_joints = [[]]*len(plan.joint_names)
				for j in range(len(plan.joint_names)):
					linspace_joints[j] = np.linspace(plan.movements[i].joints[j], plan.movements[i+1].joints[j], num=len(res.path.poses))

				move_astar.movements = []
				#for j in range(len(res.path_sparse.poses)):
				for j in range(len(res.path.poses)):
					move_astar.movements.append(Movement())
					#move_astar.movements[j].base.position = res.path_sparse.poses[j].position
					move_astar.movements[j].base.position = res.path.poses[j].position
					move_astar.movements[j].base.yaw = linspace_yaws[j]
					#move_astar.movements[j].joints = []*len(
					for k in range(len(plan.joint_names)):
						move_astar.movements[j].joints.append(linspace_joints[k][j])

					if j < (len(res.path.poses) - 1):
						dx = res.path.poses[j+1].position.x - res.path.poses[j].position.x
						dy = res.path.poses[j+1].position.y - res.path.poses[j].position.y
						dz = res.path.poses[j+1].position.z - res.path.poses[j].position.z

						lt += sqrt((dx*dx)+(dy*dy)+(dz*dz)) / plan.nominal_velocity

				rt = 0.0
				if plan.movements[i+1].base.yaw > plan.movements[i].base.yaw:
					yawd = (plan.movements[i+1].base.yaw - plan.movements[i].base.yaw) % pi
					rt = yawd / plan.nominal_rate
				else:
					yawd = (plan.movements[i].base.yaw - plan.movements[i+1].base.yaw) % pi
					rt = yawd / plan.nominal_rate

				jt = 0.0
				for j in range(len(plan.joint_names)):
					rospy.loginfo("\tj%i: %0.4f -> %0.4f" % (j,plan.movements[i].joints[j], plan.movements[i+1].joints[j]))
					jt = max([jt, abs(plan.movements[i+1].joints[j] - plan.movements[i].joints[j] ) / plan.nominal_joint_rate])

				move_time = max([lt,rt,jt])
				move_astar.c_duration = rospy.Duration.from_sec(move_time)

				self.client.send_goal(move_astar)
				self.client.wait_for_result()
				success = (self.client.get_state() == GoalStatus.SUCCEEDED)
			else:
				rospy.logerr("No valid path received from breadcrumb, abandoning planning")

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
			else:
				rospy.logerr("Invalid movement mode set (%s), only discrete plans are supported" % plan.mode)
				raise ValueError("Error: invalid mode")
		else:
			raise ValueError("Error: <2 movements")

		return success
