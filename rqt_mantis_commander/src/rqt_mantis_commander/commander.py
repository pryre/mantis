import os
import math
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from mantis_msgs.msg import State, Parameters
from geometry_msgs.msg import Pose, Vector3, Quaternion

from std_srvs.srv import SetBool

import actionlib
from contrail.msg import TrajectoryAction, TrajectoryGoal
from mantis_router_joints.msg import JointMovementAction, JointMovementGoal

class MantisCommander(Plugin):
	def __init__(self, context):
		super(MantisCommander, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('MantisCommander')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		#from argparse import ArgumentParser
		#parser = ArgumentParser()
		# Add argument(s) to the parser.
		#parser.add_argument("-q", "--quiet", action="store_true",
		#              dest="quiet",
		#              help="Put plugin in silent mode")
		#args, unknowns = parser.parse_known_args(context.argv())
		#if not args.quiet:
		#    print 'arguments: ', args
		#    print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_mantis_commander'), 'resource', 'MantisCommander.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('MantisCommanderUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.ns
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		self._widget.button_update_namespaces.clicked.connect(self.button_update_namespaces_pressed)
		self._widget.button_arm.clicked.connect(self.button_arm_pressed)
		self._widget.button_disarm.clicked.connect(self.button_disarm_pressed)
		self._widget.button_send_command.clicked.connect(self.button_send_command_pressed)
		self._widget.combo_joints.currentIndexChanged.connect(self.combo_joints_pressed)
		self._widget.textbox_goto_joint.textChanged.connect(self.goto_joint_changed)

		self.sub_state = None
		self.has_state = False
		self.current_pose = None
		self.current_joints = None
		self.joint_sp = []
		self.client_joints = []

		self.update_namespaces()

	def shutdown_plugin(self):
		if self.sub_state is not None:
			self.sub_state.unregister()

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value('namespace', self.ns)
		instance_settings.set_value('contrail_prefix', self.contrail_prefix)
		instance_settings.set_value('joint_prefix', self.joint_prefix)

	def restore_settings(self, plugin_settings, instance_settings):
		#self.ns = instance_settings.value('namespace')
		#self.contrail_prefix = instance_settings.value('contrail_prefix')
		#self.joint_prefix = instance_settings.value('joint_prefix')
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def quaternion_to_rpy(self,q):
		if not isinstance(q, Quaternion):
			raise TypeError('Input should be a geometry_msgs/Quaternion')

		q2sqr = q.y * q.y;
		t0 = -2.0 * (q2sqr + q.z * q.z) + 1.0
		t1 = 2.0 * (q.x * q.y + q.w * q.z)
		t2 = -2.0 * (q.x * q.z - q.w * q.y)
		t3 = 2.0 * (q.y * q.z + q.w * q.x)
		t4 = -2.0 * (q.x * q.x + q2sqr) + 1.0

		if(t2 > 1.0):
			t2 = 1.0
		elif(t2 < -1.0):
			t2 = -1.0

		e = Vector3()
		roll = math.atan2(t3, t4);
		pitch = math.asin(t2);
		yaw = math.atan2(t1, t0);

		return (roll,pitch,yaw)

	def goto_joint_changed(self):
		try:
			if(len(self.joint_sp) > 0):
				val = float(self._widget.textbox_goto_joint.text())
				idx = self._widget.combo_joints.currentIndex()
				self.joint_sp[idx] = val
		except:
			rospy.logerr("Unexpected error!")

	def combo_joints_pressed(self):
		if(len(self.joint_sp) > 0):
			idx = self._widget.combo_joints.currentIndex()
			self._widget.textbox_goto_joint.setText(str(self.joint_sp[idx]))

	def button_send_command_pressed(self):
		rospy.logdebug("Send command pressed!")

		if self.client_base.wait_for_server(rospy.Duration(0.5)):
			try:
				if self.has_state:
					dur = rospy.Duration(float(self._widget.textbox_goto_duration.text()))

					pos_x = float(self._widget.textbox_goto_pos_x.text())
					pos_y = float(self._widget.textbox_goto_pos_y.text())
					pos_z = float(self._widget.textbox_goto_pos_z.text())
					yaw = float(self._widget.textbox_goto_yaw.text())
					roll_c, pitch_c, yaw_c = self.quaternion_to_rpy(self.current_pose.orientation)

					goal_base = TrajectoryGoal()
					goal_base.duration = dur
					goal_base.positions = [Vector3(self.current_pose.position.x,self.current_pose.position.y,self.current_pose.position.z),
										   Vector3(pos_x,pos_y,pos_z)]
					goal_base.yaws = [yaw_c, yaw]

					goal_joints = []
					for i in range(len(self.joint_sp)):
						goal_joints.append(JointMovementGoal())
						goal_joints[i].duration = dur
						goal_joints[i].positions = [self.current_joints[i], self.joint_sp[i]]

					timestamp = rospy.Time.now() + rospy.Duration.from_sec(0.5)

					goal_base.start = timestamp
					for i in range(len(self.joint_sp)):
						goal_joints[i].start = timestamp

					self.client_base.send_goal(goal_base)
					for i in range(len(self.joint_sp)):
						self.client_joints[i].send_goal(goal_joints[i])

					rospy.loginfo("Sending command...")
				else:
					rospy.logerr("No state information received, can't generate command action!")
			except:
				rospy.logerr("Unexpected error!")
		else:
			rospy.logerr("Not connected to action server!")


	def button_update_namespaces_pressed(self):
		self.update_namespaces()
		rospy.logdebug("Update namespaces button pressed!")

	def callback_state(self, msg_in):
		self.has_state = True
		self.current_pose = msg_in.pose
		self.current_joints = msg_in.r

	def callback_params(self, msg_in):
		self.joint_names = []
		self.client_joints = []
		self.joint_sp = []

		for j in range(len(msg_in.joints)):
			if msg_in.joints[j].type != "static":
				self._widget.combo_joints.addItem(msg_in.joints[j].name)
				self.client_joints.append(actionlib.SimpleActionClient(self.ns + '/' + self.joint_prefix + '/' + msg_in.joints[j].name, JointMovementAction) )
				self.joint_sp.append(0.0)

	def update_namespaces(self):
		self.ns = self._widget.textbox_namespace.text()
		self.contrail_prefix = self._widget.textbox_contrail.text()
		self.joint_prefix = self._widget.textbox_joint_prefix.text()

		self.sub_state = rospy.Subscriber(self.ns + "/state", State, self.callback_state)
		self.sub_params = rospy.Subscriber(self.ns + "/params", Parameters, self.callback_params)

		self.client_base = actionlib.SimpleActionClient(self.ns + '/' + self.contrail_prefix, TrajectoryAction)

	def button_arm_pressed(self):
		rospy.logdebug("Button arm pressed!")

		try:
			arm_jc = rospy.ServiceProxy('/mantis_uav/dynamixel_interface/enable_torque_all', SetBool)
			res = arm_jc(True)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
	def button_disarm_pressed(self):
		rospy.logdebug("Button disarm pressed!")

		try:
			arm_jc = rospy.ServiceProxy('/mantis_uav/dynamixel_interface/enable_torque_all', SetBool)
			res = arm_jc(False)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
