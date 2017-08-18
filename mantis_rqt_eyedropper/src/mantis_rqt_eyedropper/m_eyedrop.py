import os
import math
import numpy as np
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsScene
from python_qt_binding.QtGui import QBrush, QColor
from python_qt_binding.QtCore import QObject, pyqtSignal

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class MEyedrop(Plugin):
	def __init__(self, context):
		super(MEyedrop, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('MEyedrop')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		from argparse import ArgumentParser
		parser = ArgumentParser()
		# Add argument(s) to the parser.
		parser.add_argument("-q", "--quiet", action="store_true",
		              dest="quiet",
		              help="Put plugin in silent mode")
		self.args, unknowns = parser.parse_known_args( context.argv() )

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('mantis_rqt_eyedropper'), 'resource', 'MEyedrop.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('MEyedropUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		self._widget.button_refresh_topics.clicked.connect(self.button_refresh_topics_pressed)
		self._widget.button_take_sample.clicked.connect(self.button_take_sample_pressed)

		self._widget.combo_box_topic_image.currentIndexChanged.connect(self.do_subscribe_image)
		self._widget.combo_box_topic_point.currentIndexChanged.connect(self.do_subscribe_point)

		self.colour_pane_scene = QGraphicsScene()
		self.colour_pane_scene.setBackgroundBrush( QBrush( QColor(0, 0, 0) ) )
		self._widget.graphics_view_colour_pane.setScene(self.colour_pane_scene)
		self._widget.graphics_view_colour_pane.show()

		self.bridge = CvBridge()

		self.do_refresh_topic_lists()

	def shutdown_plugin(self):
		try:
			self.image_sub.unregister()
		except:
			pass

		try:
			self.point_sub.unregister()
		except:
			pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def do_get_topic_list(self, msg_type):
		topics = rospy.get_published_topics()

		match_topics = []

		for t in topics:
			if t[1] == msg_type:
				match_topics.append(t[0])

		return match_topics

	def do_refresh_topic_lists(self):
		self._widget.combo_box_topic_image.clear()
		self._widget.combo_box_topic_point.clear()
		self._widget.combo_box_topic_image.addItem('')
		self._widget.combo_box_topic_point.addItem('')

		for it in self.do_get_topic_list('sensor_msgs/Image'):
			self._widget.combo_box_topic_image.addItem(it)

		for pt in self.do_get_topic_list('geometry_msgs/Point'):
			self._widget.combo_box_topic_point.addItem(pt)

	def button_refresh_topics_pressed(self):
		self.do_refresh_topic_lists()

	def button_take_sample_pressed(self):
		self.do_take_sample()

	def do_subscribe_image(self):
		topic_name = self._widget.combo_box_topic_image.currentText()

		rospy.loginfo("image topic: %s" % topic_name)

		if topic_name:
			self.image_sub = rospy.Subscriber(topic_name, Image, self.image_callback)
		else:
			try:
				self.image_sub.unregister()
			except:
				pass

	def do_subscribe_point(self):
		topic_name = self._widget.combo_box_topic_point.currentText()

		rospy.loginfo("point topic: %s" % topic_name)

		if topic_name:
			self.point_sub = rospy.Subscriber(topic_name, Point, self.point_callback)
		else:
			try:
				self.point_sub.unregister()
			except:
				pass

	def do_take_sample(self):
		try:
			px_x = int(self._widget.spinbox_pixel_location_x.value())
			px_y = int(self._widget.spinbox_pixel_location_y.value())

			b = self.cv_image.item(px_y, px_x, 0)
			g = self.cv_image.item(px_y, px_x, 1)
			r = self.cv_image.item(px_y, px_x, 2)

			px_bgr = np.zeros((1, 1, 3), np.uint8)
			px_bgr[:] = tuple((b, g, r))
			px_hsv = cv2.cvtColor(px_bgr, cv2.COLOR_BGR2HSV)

			h = px_hsv.item(0, 0, 0)
			s = px_hsv.item(0, 0, 1)
			v = px_hsv.item(0, 0, 2)

			self._widget.line_edit_pixel_b.setText(str(b))
			self._widget.line_edit_pixel_g.setText(str(g))
			self._widget.line_edit_pixel_r.setText(str(r))

			self._widget.line_edit_pixel_h.setText(str(h))
			self._widget.line_edit_pixel_s.setText(str(s))
			self._widget.line_edit_pixel_v.setText(str(v))

			self.colour_pane_scene.setForegroundBrush( QBrush( QColor(r,g,b) ) )
			self._widget.graphics_view_colour_pane.show()

			if not self.args.quiet:
				rospy.loginfo("----------------")
				rospy.loginfo("XY:  [%d, %d]" % (px_x, px_y))
				rospy.loginfo("RGB: [%d, %d, %d]" % (r, g, b))
				rospy.loginfo("HSV: [%d, %d, %d]" % (h, s, v))

		except Exception as e:
			rospy.loginfo(e)

	def point_callback(self,data):
		self._widget.spinbox_pixel_location_x.setValue(int(data.x))
		self._widget.spinbox_pixel_location_y.setValue(int(data.y))

		self.do_take_sample()

	def image_callback(self,data):
		self._widget.spinbox_pixel_location_x.setMaximum(data.width - 1)
		self._widget.spinbox_pixel_location_y.setMaximum(data.height - 1)

		self._widget.spinbox_pixel_location_x.setEnabled(True)
		self._widget.spinbox_pixel_location_y.setEnabled(True)
		self._widget.button_take_sample.setEnabled(True)

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.loginfo(e)

		if self._widget.check_box_auto_update.isChecked():
			self.do_take_sample()



