import os
import math
import numpy as np
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QGraphicsScene
from python_qt_binding.QtGui import QBrush
from python_qt_binding.QtGui import QColor

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
		self.args, unknowns = parser.parse_known_args(context.argv())
		if not self.args.quiet:
		    print 'arguments: ', args
		    print 'unknowns: ', unknowns

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

		self.colour_pane_scene = QGraphicsScene()
		self.colour_pane_scene.setBackgroundBrush( QBrush( QColor(0, 0, 0) ) )
		self._widget.graphics_view_colour_pane.setScene(self.colour_pane_scene)
		self._widget.graphics_view_colour_pane.show()

		self.bridge = CvBridge()

		#TODO: Should be dynamic!
		self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
		self.point_sub = rospy.Subscriber("/camera/image_raw_mouse_left", Point, self.point_callback)

	def shutdown_plugin(self):
		self.image_sub.unregister()
		self.point_sub.unregister()

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

	#XXX: rospy.get_published_topics()

	def button_refresh_topics_pressed(self):
		pass

	def button_take_sample_pressed(self):
		self.do_take_sample()

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



