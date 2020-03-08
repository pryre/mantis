# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from geometry_msgs.msg import Vector3

class BaseMovement():
	def __init__(self):
		self.position = Vector3()
		self.yaw = float()

	def __repr__(self):
		return "BaseMovement(" + self.__str__() + ")"

	def __str__(self):
		return str(self.position.x) + ";" + str(self.position.y) + ";" + str(self.position.z) + ";" + str(self.yaw)

class Movement():
	def __init__(self):
		self.base = BaseMovement()
		self.joint_positions = []

	def __repr__(self):
		return "Movement(" + self.__str__() + ")\n"

	def __str__(self):
		return str(self.base) + "\n" + str(self.joint_positions)
