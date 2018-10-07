from geometry_msgs.msg import Vector3

class BaseMovement():
	position = Vector3()
	yaw = float()

	def __repr__(self):
		return "BaseMovement(" + self.__str__() + ")"

	def __str__(self):
		return str(self.position.x) + ";" + str(self.position.y) + ";" + str(self.position.z) + ";" + str(self.yaw)

class Movement():
	base = BaseMovement()
	joint_positions = []

	def __repr__(self):
		return "Movement(" + self.__str__() + ")\n"

	def __str__(self):
		return str(self.base) + "\n" + str(self.joint_positions)
