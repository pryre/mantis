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
