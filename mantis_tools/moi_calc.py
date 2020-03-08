# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

#!/usr/bin/env python

import sys

def format_num(num):
	if isinstance(num, (int, long, float, complex)):
		return str("%.6g" % num)
	else:
		return str(num)

def get_max_width(table, index):
    return max([len(format_num(row[index])) for row in table])

def pprint_table(out, table):
    col_paddings = []

    for i in range(len(table[0])):
        col_paddings.append(get_max_width(table, i))

    for row in table:
		# left col
		print >> out, row[0].center(col_paddings[0] + 2),

		# rest of the cols
		for i in range(1, len(row)):
			col = format_num(row[i]).rjust(col_paddings[i] + 2)
			print >> out, col,

		print >> out

def calc_cuboid(m):
	x = float(raw_input("Enter X dimension (m): "))
	y = float(raw_input("Enter Y dimension (m): "))
	z = float(raw_input("Enter Z dimension (m): "))

	ixx = (m/12)*((y*y) + (z*z))
	iyy = (m/12)*((x*x) + (z*z))
	izz = (m/12)*((x*x) + (y*y))

	return (ixx, iyy, izz)

def calc_sphere(m):
	r = float(raw_input("Enter radius dimension (m): "))

	ixyz = (2*m/5)*(r*r)

	return (ixyz, ixyz, ixyz)

def calc_sphere_h(m):
	r = float(raw_input("Enter radius dimension (m): "))

	ixyz = (2*m/3)*(r*r)

	return (ixyz, ixyz, ixyz)

def calc_cylinder(m):
	r = float(raw_input("Enter radius dimension (m): "))
	h = float(raw_input("Enter height dimension (m): "))

	ixx = (m/12)*(3*(r*r) + (h*h))
	iyy = (m/12)*(3*(r*r) + (h*h))
	izz = (m/2)*(r*r)

	return (ixx, iyy, izz)

def calc_cylinder_h(m):
	r1 = float(raw_input("Enter inner radius dimension (m): "))
	r2 = float(raw_input("Enter outer radius dimension (m): "))
	h = float(raw_input("Enter height dimension (m): "))

	ixx = (m/12)*(3*((r1*r1)+(r2*r2)) + (h*h))
	iyy = (m/12)*(3*((r1*r1)+(r2*r2)) + (h*h))
	izz = (m/2)*((r1*r1) + (r2*r2))

	return (ixx, iyy, izz)

def calc_ellipsoid(m):
	x = float(raw_input("Enter X dimension (m): "))
	y = float(raw_input("Enter Y dimension (m): "))
	z = float(raw_input("Enter Z dimension (m): "))

	ixx = (m/5)*((y*y) + (z*z))
	iyy = (m/5)*((x*x) + (z*z))
	izz = (m/5)*((x*x) + (y*y))

	return (ixx, iyy, izz)

if __name__ == "__main__":
	inertial_tensors = {"cuboid": calc_cuboid,
						"sphere": calc_sphere,
						"hollow sphere": calc_sphere_h,
						"cylinder": calc_cylinder,
						"hollow cylinder": calc_cylinder_h,
						"ellipsoid": calc_ellipsoid}

	tensor = raw_input("Select tensor (or \"list\" for options): ")

	if (tensor == "list") or not (tensor in inertial_tensors):
		print("Please select a tensor to calculate from this list:")

		for key, value in inertial_tensors.iteritems():
			print("\t- " + key)

		exit(0)

	mass = float(raw_input("Object mass (kg): "))

	(ixx, iyy, izz) = inertial_tensors[tensor](mass)

	table = [["I", "x", "y", "z"],
			["x", ixx, 0, 0],
			["y", 0, iyy, 0],
			["z", 0, 0, izz]]

	out = sys.stdout
	pprint_table(out, table)
