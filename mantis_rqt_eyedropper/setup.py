# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mantis_rqt_eyedropper'],
    package_dir={'': 'src'},
    requires=['cv_bridge', 'sensor_msgs', 'geometry_msgs', 'rospy']
)

setup(**setup_args)
