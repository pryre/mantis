std_msgs/Header header

# Stamps identify to the current parmeter listing
#
# This allows for some parameters to be updated
# online without causing a race condition for
# other nodes that are configuration-dependant
# A practical application is that it allows
# inertial properties to be changed without requring
# the a new state estimate as the body configuration
# of the MM-UAV haven't actually changed
#
# The configuration stamp identifies the last time
# changes were made that impact the current state or
# physical configuration in some manner.
# (e.g. adding a new body or changing motor mapping)
#
# The parametric stamp identifies the last time
# changes were made that only modfied dynamic or
# limiting parameters.
# (e.g. thrust coefficients or inertial properties)
time configuration_stamp
time parametric_stamp

# Parameter listing
mantis_msgs/Parameters parameters
