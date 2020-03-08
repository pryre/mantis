# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

sudo systemctl stop serial-getty@ttySAC2.service 
sudo chmod g+rw,o+rw,u+rw /dev/ttySAC2
#roslaunch ~/catkin_ws/launch/px4.launch
