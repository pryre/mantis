sudo systemctl stop serial-getty@ttySAC2.service 
sudo chmod g+rw,o+rw,u+rw /dev/ttySAC2
#roslaunch ~/catkin_ws/launch/px4.launch
