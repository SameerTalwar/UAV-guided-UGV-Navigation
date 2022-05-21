# **Submission for DRDO's UAV Guided UGV challenge at InterIIT Tech Meet 10.0** #

## **Installation Instructions** ##

### Installing Ardupilot and MAVProxy In home directory (both git clone and update will take time): ###
```
cd ~/ 
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git 
cd ardupilot 
git checkout Copter-3.6
git submodule update --init --recursive
```
### Install Dependencies ###
```
sudo apt-get update -y
sudo apt-get install -y ros-melodic-mavros geographiclib-tools python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect 
sudo geographiclib-get-geoids egm96-5 
sudo pip install future pymavlink MAVProxy
```

### Install Gazebo plugin for APM (ArduPilot Master) : ### 
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
```

### Build and install plugin ###
```
mkdir build
cd build
cmake .. make -j4 
sudo make install
```
### Install the package ###
```
cd ~/catkin_ws/src/
git clone https://github.com/Roboaries/MP_DR_T8 
catkin_make
```
#### Or, download the repository as zip and extract in your "catkin_ws/src" folder ####

### Add to .bashrc or .zshrc and reload (**This step is very important to ensure that gazebo finds the models and your terminal finds the sim_vehicle.py script**) ###
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest:/usr/lib/ccache
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$HOME/ardupilot_gazebo/worlds 
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/ardupilot_gazebo/models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/MP_DR_T8-main/MP_DR_T8-main/interiit22/interiit22/models
```

## Launch the simulation for world 1 (for running world 2 , change the launch file name to "drdo_world2.launch") ##
```
roslaunch interiit22 drdo_world1.launch 
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console 
```
## After takeoff, run the scripts ##
```
rosrun interiit22 ugv_nav.py 
rosrun interiit22 drone_nav.py
```

# NOTE: #

- The line ```contours,hierarchy=cv2.findContours()``` may give an error: _too many values to unpack_, because the function ```cv2.findContours()``` returns 2 or 3 values depending on the python version. 
If the script gives an error , just add an extra argument:
``` _,contours,hierarchy=cv2.findContours()```
- The spawn location of the UAV has been changed slightly, (within 5 meters of the UGV), to aid in takeoff.
- The range of the depth camera has been increased, but the FOV has not been changed (in accordance with the rules).
- Some paths/URIs in the .launch and the .world files may need to be changed according to one's own directory structure.
