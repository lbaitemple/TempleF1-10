### Installing ROS Indigo on the Jetson

**NOTICE 1:** In this instruction, the Jetson TK1 is running Ubuntu 14.04 LTS with an ARM32 core. <br />
**NOTICE 2:** In this instruction, we will be installing ROS Indigo on board.<br />
**NOTICE 3:** Although it is recommended to use a full desktop version of ROS, but we are going for the Bare Bone version, which means we will be adding different packages once we need them in the future. <br />
**NOTICE 4:** You can also find instruction on [ROS Official Website](http://wiki.ros.org/ROS/Installation).

1. In Terminal, use command `$ lsb_release -a` or go to _System Setting > Details_ to find out the Ubuntu that is running on the Jetson board.

2. Go to _System Setting > Software Updates_.

3. In the `Software & Updates` tab, make sure **_ALL_** four repositories are checked — main, universe, restricted, multiverse.

4. Click `close` to execute changes if there is any, otherwise it will just close.

5. Open Terminal.

6. Use command `$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list’` to setup the source list.

7. Use command `$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116` to setup the ROS key for Jetson.

8. Use command `$ sudo apt-get update` to update the package list.

9. Use Command `$ sudo apt-get install ros-indigo-ros-base` to install ROS Indigo Bare Bones.

10. Use command `$ sudo rosdep init` and `$ rosdep update` to initialize rosdep.

11. Use command `$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc` to add future packages to bash session automatically.

12. Use command `$ sudo apt-get install python-rosinstall` to install rosinstall.

**NOTICE 5:** You can install different packages for ROS Indigo by simply using command `$ sudo apt-get install ros-indigo-(package name)` to install different packages. <br />
**NOTICE 6:** You can also search for different packages for ROS Indigo by simply using command `$ apt-cache search ros-indigo`. <br />
**NOTICE 7:** You can simply replace `indigo` to other ROS version name, e.g. `kinetic`, to install other ROS version. <br />
**NOTICE 8:** You should refer to the official instructions if you are installing other verison of ROS.
