### Running Partially

#### Mapping in Real Time

1. In Ubuntu, Click on the Wifi on the top-right and view `Connection Information`.

2. Get the IP address of the Jetson.

3. Open a Terminal in the Jetson.

4. Use command `$ sudo nano .bashrc` to edit the bashrc file.

5. At the very END of the file, add the following lines: <br />
  `export ROS_MASTER_URI=http://(Jetson’s IP Address):11311` <br />
  `export ROS_IP=(Jetson’s IP Address)` <br />

6. Save, exit and close the Terminal.

7. Open a new Terminal. Drag and run the “Bridge.sh” file on the desktop **OR** run the bridge instruction. [Find Details Here](https://github.com/UniKerogen/TempleF1-10/tree/master/Setup/Jetson/Bridges)

8. Use command `$ roscore` to bring ROS online.

9. Open **another** Terminal and use command `$ rosrun urg_node urg_node _ip_address:=192.168.1.11` to start LIDAR streaming.

10. On your laptop in Ubuntu, find its IP Address, then **ssh** into Jetson.

11. Use the following command to connect to Jetson’s ROS Master: <br />
  `$ export ROS_MASTER_URI=http://(Jetson’s IP Address):11311` <br />
  `$ export ROS_IP=(Your Laptop’s IP Address)` <br />
  
12. Use command `$ rosrun rviz rviz -l` to view realtime map.

<p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/RVIZStartup.png?raw=true" width="51%">
</p>
<p align="center">
    Fig.1 RVIZ Start up
</p>
<p align="center">
    <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/RVIZReceivingData.png?raw=true" width="48%">
</p>
<p align="center">
   Fig. 2 RVIZ Receiving Data. Credit to linorobot.org
</p>

#### Mapping Structure

1. On Jetson, drag and run the `Bridge.sh` file from Desktop in a Terminal.

2. Check Jetson’s IP Address. If there is a change, edit the .bashrc file and reopen Terminal.

3. Use command `$ urg_node urg_node _ip_address:=192.168.1.11` to bring LINDAR online.

4. On your laptop in Ubuntu, Use following command to connect to Jetson’s ROS Master: <br />
  `$ export ROS_MASTER_URI=http://(Jetson’s IP Address):11311` <br />
  `$ export ROS_IP=(Your Laptop’s IP Address)` <br />

5. Download the `hector_slam_example` file once given. Put the file inside ~/f1tenth folder. If you do not have the folder, create one.

6. Use command `$ cd f1tenth/hector_slam_example/launch` and `$ roslaunch hector_hokuyo.launch`to launch a pre-built rviz.

7. Take a walk with the Jetson or the part where LINDAR is mounted.

8. The structure map will be generated in rviz, example shown below.

<p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/RVIZMapping.png?raw=true" width="50%"/>
</p>
