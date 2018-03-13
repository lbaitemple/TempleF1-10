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
