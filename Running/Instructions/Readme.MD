## Running Partially

#### Mapping in Real Time

1. In Ubuntu, Click on the Wifi on the top-right and view `Connection Information`.

2. Get the IP address of the Jetson.

3. Open a Terminal in the Jetson.

4. Use command `$ sudo nano .bashrc` to edit the bashrc file.

5. At the very END of the file, add the following lines: <br />
  `export ROS_MASTER_URI=http://(Jetson’s IP Address):11311` <br />
  `export ROS_IP=(Jetson’s IP Address)` <br />

6. Save, exit and close the Terminal.

7. Open a new Terminal. Drag and run the “Bridge.sh” file on the desktop **OR** run the bridge instruction. [Find Details Here](https://github.com/UniKerogen/TempleF1-10/tree/master/Setup/Jetson/Bridges#adding-a-bridge-on-the-jetson-board)

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
   Fig.2 RVIZ Receiving Data. Credit to linorobot.org
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

#### Drive with Keyboard

1. On Jetson, use command `$ mkdir -p catkin_ws_race/src` in a new Terminal to create file directory for race topic.

2. Use command `$ cd catkin_ws_race` to go inside the folder.

3. Copy and paste the race folder from _f1_10_code_public/week_2/_ to _catkin_ws_race/src_.

4. After finished the copying process, use command `$ catkin_make` to create the race topic.

5. Copy and paste `keyboard.py` and `talker.py` under _f1_10_code_public/week_2/race/src_ into _catkin_ws_race/install/share/race_.

6. Right click on those two files and select _Properties > Permissions_. Make these two files `allow executing file as program`.

7. Open a new Terminal and use command `$ sudo nano .bashrc` to edit the bashrc file.

8. At the END of the file, add the following line: <br />
  `source ~/catkin_ws_race/install/setup.bash`

9. Check if the Jetson’s IP Address has changed. If it changes, change the corresponding line in the bashrc file.

10. Save and exit the bashrc file. Close **ALL** Terminal.

11. Open a Terminal, use command `$ roscore` to bring ROS online.

12. On your laptop, open **three** Terminals and ssh all of them into Jetson.

13. In Terminal 1, use command `$ rosrun race talker.py` to initialize talker.

14. In Terminal 2, use command `$ rosrun rosserial_python serial_node.py /dev/ttyACM0` to initialize the connection between the Teensy and the Jetson.

15. In Terminal 3, use command `$ rosrun race keyboard.py` to initialize keyboard control for the car.

16. Now the car should be respond to the keyboard commands. It should start moving at up/down 5ish and left/right depending on the visibility of the input angle.

17. Once finish, press `q` in Terminal 3 and `Control` + `C` in Terminal 1 & 2 to quit.

## Automatic Mode

**NOTICE I:** Make sure to try to drive with keyboard first before continue.<br />
**NOTICE II:** You have to complete the code in python file `control.py` and `dist_finder.py`. <br />
**NOTICE III:** After moving those three files mentioned below, there should be five python files in the directory, two of them should be moved in the pervious [Drive with Keyboard](https://github.com/UniKerogen/TempleF1-10/tree/master/Running/Instructions#drive-with-keyboard) section. <br />
**NOTICE IV:** Please Follow Terminal Steps.

1. On the Jetson, open two python files in directory, _~/catkin_ws_race/arc/race/src_, named `control.py` and `dist_finder.py`.

2. Create, input and make sure your code are in those two files.

3. After you finish those two files together with `kill.py` in the same directory, copy them to directory _~/catkin_ws_race/install/share/race_.

4. In your laptop, open **seven** terminals and SSH all of them into the Jetson.

5. In Terminal 1, use command `$ roscore` to initialize ROS system.

6. In Terminal 2, use command `$ ‘/home/ubuntu/Desktop/Bridge.sh’` to build the bridge of the LIDAR and the Jetson.

7. In Terminal 2, use command `$ rosrun urg_node urg_node _ip_address:=192.168.1.11` to start LIDAR broadcasting.

8. In Terminal 3, use command `$ rosrun rosserial_python serial_node.py /dev/ttyACM0` to start the communication between the Teensy and the Jetson.

9. In Terminal 4, use command `$ rosrun race talker.py` to start the talker.

10. In Terminal 5, use command `$ rosrun race kill.py` to make the kill operation standby.

11. In Terminal 6, use command `$ rosrun race control.py` to start the auto-control of the car.

12. In Terminal 7, use command `$ rosrun race dist_finder.py` to start the distance finding procedure. And once you enter this command, the car will start moving.

13. Use `\` for normal stop and `(SPACE)` for emergency stop of the car in Terminal 5.

14. Use `control` + `c` to close other Terminals.
