### Setting up Teensy

**Notice 1:** The Jetson TK1 is running Ubuntu 14.04 LTS with an ARM32 core. <br />
**Notice 2:** You can also try to do this on your own laptop.

1. Download Arduino IDE from the [official site](https://www.arduino.cc/en/Main/Software) to Jetson.

2. Extract the file to directory _:/home_.

3. In a Terminal, use **sudo** command to run the `install.sh` file of Arduino.

4. Download Teensyduino from the [official website](https://www.pjrc.com/teensy/td_download.html).

5. Use command `$ sudo nano 49-teensy.rules` to create a new rule file.

6. Click on `Linux udev rules` on the top-right corner of the [website](https://www.pjrc.com/teensy/td_download.html) mentioned above. 
  Copy and paste everything inside into the Terminal window of 49-teensy.rules.

7. Save and **quit** the file editing window (**NOT EXIT**).

8. Use command `$ sudo cp 49-teensy.rules /etc/udev/rules.d/` to add the rule into the root system. 
  This will give you access to the Teensy device.
  
9. Locate the downloaded Teensduino file and use command `$ chmod 755 (File Path)` to run the file in a Terminal. 
  Replace `(File Path)` to your own file path. This will make the file executable. 

10. Double click on the file to install Teensyduino.

11. In a Terminal, use command `$ sudo apt-get install ros-indigo-rosserial-arduino` and `$ sudo apt-get install ros-indigo-rosserial`
  to install rosserial packages.
  
12. Use command `$ mkdir -p catkin_ws_rosserial/src` to create a rosserial package path.

13. Use command `$ git clone https://github.com/ros-drivers/rosserial.git` to clone the rosserial packages from github.

14. In Files, **Drag** all file inside _/rosserial_ folder into _/catkin_ws_rosserial/src_.

15. Delete the clone folder (_/rosserial_) once finish moving all those files.

16. In a **NEW** Terminal, use command `$ cd ~/catkin_ws_rosserial` to get inside the catkin_ws_rosserial folder in Terminal.

17. Use command `$ catkin_make` to make the catkin workspace.

18. Use command `$ cd ~` to go back to default page in Terminal or open a new Terminal.

19. Use command `$ sudo nano .bashrc` to access the bashrc file.

20. Add command: `source ~/catkin_ws_rosserial/devel/setup.bash` to the END of the file.

21. Save and exit.

22. Use command `$ git clone https://github.com/mlab-upenn/f1_10_code_public/` to obtain essential files.

23. Copy and paste `teensy_drive.ino` from _/f1_10_code_public/Teensy Firmware_ to the _/home_ page.

24. Double click on the `teensy_drive.ino` under _/home_ and click okay once prompt.

25. Go inside the teensy_drive folder and create a folder named _libraries_.

26. Use command `$ cd /teensy_drive/libraries` to get inside the folder in Terminal.

27. Use following commands to create a ROS library for Teensy. <br /> 
  `$ rm -rf ros_lib` <br />
  `$ rosrun rosserial_arduino make_libraries.py` <br />
  
28. After adding the library, restart the Arduino IDE. 
  If you find ros_lib under _File_ > _Example_, it means you add the libraries correctly.
  
29. Connect the Teensy to the Jetson via a USB.

30. Open the Teensy Drive file in Arduino IDE and then go to the Tool menu.

31. Inside the menu, make sure that the broad is `Teensy 3.2/3.1`; Use Type is `Serial`; CPU Speed is `96 MHz`.

32. Upload the drive to the Teensy.

33. Click the button on Teensy once you finish uploading. There should also be a window pop out in the desktop named Teensy.

34. Make sure that Teensy reboots before disconnect.
