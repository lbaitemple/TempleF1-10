### Instructions for Adding and Breaking Bridges

#### Adding a bridge on the Jetson Board

1. Put Pico Station in Station Mode to access the internet.

2. Open a Terminal and use command `$ sudo apt-get update` and `$ sudo apt-get upgrade` to update and upgrade your packages.

3. Use command `$ sudo apt-get install autotools-dev` to install autotools.

4. Use command `$ sudo apt-get install autoconf` to install autoconf.

5. Use the following commands to install brctl command function: <br />
  `$ git clone git://git.kernel.org/pub/scm/linux/kernel/git/shemminger/bridge-utils.git` <br />
  `$ cd bridge-utils` <br />
  `$ autoconf` <br />
  `$ ./configure` <br />
  After executing these commands, brctl should be in your command tools.

6. Use command `$ sudo apt-get update` and `$ sudo apt-get dist-upgrade` to force update and upgrade your packages again.

7. Use the following commands to install Grinch Kernel to the Jetson: <br />
  `$ git clone https://github.com/jetsonhacks/installGrinch.git` to clone the file under directory: `./installGrinch` <br />
  `$ cd ./installGrinch` <br /> 
  `$./installGrinch.sh`
  
8. After install Grinch Kernel, reboot the Jetson.

9. After rebooting the Jetson, open a Terminal.

10. Use command `$ ./installGrinch/installGrinchNoDownload.sh` to verify the install.

11. Use command `$ sudo apt-get install bridge-utils` to install bridge utility.

12. Use command `$ sudo apt-get install nmap` to install nmap.

13. Use command `$ sudo apt-get install ros-indigo-urg-node` to install urg-node package.

**NOTICE:** If you want to type everytime for bridge, **SKIP** Step 14 - 21, and type in commands in Step 15. <br />
14. On Desktop, right click and create a new file.

15. Inside the file, put in the following lines:<br />
  `sudo brctl addbr br0` <br />
  `sudo brctl addif br0 eth0` <br />
  `sudo brctl addif br0 eth1` <br />
  `sudo ifconfig br0 192.168.1.99 netmask 255.255.255.0 up` <br />
  `ifconfig br0`
  
16. Save and exit.

17. Right click on the file and go to properties.

18. In _Properties > Basic_, change the name into `Bridge.sh`.

19. In _Properties > Permission_, allow executing file as program.

20. Save and exit _Properties_.

21. Drag the file into a Terminal and run the file. It should show up the configuration as “br0" once established the connection.

22. Use command `$ nmap -sP 192.168.1.0/24` to find out all the connection to the network that the Jetson is on. The LIDAR should also show up in the form of its corresponding IP address.

**NOTICE:** To find Streaming data, refer to Mapping.
