### Configuring Pico Station

1. Power on the Jetson and connect it to an external monitor and a set of mouse and keyboard. Make sure that Pico Station’s cable is plugged in.

2. Open Terminal and use command `$ ls /sys/class/net` to verify the internet connection of the Jetson. Make sure you know which on is Pico Station.

3. Use command `$ ifconfig` if you want to know more about the connection.

4. Go to _System Setting > Network > Wired_ and select corresponding wired connection.

5. Click on `Options` and go to `IPv4 Setting` Tab.

6. Select Method: `Mannual`.

7. Add the following address: <br />
  Address: 192.168.1.1  <br />
  Netmask: 255.255.255.0 <br />
  Gateway: 192.168.1.1 <br />
  
  <p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/PICOIPv4.png?raw=true" width="50%"/>
  </p>
  
8. Save and close.

9. Open a web browser and access Pico Station’s IP address, then log in. The default IP address is `192.168.1.20` and the default user name and passcode are `ubnt`.

10. For internet access uses: upgrade system file and install applications go to Step 11 - 14. <br />
  For access uses: connect to the Jetson broad on other laptops without internet connection, please **SKIP** Step 11 - 14. <br />
  However, we do not recommend to put the Jetson in access use only. You can always ssh into the Jetson if it is in the same Wifi network as your laptop.
  
11. Select and go to `Wireless` tab.

12. Change sections into values in the picture: <br />
  **NOTICE:** Pick your Password
<p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/PICOWirelessTab.png?raw=true" width="95%"/>
</p>

13. Select and Go to `Network` tab.

14. Change sections into values in the picture: <br />
<p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/PICONetworkTab.png?raw=true" width="95%"/>
</p>

15. For internet access uses, please **SKIP** Step 16 - 20.

16.  Go to `Wireless` Tab

17. Change sections into values shown in the picture: <br />
  **Notice:** Pick your Password
  <p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/PICOWirelessTabAccess.png?raw=true" width="95%"/>
  </p>
  
18. Select and go to the very first tab, the one before `Main`.

19. Uncheck `airMax`, in other words **DISABLE** airMAX.

20. The `Network` tab should be kept as same as the internet access setup.

21. Open Terminal.

22. Use command `$ sudo nano /etc/network/interfaces` to access the interfaces file.

23. Inside the interfaces file, make sure that there is **NOTHING** executed **BESIDES** the line starts with: source-directory

24. Quit and make sure the configuration of the LAN.

25. In the internet access mode, now you can ssh into the Jetson as long as your laptop is in the same network as the Jetson. <br />
  In the access mode, now you can ssh into the Jetson as long as your laptop is in the network of the Pico Station.
  
**NOTICE:** It is recommended to set the Pico Station in Station mode and connected to certain portable Wifi network.

