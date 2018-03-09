### Configuring LINDAR Sensor

1. Go into Windows 10 and download the configuration software UrgBenri. [Here is the link](https://sourceforge.net/projects/urgbenri/)

2. Once finish downloading, run and install the software.

3. Connect the LIDAR to your laptop via an ethernet cable.

4. Go to _Control Pannel > View Network Status and Task > Change Adapter Settings_.

5. Select and right click on the corresponding ethernet connection and select `Properties`.

6. Double click and open the `Internet Protocol Version 4 (IPv4)` tab.

7. Select `Use the following IP address`, and change into following values on the left of the attachment:
<p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/LINDARSetting1.png?raw=true" width="80%"/>
</p>

8. Click OK to apply and the UrgBenri will show a map (As on the right of the attachment above). <br />
  If it does not show up, make sure your LIDAR is reset to default.

9. Scroll down the right column and select `Change IP Address`.

10. Change the LIDAR’s IP address into following values: <br />
  IP Address: 192.168.1.11 <br /> 
  Mask: 255.255.255.0 <br /> 
  Gateway: 12.168.1.1 <br />
  **Notice:** This IP Address will be shown up in Ubuntu later on. Make sure you remember this IP Address.
  <p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/LINDARSetting2.png?raw=true" width="50%"/>
</p>
  
  
11. Click `Update` to apply changes.

12. The LINDAR will restart and reconnect, the map be blank for now.

13. After LINDAR’s reconnection, change the IPv4 setting into these values as shown on the left of the attachment: <br />
   IP Address: 192.168.1.15 <br />
   Subnet Mask: 255.255.255.0 <br />
   Default Gateway: 192.168.1.1 <br />
   <p align="center">
  <img src="https://github.com/UniKerogen/TempleF1-10/blob/master/Documents/Miscellaneous/Pictures/LINDARSetting3.png?raw=true" width="80%"/>
</p>

14. Click `Okay` to apply changes.

15. If the map is regenerated in the UrgBenri as shown on the right of the attachment above, then it means the IP Address of the LIDAR has been changed successfully.
