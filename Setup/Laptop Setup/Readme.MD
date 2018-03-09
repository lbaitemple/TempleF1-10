## Instruction for Installing Ubuntu on Laptops 

In this specific instruction, we will tri-boot a Macbook Pro. 
We will first install Windows 10 in Macbook running macOS 10.12.5 and then install ubuntu on Windows 10.

**IF** you are running Windows 10 already, skip to _Setting up Ubuntu on Windows_

- ### Setting up Windows on Mac
**Before** you try any of this, make sure your laptop is backed up.

1. Make sure that you have sufficient storage space for your Mac.

2. Back up MacBook via Time Machine or else.

3. Download Windows 10 iso file from Microsoft’s official website. The iso file should be about 5 GB. As for March 2017, the version they offer is 1709. [Here is the Link](https://www.microsoft.com/en-us/software-download/windows10ISO)

4. After download the iso file, make sure MacBook has sufficient storage to install Windows. Go to the apple logo on the top left corner of the screen, click and go to _About this Mac > Storage_. You will need at least 70GB free space to make sure all three system will function properly without storage problems.
<br />- Generally, Windows 10 takes about 30 GB for system file, so we recommend 50GB for Windows 10; Ubuntu takes about 5GB for system file, so we recommend 10 GB for ubuntu. The installation takes about 8GB of your system storage, which will **not** be accessible for either of the system. [Please CHECK the further requirement here](https://support.apple.com/en-us/HT201468)

5. Open _Boot Camp Assistant_. It can be found under _LaunchPad > Utilities_ or search it from Spotlight Search or ask Siri to find it for you.

6. Click `Continue`.


7. Check `Create a Windows 7 or later version install disk`. Then `Continue`.


8. Select the iso file just downloaded from Microsoft, then `Continue`.

9. In _Create a Partition for Windows_, the partition of Windows means the total size of Windows 10 and Ubuntu. So as recommended, slide the bar and make sure it is 60GB for Windows. Then Install. <br />- During the process, Mac will automatically download the support file and drives. It will restart when it is done.

10. Once Mac reboot into the installation process, select your desired language, number formats, and keyboard. Then click on `Next`.

11. If you have a active key, active Windows. Otherwise click on `Skip`.

12. Once prompt _Where do you want to install Windows_, select the drive that has 60GB, usually it will show up as _“Drive 0 partition X: BOOTCAMP”_ and format the drive. After formation, click on `Next`.

13. Windows **now** will start the installation.

14. Once Mac reboot into Windows, if not prompted Boot Camp Installer, go to _My Computer > OSXRESERVED > BootCamp_. Select and run `Setup.exe` to install BootCamp on Windows. Bootcamp ensure compatibility with Windows 10’s software and Mac’s hardware.

15. Now Windows should be running correctly on Mac. You can switch back and forth from macOS and Windows by holding on `ALT` button upon boot. You can also right click on the bootcamp icon in Windows’ task bar and select “Restart in OS X” to reboot into macOS.

#### **Notice:**
Starting macOS 10.13, aka macOS High Sierra, Option `"Restart in OS X"` is not avaliable. You will need to hold `alt` while booting, specifically after hearing the starting sound, in order to switch operating systems.

- ### Setting up Ubuntu on Windows
**Before** you try this, make sure your laptop is backed up

1. Back up Windows 10 into an external storage unit.

2. In Windows 10, right click on the Windows logo in the task bar and select `Disk Management`.

3. In _Disk Management_, select _WindowsOS Volume_, (C:), and right click then select `Shrink`.

4. In the _Shrink Menu_, enter the amount of space to shrink.
<br />- The **entered amount** will be your Ubuntu volume and the rest will stay in Windows. Use the equation 1GB = 1000MB to determine the desired size.

5. After finish shrinking the disk, format the new volume into NTFS format by right click on the corresponding volume in Disk Management and select `format`.

6. Go to Ubuntu’s official site and download desired firmware for ubuntu. Usually, we use the latest desktop LTS version unless suggested otherwise. [Here is the link](https://www.ubuntu.com/download/desktop)

7. Plug in a USB drive, at least 2GB, and create a bootable USB from Windows 10. [Here is the instruction on creating a bootable USB on Windows 10](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0)
<br />- MAC: Upon reboot into the bootable USB, hold `ALT` button to choose the USB.
<br />- Windows: Upon reboot into the bootable USB, hole `F11` to choose the USB. You can also use other methods.

8. Once it starts up, select _Try Ubuntu_ using arrow keys.

9. In the Try Ubuntu session, **make sure everything works**, including network and keyboard.

10. Double click on `install Ubuntu` on the desktop to start the installation. Do **NOT** worry if Wifi is not connected during the installation.

11. During the pre-installation, follow the instruction until prompted _Allocate drive space_.

12. In Allocate drive space, select `Something else`.

13. Select the desired disk and format it into **EXT4 format**. If you need a swap space, you can separate desired portion from the EXT4 disk and format into Swap.

14. Select `/dev/sda1` as boot-loader installation. Then click on `install Now`.

15. Follow the instruction during the installation.

16. **If** you are a Mac User, in “keyboard layout”, select `Macintosh keyboard layout`.

17. In _Who are you?_, we suggest to check the _Log in automatically_ selection.

18. After the installation, **_if_** you are a Mac User, do **NOT** restart Mac now and go to the _Mac Troubleshoot_ section.
