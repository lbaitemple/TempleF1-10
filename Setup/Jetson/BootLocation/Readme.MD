### Instructions for Ubuntu on Jetson TK1

- #### Switch Default Boot Location of the Jetson
- [ ] Verified  **NOTICE** -> This section is not verified

1. Power on the Jetson and connect it to an external monitor and a set of mouse and keyboard.

2. Insert a SD card to the SD card slot.

3. Use the following commands for preparations: <br />
  `$ sudo midst.ext4 /dev/mmcblk1p1` <br />
  `$ wget http://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_tk1/007/linux-x64/Tegra124_Linux_R21.4.0_armhf.tbz2` <br />
  `$ wget http://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_tk1/007/linux-x64/Tegra_Linux_Sample-Root-Filesystem_R21.4.0_armhf.tbz2`
  
4. Use the following commands to Mount SD card and extract them to SD card: <br />
  `$ sudo tar xpf Tegra124_Linux_R21.4.0_armhf.tbz2` <br />
  `$ sudo mount /dev/mmcblk1p1 Linux_for_Tegra/rootfs` <br />
  `$ cd Linux_for_Tegra/rootfs $ sudo tar xpf ../../Tegra_Linux_Sample-Root-Filesystem_R21.4.0_armhf.tbz2` <br />
  
5. Use to following commands to reRun the `apply_binaries.sh` script to copy the NVIDIA libraries into rootfs directory: <br />
  `$ cd ..` <br />
  `$ sudo ./apply_binaries.sh`

6. Use the following commands to copy `extlinux.conf` filePath to a kernel file and kernel parameters are written in _/boot/extlinux/extlinux.conf_. U-Boot read this file and load kernel. **If** U-Boot failed to read this file, it tries to boot from other storage. <br />
  `$ cd rootfs/boot/extlinux/` <br />
  `$ sudo cp jetson-tk1_extlinux.conf.sdcard extlinux.conf`
  
7. Use to following commands to unmount SD card: <br />
  `$ cd ../../../` <br />
  `$ sync` <br />
  `$ sudo umount rootfs`

8. Log into the Jetson and use command `$ df -h` to check root file size.

9. The root size should be the size of the SD card.

10. You **HAVE TO** make sure that the Jetson is running on SD card before proceed to the next section.
