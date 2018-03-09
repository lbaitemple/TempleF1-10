### Fix Problem with Tribooting a Mac

- #### Fix non-bootable Windows 10, the blue screen Windows 10
1. In **Ubuntu**, make sure you are connected to the internet, open Terminal

2. Use command `sudo apt-get install gdisk` to install gdisk. You will need to enter your password.

3. After installation of gdisk, use command `sudo gdisk /dev/sda`.

4. Press `P` to verify if you are working on the correct disk. Press `Q` to exit if not.

5. In gdisk, press `X` to enter expert menu.

6. Press `N` to creat a new and the press `W` to write and confirm the change.


- #### Change default boot to Mac in Ubuntu
1. In **Ubuntu**, open Terminal.

2. Use command `sudo efibootmgr - v` to load the boot manager.

3. Use command `sudo efibootmgr -o 0080,0000` to change the order of boot. Initially, Mac is 0080 and ubuntu is 0000.

4. Once finish, **make sure** that Mac is 0000 and ubuntu is 0080.


- #### Enter Tribooting Screen by default upon reboot via rEFInd
1. Reboot Mac into recovery mode by holding `Command` and `R` at the same time.

2. Go to _Utilities > Terminal_.

3. In Terminal, enter `csrutil disable` and press `return` on keyboard.

4. Reboot into **macOS** and log into Administrator account.

5. Go to _System Preferences > Startup disk_, and **make sure** it is locked onto `macOS`.

6. Download the _binary zip file_ for rEFInd file and put the file somewhere safe on the disk. [Here is the link](http://www.rodsbooks.com/refind/getting.html)

7. After downloading, upzip the file.

8. Open Terminal, then drag the `refind-install` file from the just downloaded file into the Terminal and press `return` on the keyboard.

9. Enter your passcode and press `return` on the keyboard.

10. When it shows _Installation has completed successfully_, it means it is properly installed.
