.. _doc_optional_software_nx:

Optional Compute: NVIDIA Jetson Xavier NX
=========================================

In place of the NVIDIA Jetson TX2 module, an NVIDIA Jetson Xavier NX Developer Kit can be used. However, the software setup for this module is significantly different than the setup for the TX2. Unlike the TX2, the Xavier NX does not require any special software on the Host/Pit computer.

**Equipment Used:**

* Pit/Host laptop/computer running any Operating System
* Fully-built F1TENTH vehicle
* microSD card (16GB minimum)
* USB micro cable (must have both data and power wires)
* microSD card reader/writer for Pit/Host PC
* SD card image burning software (e.g. Balena Etcher)
* Terminal emulation software (e.g. PuTTy, screen, miniterm.py, etc.)

**Approximate Time Investment:** 1-2 hours

Initial Configuration Steps
---------------------------

1. Go to the NVIDIA Develoeprs Download Center at https://developer.nvidia.com/downloads and click Jetson.

        .. figure:: img/nx/nx-software-step1.png
                :align: center

                NVIDIA Developers Download Center

2. Under SD Card Image Method, click Jetson Xavier NX Developer Kit.

        .. figure:: img/nx/nx-software-step2.png
                :align: center

                Jetson Xavier NX Developer Kit downloads.

3. The next page will require you to log in with an NVIDIA Developer Program login. If you do not have one, click Join Now. If you already have an account, click Login. Registration is free.

        .. figure:: img/nx/nx-software-step3.png
                :align: center

                NVIDIA Developer Program login page.

4. Once you have logged in, you will be redirected to your profile settings page. At the top of this page, you should see a banner with a button with the text "Jetson Xavier NX Developer Kit SD Card Image." Click this button.

        .. figure:: img/nx/nx-software-step4.png
                :align: center

                NVIDIA Developer Program profile page download button.

5. Once the zip file has finished downloading, extract it. This will create a new file, sd-blob.img. This is the file that contains the NVIDIA Jetson Xavier NX Developer Kit software and operating system.
6. Put the microSD card into the SD card reader/writer. If your SD card reader/writer is not part of the host PC (e.g. a USB stick) then plug it into the host PC.
7. Open your SD card image burning software.

        .. figure:: img/nx/nx-software-step7.png
                :align: center

                SD card burner software.

8. Choose to flash the image from a file. When the file selection window comes up, choose the sd-blob.img file extracted earlier.

        .. figure:: img/nx/nx-software-step8.png
                :align: center

                File selection window.

9. For the "target" device, choose the microSD card in the microSD card reader/writer.

        .. figure:: img/nx/nx-software-step9.png
                :align: center

                Target selection window.

10. Click "Flash!" (or similar for your software). This process will take some time (20+ minutes).

        .. figure:: img/nx/nx-software-step10.png
                :align: center

                Flashing process.

11. Once the flashing process is complete, verify that any activity lights on your SD card reader/writer are no longer blinking. Properly un-mount/eject the microSD card before physically removing it from the reader/writer.
12. Insert the microSD card into the NVIDIA Jetson Xavier NX module with the label facing up. Push the microSD card all the way in until it locks into place with a small click. The edge of the microSD card should be flush with the PCB of the NVIDIA Jetson Xavier NX module and carrier board.

        .. figure:: img/nx/nx-insert-sd.jpg
                :align: center

                Insert SD card.

        .. figure:: img/nx/nx-sd-inserted.jpg
                :align: center

                SD card flush.

13. Connect the USB micro end of the USB micro cable to the USB micro port on the NVIDIA Jetson Xavier NX carrier board. Connect the USB A end of the USB micro cable to the host PC.

        .. figure:: img/nx/nx-attach-usb.jpg
                :align: center

                Attaching USB micro end of cable.

14. Connect the battery on the F1TENTH vehicle.
15. Flip the switch on the power distribution board to the ON position.
16. After several minutes, you should see a new drive become available on the host PC called "L4T-README." If you do not see this then either the flashing of the microSD card failed or your USB cable is bad or incorrect in some way (e.g. missing data lines).
17. In addition to the new drive, you should also have a new Serial, COM, or TTY device available. On Linux and MacOS, this will be in the form of /dev/ttyACMx where x is a number. On Windows, this will be a new COM port. Open your terminal emulator software and connect to this new port using the following settings:

* Baud rate: 115200 bps
* Data bits: 8
* Stop bits: 1
* Parity: None
* Flow control: None

18. Once connected, you may not see any output on the terminal. Hitting the space bar should show you the license agreement for the NVIDIA software.

        .. figure:: img/nx/nx-software-step18.png
                :align: center

                NVIDIA license agreement.

19. Hit TAB to select the ``<Ok>`` button. Hit ENTER to accept the license agreement.
20. On the next screen, choose your language of choice and hit ENTER.

        .. figure:: img/nx/nx-software-step20.png
                :align: center

                Language selection.

21. On the next screen, select your region to properly set the time zone and hit ENTER.

        .. figure:: img/nx/nx-software-step21.png
                :align: center

                Region selection.

22. On the next screen, choose your time zone and hit ENTER.

        .. figure:: img/nx/nx-software-step22.png
                :align: center

                Time zone selection.

23. On the next screen, you will be asked if the system clock is set to UTC. Choose <Yes> and hit ENTER.

        .. figure:: img/nx/nx-software-step23.png
                :align: center

                System clock base selection.

24. On the next screen, you will be asked to enter a name for the new user account. Enter ``f1tenth``, hit TAB to select the ``<Ok>`` button, and then hit ENTER.

        .. figure:: img/nx/nx-software-step24.png
                :align: center

                User account full name selection.

25. On the next screen, you will be asked to enter a username for the new user account. Leave the default of ``f1tenth``, hit TAB to select the ``<Ok>`` button, and hit ENTER.

        .. figure:: img/nx/nx-software-step25.png
                :align: center

                Username selection.

26. On the next screen, you will be asked to enter a password for the new user. Enter the password ``G0Fast!`` (with a zero instead of the letter o). Hit TAB to select the ``<Ok>`` button, and hit ENTER.

        .. figure:: img/nx/nx-software-step26.png
                :align: center

                Password selection.

27. On the next screen, you will be asked to re-enter the password. Enter the password again, hit TAB to select the ``<Ok>`` button, and then hit ENTER.

        .. figure:: img/nx/nx-software-step27.png
                :align: center

                Password re-enetry.

28. On the next screen, you will receive a warning that the selected password is "too weak" due to the lenth. Hit TAB to select <Yes> and then hit ENTER.

        .. figure:: img/nx/nx-software-step28.png
                :align: center

                Weak password confirmation.

29. On the next screen, you will be asked to select the desired size of the APP partition. Leave the default, hit TAB to select the ``<Ok>`` button, and then hit ENTER.

        .. figure:: img/nx/nx-software-step29.png
                :align: center

                APP partition size selection.

30. On the next screen, you will be asked to select a primary network interface. Use the arrow keys to select ``eth0``, hit the TAB key to select the ``<Ok>`` button, and then hit ENTER (we will change this after setup is complete).

        .. figure:: img/nx/nx-software-step30.png
                :align: center

                Primary network interface selection.

31. The next several screens will show the status of connecting to the network. Since there is no Ethernet cable connected to ``eth0``, this is expected to fail. Hit ENTER to continue.

        .. figure:: img/nx/nx-software-step31.png
                :align: center

                Network connection failure.

32. On the next screen, you will be given several options on how to proceed with connecting to a network. Use the arrow keys to select ``Do not configure the network at this time``, hit the TAB key to select the ``<Ok>`` button, and then hit ENTER.

        .. figure:: img/nx/nx-software-step32.png
                :align: center

                Network configuration selection.

33. On the next screen, you will be asked to enter the hostname for the NVIDIA Jetson Xavier NX. Erase the current text and type ``jetson-nx``. Hit TAB to select the ``<Ok>`` button, and then hit ENTER.

        .. figure:: img/nx/nx-software-step33.png
                :align: center

                Hostname selection.

34. The next several screens will show the status of the installation and configuration of the NVIDIA Jetson Xavier NX system. During this process, your terminal session will likely be interrupted and the L4T-README drive will be removed and reconnected.
35. Wait at least 30 seconds and then reconnect your terminal session using the same settings as before. This time you should be prompted with a login for the device. Enter the username ``f1tenth`` and then hit ENTER.

        .. figure:: img/nx/nx-software-step35.png
                :align: center

                Terminal login.

36. You will then be prompted for the password. Enter the password ``G0Fast!`` and hit ENTER. Note that you will not be able to see the characters being entered as you type.
37. You should now be logged in to the NVIDIA Jetson Xavier NX Developer Kit.

        .. figure:: img/nx/nx-software-step37.png
                :align: center

                Logged in!


Configuring WiFi and SSH
------------------------

1. We will use the Network Manager command-line tool nmcli to configure the WiFi on the NVIDIA Jetson Xavier NX. To find the interface name of your WiFi adapter, start by typing ``nmcli d`` and hitting ENTER. This will list your available interfaces. My wifi interface is named ``wlan0`` so I will use that in all future steps. If your WiFi interface is named something different, you will have to replace that in future commands.

        .. figure:: img/nx/nx-wifi-step-1.png
                :align: center

                WiFi network selection.

2. To make sure that your WiFi radio is turned on, type ``nmcli r wifi on`` and hit ENTER. This will not show anything on the terminal if the command succeeded.

        .. figure:: img/nx/nx-wifi-step-2.png
                :align: center

                Enable WiFi radio.

3. To see the list of WiFi SSIDs that your WiFi adapter can see, type ``nmcli d wifi list`` and hit ENTER. After the list is printed, hit ``q`` to continue.

        .. figure:: img/nx/nx-wifi-step-3.png
                :align: center

                WiFi SSID selection.

4. To connect to a specific WiFi SSID, use the command ``sudo nmcli d wifi connect [SSID] password [PASSWORD]`` where ``[SSID]`` is replaced with the SSID with which you want to connect and ``[PASSWORD]`` is replaced with the password to connect to that SSID. Hit ENTER.

        .. figure:: img/nx/nx-wifi-step-4.png
                :align: center

                Connect to specific WiFi network.

5. If the connection was successful, you should see the message ``Device 'wlan0' successfully activated with [GUID]``.
6. By default, WiFi will be connected using DHCP which means you may get a new IP address each time the device is turned on. In the next steps, we will configure the WiFi connection with a static IP address so you can SSH into the Developer Kit reliably. To set a static IP address, you will need to know the subnet, IP address range, and gateway of your wifi network.
7. To get the currently-assigned IP address use the command ``ip addr show dev wlan0``.

        .. figure:: img/nx/nx-wifi-step-7.png
                :align: center

                Currently-connected WiFi IP address.

8. To set a static IP address, you will also need to know the name of the connection. This is usually the same as the SSID of the WiFi network but not always. To see the list of current connections, use the command ``nmcli c show``.

        .. figure:: img/nx/nx-wifi-step-8.png
                :align: center

                List of connections.

9. To set a static IP address use the command ``sudo nmcli c mod [CONNECTION_NAME] ipv4.address [NEW_ADDRESS]/[CIDR]`` where ``[CONNECTION_NAME]`` is replaced with the name of your WiFi connection that you got from step 8, ``[NEW_ADDRESS]`` is replaced with the static IP address that you want to set, and ``[CIDR]`` is the `CIDR representation <https://www.ionos.com/digitalguide/server/know-how/cidr-classless-inter-domain-routing/>`_ of the subnet (usually 24).

        .. figure:: img/nx/nx-wifi-step-9.png
                :align: center

                Setting static IP address.

10. To set the connection's default gateway, use the command ``sudo nmcli c mod [CONNECTION_NAME] ipv4.gateway [GATEWAY_IP]`` where ``[CONNECTION_NAME]`` is replaced with the name of your WiFi connection that you got from step 8 and ``[GATEWAY_IP]`` is replaced with the IP address of your WiFi network's gateway/router.

        .. figure:: img/nx/nx-wifi-step-10.png
                :align: center

                Setting IP gateway.

11. To set the connection's DNS servers, use the command ``sudo nmcli c mod [CONNECTION_NAME] ipv4.dns "[DNS_SERVER1]"`` where ``[CONNECTION_NAME]`` is replaced with the name of your WiFi connection that you got from step 8 and ``[DNS_SERVERS]`` is replaced with a comma-separated list of DNS server IP addresses. Google DNS servers at 8.8.8.8 and 8.8.4.4 are recommended.
12. To disable DHCP and always use the static IP address on this connection, use the command ``sudo nmcli c mod [CONNECTION_NAME] ipv4.method manual`` where ``[CONNECTION_NAME]`` is replaced with the name of your WiFi connection that you got from step 8.

        .. figure:: img/nx/nx-wifi-step-12.png
                :align: center

                Setting connection to always use static IP.

13. To save the changes you've made, run the command ``sudo nmcli c up [CONNECTION_NAME]`` where ``[CONNECTION_NAME]`` is replaced with the name of your WiFi connection that you got from step 8.
14. To verify that you can SSH into the NVIDIA Jetson Xavier NX Developer Kit, verify that the Pit/Host PC is connected to the same network as the Jetson Xavier NX Developer Kit and use an SSH client on the Host PC to connect to the new IP address of the Developer Kit. On Linux this would be done with the command ``ssh f1tenth@[IP_ADDRESS]`` where ``[IP_ADDRESS]`` is replaced with the static IP address that you assigned to the Developer Kit. After you have verified that SSH works correctly, you can close the connection to the Developer Kit in your terminal emulator and disconnect the micro USB cable.

Updating Packages
-----------------

1. All further steps assume that your NVIDIA Jetson Xavier NX Developer Kit is connected to the internet and you are connected to the Developer Kit via SSH.
2. To update the list of available packages, run ``sudo apt update``.
3. To install all available updates, run ``sudo apt full-upgrade``.
4. Once all packages have been upgraded run ``sudo reboot`` to restart the Developer Kit and apply any changes.

Creating a Swapfile
-------------------

1. Run the following commands to create a swapfile which can help with memory-intensive tasks

  .. code:: bash

    sudo fallocate -l 4G /var/swapfile
    sudo chmod 600 /var/swapfile
    sudo mkswap /var/swapfile
    sudo swapon /var/swapfile
    sudo bash -c 'echo "/var/swapfile swap swap defaults 0 0" >> /etc/fstab'

Installing ROS
--------------

We use ROS to connect everything together and ultimately run the car. We'll need to set up the :ref:`ROS workspace <ros_workspace>`, set up some :ref:`udev rules <udev_rules>`, and :ref:`test the lidar connection <lidar_setup>`. Everything in this section is done on the **Jetson Xavier NX** so you will need to connect to it via SSH from the **Pit/Host** laptop/computer or plug in a monitor, keyboard, and mouse.

These instructions are specific to setting up the software on the Jetson Xavier NX as the setup is a bit different than the TX2. Many thanks to `Jim from JetsonHacks <https://www.jetsonhacks.com/>`_ and `Josh Whitley from The Autoware Foundation <https://autoware.org/>`_ for figuring this out.

1. Install the Logitech F710 driver on the Jetson.

  .. code:: bash

    git clone https://github.com/jetsonhacks/logitech-f710-module
    cd logitech-f710-module
    ./install-module.sh

2. Install ROS

  .. code:: bash

    $ cd ~
    $ git clone https://github.com/jetsonhacks/installROS
    $ cd installROS
    $ ./installROS -p ros-melodic-ros-base
    $ ./setupCatkinWorkspace.sh f1tenth_ws

  (This will setup a catkin workspace in the home directory named ``f1tenth_ws``)

3. We are now ready to install the F1/Tenth ROS packages

  .. code:: bash

    cd ~/f1tenth_ws/src
    git clone https://github.com/f1tenth/f1tenth_system
    find . -name '*.py' -exec chmod +x {} \;
    cd ..
    source devel/setup.bash
    rosdep install -a -y -i
    catkin_make
