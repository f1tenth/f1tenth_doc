.. _doc_software_combine:


Bringing the Host and Jetson Together
======================================
We could log into the Jetson using a monitor, keyboard, and mouse, but what about when we’re driving the car? Fortunately, the Jetson has Wi-Fi capability and can be accessed remotely via an SSH session. Throughout this tutorial, you will be asked to configure the Jetson’s and your laptop’s network settings. Make sure to get these right! Using the wrong IP address may lead to conflicts with another classmate, meaning neither of you will be able to connect.

Connecting the Jetson/Car to the Access Point
----------------------------------------------

#. Click the wireless icon at the top right of the screen and click the f110 network to start connecting to it. You will be prompted for a Wi-Fi password for the network: enter the password the TAs give you.

	* It’s normal for the wireless icon to appear as if the Jetson is not connected immediately to the network since we still need to assign it an IP address.

#. In the same menu, click “Edit Connections.” In the pop-up value that appears, highlight the f110 network and click the Edit button.

#. Navigate to the IPv4 Settings tab and, under “Addresses,”, click the Add button.

	* In the “Address” field, type ​192.168.2.xxx​, where ​xxx​ is your team’s number plus 200. (For example, if I was on team 2, I would type ​192.168.2.202​.)
	* In the “Netmask” field, type ​255.255.255.0​.
	* In the “Gateway” field, type ​192.168.2.1​.

#. In the “DNS servers” field, type the same entry you used for the default gateway: 192.168.2.1​. (The router already has DNS servers configured in its internal settings.)

#. You should now be connected. Try opening Chromium and connecting to a site like Google, or using the ​ping​ utility from a terminal to test internet connectivity.

	* If you experience signal strength issues, try moving closer to the router.
	* If you can’t see the router at all, ensure that your Wi-Fi antennas are securely connected to the Jetson. You can also try toggling the adapter on and off via the “Enable Wi-Fi” option in the wireless settings menu.
	* If you are connected to the router but can’t reach the internet, you may need to set up the Hokuyo to not allow routing through it.

Connecting Your Host/Laptop to the Access Point
-------------------------------------------------
Important Note​: when connecting your laptop to the router, use an IP address of the form 192.168.2.xxx​, where ​xxx​ is your team’s number multiplied by 4, added to 100, and then added to a number between 0 and 3 according to the alphabetical order of your last name in your team. For example, if I am on team 2, my name is Jack Harkins, and my teammates are Chris Kao, Sheil Sarda, and Houssam Abbas, I would add 1 since my last name (Harkins) comes second, making my final IP address ​192.168.2.209​.

Linux
^^^^^^
If you’re running Linux in a dual-boot configuration or as a standalone OS, the steps to connect are the same as those for the Jetson above; just make sure you use the correct IP address for your laptop instead of the one for the Jetson. If you’re running Linux in a VM, connect your ​host​ computer to the router using the instructions below. Depending on which VM software you have and the default VM configuration, you may also need to set its network adapter configuration to NAT mode. This ensures your VM will share the wireless connection with your host OS instead of controlling the adapter itself.

Windows
^^^^^^^^
These instructions are for Windows 10, but they should be easily replicable on older Windows versions as well.

#. Click the wireless icon at the bottom right of the taskbar, select the f110 network, and click the Connect button. Enter the network password when prompted.
#. Right-click the same wireless icon and click “Open Network & Internet settings.” Click “Change connection properties” in the window that pops up.
#. Scroll down, and under “IP settings,” hit the Edit button. Change “Automatic (DHCP)” to manual, click the IPv4 slider, and enter the IP address, gateway, and DNS server as described previously.

		* “Subnet prefix” should be set to ​24​, not ​255.255.255.0​ as you did with the Jetson.
		* You can leave “Alternate DNS” blank.
		* Remember to use the correct IP address for your computer; it should be different from the one you used on the car.)
#. If successful, the yellow exclamation mark on the wireless icon should go away. You can test connectivity using the ​ping​ utility included with the Windows command prompt.

Mac OS
^^^^^^^^
Coming Soon

SSHing into the Car
-------------------------------------------
The ​ssh​ utility is useful for gaining terminal access to your car when you don’t have a monitor around and when you don’t need to do visualization (e.g. via rviz​). Using this utility will give you the ability to edit and run your ROS code remotely and is especially useful when you want to rapidly develop and test new algorithms without the hassle a monitor can bring.

Before doing this, make sure both your laptop and car are connected to the f110 network as described ​here​.

#. Open a terminal on your laptop and type $ ​ssh your car’s IP address​ to connect to the car. You will be prompted for your Jetson login password; type this in as well.

	* The first time you SSH into the car, you will probably be told that the “authenticity of the host can’t be established.” Just type in “yes” and the dialog will not appear again.
#. If successful, you should see a prompt similar to ​

	.. code-block:: bash

		ubuntu@tegra-ubuntu:~$​, 

	which indicates that you’re now connected to the car’s terminal. Try starting ​ roscore​ and running some ROS scripts. Don’t forget to source your working directory’s setup file beforehand.
#. Don’t forget that while you’re SSH’ed into the car, you’re running over the wireless network. Try not to get too far away from the car so you don’t accidentally get logged out, and make sure you ​save your work often​.

Setting Up Wireless Hot Spot on Jetson
-------------------------------------------
As you begin to test on larger tracks, you may find a need to have a direct connection to your car, so as to not have to rely on the car being within a certain distance of your router. The solution here is to set up wireless hot spot on the Jetson. It is extremely easy.

Go to System Settings on your Jetson. Then Network.

.. image:: img/wireless1.jpg

On the bottom center of the pop-up window for the network, click on “Use as Hotspot...” You will no longer have internet connection because your wireless antennas will now be used as a hot spot rather than to connect to the previous Wi-Fi connection that you were on.

Note that if you plan on using the wireless hotspot feature often, you will want it to boot up on startup. To do this, open up Network Connections, under Wi-Fi select Hotspot and Edit.

.. image:: img/wireless2.jpg

Under General click on “Automatically connect to this network when available”.

On your phone, tablet, or laptop you can now connect directly to this Hotspot, and you can use it with VNC viewer as well if you have set up a VNC server. The default IP address for Hotspot on the Jetson is 10.42.0.1.

Setting Up VNC Server on Jetson
-------------------------------------------
(This is not essential, just useful if you feel strongly about having a GUI-type of desktop)

Setting up a VNC server on the Jetson allows you to control the Jetson remotely. Why is this beneficial? When the car is running in the real world we won’t be able to connect the Jetson to an HDMI display. The traditional solution has been to ssh into the Jetson to see the directories, but what if we want to see graphical programs such as Rviz? (in order to see laser scans in live time and camera feeds). Or what if we want to be able to see multiple terminal windows open on the Jetson? A VNC server does this trick.

Here are sequential instructions to install x11vnc and set it up so it loads every time at boot up, taken from ​http://c-nergy.be/blog/?p=10426​ . The article linked contains a link to a shell file to launch all these instructions. We have just pasted it here in case the original article or its link become inaccessible.

.. code-block:: bash

	#​ ​##################################################################
	#​ Script Name : vnc-startup.sh
	#​ Description : Perform an automated install of X11Vnc
	#​ Configure it to run at startup of the machine
	#​ Date : Feb 2016
	#​ Written by : Griffon
	#​ Web Site :http://www.c-nergy.be - http://www.c-nergy.be/blog
	#​ Version : 1.0
	#
	#​ Disclaimer : Script provided AS IS. Use it at your own risk....
	#
	#​ ​#################################################################

	#​ Step 1 - Install X11VNC
	#​ ​#################################################################

	sudo apt-get install x11vnc -y

	#​ Step 2 - Specify Password to be used ​for​ VNC Connection
	#​ ​#################################################################

	sudo x11vnc -storepasswd /etc/x11vnc.pass

	#​ Step 3 - Create the Service Unit File
	#​ ​#################################################################

	cat > /lib/systemd/system/x11vnc.service << EOF
	[Unit]
	Description=Start x11vnc at startup.
	After=multi-user.target

	[Service]
	Type=simple
	ExecStart=/usr/bin/x11vnc -auth guess -forever -loop -noxdamage -repeat
	-rfbauth /etc/x11vnc.pass -rfbport 5900 -shared

	[Install]
	WantedBy=multi-user.target
	EOF

	#​ Step 4 -Configure the Service
	#​ ​################################################################

	echo "Configure Services"
	sudo systemctl enable x11vnc.service
	sudo systemctl daemon-reload
	sleep  5s
	sudo shutdown -r now

Note that if you want to change the port that the VNC server lives on, then simply change 5900 to some other number. The article states that if 5900 is used on the Jetson, then the VNC server will automatically forward through 5901. And if that is taken, then 5902, and so on and so forth.

To connect to your VNC server, use a VNC viewer. A free one that works pretty well is ​ Real VNC’s VNC Viewer​. If you are on a mac, you can also use the included Screen Sharing app. Connect by typing into the url [jetson’s ip address]:[port number]. So for instance, if the jetson is connected on ip address 192.168.2.9 with port number 5900, then type in 192.168.2.9:5900.

Lastly, you will want to use an HDMI emulator, like this one, in order to trick the Jetson to thinking that a display is connected so that it will display at higher resolutions by running the GPU. Otherwise, if the Jetson is booted up with nothing connected into the HDMI port, the VNC server will default to a really low resolution, like 640 x 480. There is probably also an OS way to configure this, but it’s a lot easier to buy a $10 piece that solves the issue by hardware.

Note that there are existing softwares to be able to set up VNC servers as well, such as Real VNC. However, we found that these could not install on the Jetson TX2 because it uses an AARM64 processor. That is why we had to use x11vnc.