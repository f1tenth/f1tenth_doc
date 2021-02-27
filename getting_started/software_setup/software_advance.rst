.. _doc_software_advance:

Advanced Setups
===================
The following two sections are optional.

* :ref:`Wireless Hot Spot on the NVIDIA Jetson NX <jetson_wireless>` is relevant if you want to test on larger tracks and need to have a direct connection to your vehicle.
* :ref:`VNC Server on the NVIDIA Jetson NX <jetson_wireless>` is useful if you're running mapping and localization algorithms, you’ll need to see RViz and use its tools for some applications.

 .. _jetson_wireless:

Wireless Hot Spot on the NVIDIA Jetson NX
---------------------------------
**Equipment Used:**
	* Fully built F1TENTH vehicle
	* Pit/Host Laptop OR
	* External monitor/display, HDMI cable, keyboard, mouse

**Approximate Time Investment:** 30 minutes

As you begin to test on larger tracks, you may find a need to have a direct connection to your car, so as to not have to rely on the car being within a certain distance of your router. The solution here is to set up wireless hot spot on the NVIDIA Jetson NX.

Connect to the **NVIDIA Jetson NX** via SSH on the **Pit** laptop or via a monitor, keyboard, and mouse.

On the NVIDIA Jetson NX, go to System Settings > Network.

.. image:: img/combine/wireless1.jpg

On the bottom center of the pop-up window for the network, click on “Use as Hotspot...” You will no longer have internet connection because your wireless antennas will now be used as a hot spot rather than to connect to the previous Wi-Fi connection that you were on.

Note that if you plan on using the wireless hotspot feature often, you will want it to boot up on startup. To do this, open up Network Connections, under Wi-Fi select Hotspot and Edit.

.. image:: img/combine/wireless2.jpg

Under General click on “Automatically connect to this network when available”.

On your phone, tablet, or laptop you can now connect directly to this Hotspot, and ssh into it. You can use a VNC client as well if you have set up a VNC server on the car. The default IP address for Hotspot on the Jetson is 10.42.0.1.

 .. _jetson_vnc:

VNC Server on the NVIDIA Jetson NX
-------------------------
**Equipment Used:**
	* Fully built F1TENTH vehicle
	* Pit/Host Laptop OR
	* External monitor/display, HDMI cable, keyboard, mouse

**Approximate Time Investment:** 1 hour

When you start running mapping and localization algorithms, you'll need to see RViz and use its tools for some applications, meaning that you'll need a GUI interface for the remote desktop.

Setting up a VNC server on the Jetson allows you to control the Jetson remotely. Why is this beneficial? When the car is running in the real world we won’t be able to connect the Jetson to an HDMI display. The traditional solution has been to ssh into the Jetson to see the directories, but what if we want to see graphical programs such as Rviz? (in order to see laser scans in live time and camera feeds). Or what if we want to be able to see multiple terminal windows open on the Jetson? A VNC server does this trick.

#. Install XIIVNC

	.. code-block:: bash

		sudo apt install x11vnc
#. Create a password file

	.. code-block:: bash

		echo mypassword > /home/nvidia/.vnc/password

	Change this to your own password. You might have to create the .vnc directory
#. Press windows/command/super key and search for ‘startup applications’. Create a new startup command, give it a name, and the command is:

	.. code-block:: bash

		/usr/bin/x11vnc -auth guess -forever -loop -noxdamage -repeat -passwdfile /home/nvidia/.vnc/password -rfbport 5900 -shared
#. Restart the Jetson, and the vnc server should be running after you restart.
#. In your favorite VNC client (Ubuntu has Remmina installed by default, and VNC Viewer is available across most platforms), connect to your car's IP with port number 5900 to see the remote desktop. Note that the resolution of the car when it was booted without a monitor plugged in is low, you can plug in the HDMI Emulator included in the BOM or a working monitor to fix that.

.. note::
  We've had spotty experience with NVIDIA Jetson NX's network card, the hotspot sometimes just doesn't work. If your NVIDIA Jetson NX's network card is faulty, you can use a USB wifi dongle and use your network through the dongle instead of the NVIDIA Jetson NX network card.

Good work on making it through the advanced section!

.. image:: img/combine/wireless4.gif
	:align: center
	:width: 300px
