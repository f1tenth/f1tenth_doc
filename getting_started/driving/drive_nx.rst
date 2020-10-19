.. _doc_drive_nx:

NX Setup
=====================
**Equipment Required:**
	* Fully built F1TENTH  vehicle
	* Pit/Host computer OR
	* External monitor/display, HDMI cable, keyboard, mouse

**Approximate Time Investment:** 1 hour

Overview
----------
We use ROS to connect everything together and ultimately run the car. We'll need to set up the :ref:`ROS workspace <ros_workspace>`, set up some :ref:`udev rules <udev_rules>`, and :ref:`test the lidar connection <lidar_setup>`. Everything in this section is done on the **TX2** so you will need to connect to it via SSH from the **Pit** laptop or plug in the monitor, keyboard, and mouse.

These instructions are specific to setting up the software on the Jetson Xavier NX as the setup is a bit different than the TX2. Many thanks for `Jim from JetsonHacks <https://www.jetsonhacks.com/>`_ for figuring this out.

#. Download the SD card image for JetPack (currently version 4.4) to your host machine: `https://developer.nvidia.com/embedded/jetpack <https://developer.nvidia.com/embedded/jetpack>`_

#. Flash the card, many people use Etcher: `https://www.balena.io/etcher/ <https://www.balena.io/etcher/>`_

#. Boot and configure the Jetson with the SD card.

#. You will need to install the Logitech F710 driver on the Jetson.

    .. code-block:: bash

        $ git clone https://github.com/jetsonhacks/logitech-f710-module
        $ cd logitech-f710-module
        $ ./install-module.sh

#. Install ROS

    .. code-block::  bash

        $ cd ~
        $ git clone https://github.com/jetsonhacks/installROS
        $ cd installROS
        $ ./installROS -p ros-melodic-drive-base
        $ ./setupCatkinWorkspace.sh f1tenth_ws

    (This will setup a catkin workspace in the home directory named f1tenth_ws)

#. We are now ready to install the F1/Tenth ROS packages
    
    .. code-block::  bash

        $ cd ~/f1tenth_ws/src
        $ git ​clone​ https://github.com/f1tenth/f1tenth_system
        $ find . -name “*.py” -exec chmod +x {} \;
        $ source devel/setup.bash
        $ rosdep install -a -y -r
        $ catkin_make
..
   The rosdep command will download all of the package dependencies. I don't recall if rosdep handled the ros-melodic-drive-base correctly, that's why I put that command in the installROS.sh step. I think it does (it's probably at least worth testing). The less intimidating the better.
