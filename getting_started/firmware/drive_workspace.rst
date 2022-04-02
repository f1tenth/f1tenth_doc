.. _doc_drive_workspace:

3. F1TENTH Driver Stack Setup
=====================
**Equipment Required:**
	* Fully built F1TENTH  vehicle
	* Pit/Host computer OR
	* External monitor/display, HDMI cable, keyboard, mouse

**Approximate Time Investment:** 1.5 hour

Overview
----------
We use Docker to containerize the software stack. You can find a tutorial on basic Docker concepts `here <https://docs.docker.com/get-started/>`_. We'll also utilize `nvidia-docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html>`_ to use the GPU onboard inside containers. Both of these dependencies should already come with JetPack after you've flashed your Jetson.

We use ROS 2 Foxy for communication and run the car. You can find a tutorial on ROS 2 `here <https://docs.ros.org/en/foxy/Tutorials.html>`_.

In the following section, we'll go over how to set up the **drivers** for sensors and the motor control:

#. Setting up :ref:`udev rules <udev_rules>` for our sensors.
#. Setting up docker :ref:`containers and the software stack<software_stack>`.
#. Launch :ref:`teleoperation and the LiDAR <teleop_setup>`.

.. We'll need to set up the :ref:`ROS workspace <ros_workspace>`, set up some :ref:`udev rules <udev_rules>`, and :ref:`test the lidar connection <lidar_setup>`.

Everything in this section is done on the **Jetson NX** so you will need to connect to it via SSH from the Pit laptop or plug in the monitor, keyboard, and mouse.

.. _udev_rules:

1. udev Rules Setup
----------------------
When you connect the VESC and a USB lidar to the Jetson, the operating system will assign them device names of the form ``/dev/ttyACMx``, where ``x`` is a number that depends on the order in which they were plugged in. For example, if you plug in the lidar before you plug in the VESC, the lidar will be assigned the name ``/dev/ttyACM0``, and the VESC will be assigned ``/dev/ttyACM1``. This is a problem, as the car’s configuration needs to know which device names the lidar and VESC are assigned, and these can vary every time we reboot the Jetson, depending on the order in which the devices are initialized.

Fortunately, Linux has a utility named udev that allows us to assign each device a “virtual” name based on its vendor and product IDs. For example, if we plug a USB device in and its vendor ID matches the ID for Hokuyo laser scanners (15d1), udev could assign the device the name ``/dev/sensors/hokuyo`` instead of the more generic ``/dev/ttyACMx``. This allows our configuration scripts to refer to things like ``/dev/sensors/hokuyo`` and ``/dev/sensors/vesc``, which do not depend on the order in which the devices were initialized. We will use udev to assign persistent device names to the lidar, VESC, and joypad by creating three configuration files (“rules”) in the directory ``/etc/udev/rules.d``.

First, **as root**, open ``/etc/udev/rules.d/99-hokuyo.rules`` in a text editor to create a new rules file for the Hokuyo. Copy the following rule exactly as it appears below in a single line and save it:

.. code-block:: bash

	KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", SYMLINK+="sensors/hokuyo"

Next, open ``/etc/udev/rules.d/99-vesc.rules`` and copy in the following rule for the VESC:

.. code-block:: bash

	KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"

Then open ``/etc/udev/rules.d/99-joypad-f710.rules`` and add this rule for the joypad:

.. code-block:: bash

	KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"

Finally, trigger (activate) the rules by running

.. code-block:: bash

	sudo udevadm control --reload-rules
	sudo udevadm trigger

Reboot your system, and you should find three new devices by running

.. code-block:: bash

	ls /dev/sensors

and:

.. code-block:: bash

	ls /dev/input

If you want to add additional devices and don’t know their vendor or product IDs, you can use the command

.. code-block:: bash

	sudo udevadm info --name=<your_device_name> --attribute-walk

making sure to replace ``<your_device_name>`` with the name of your device (e.g. ttyACM0 if that’s what the OS assigned it. The Unix utility dmesg can help you find that). The topmost entry will be the entry for your device; lower entries are for the device’s parents.

.. .. _ros_workspace:

.. 1. Setting Up the ROS Workspace
.. ---------------------------------
.. Connect to the **Jetson NX** either via SSH on the **Pit** laptop or a wired connection (monitor, keyboard, mouse).

.. On the **Jetson NX**, setup your ROS workspace (for the driver nodes onboard the vehicle) by opening a terminal window and following these steps.

.. #. Clone the following repository into a folder on your computer.

.. 	.. code-block:: bash

.. 		$​ ​cd​ ~/sandbox (or whatever folder you want to work ​in​)
.. 		$​ git ​clone​ https://github.com/f1tenth/f1tenth_system

.. #. Create a workspace folder if you haven’t already, here called ``f1tenth_ws``, and copy the ``f1tenth_system`` folder into it.

.. 	.. code-block:: bash

.. 		$​ mkdir -p f1tenth_ws/src
.. 		$​ cp -r f1tenth_system f1tenth_ws/src/

.. #. You might need to install some additional ROS packages.

.. 	For ROS Kinetic:

.. 		.. code-block:: bash

.. 			$​ sudo apt-get update
.. 			$​ sudo apt-get install ros-kinetic-driver-base

.. 	For ROS Melodic:

.. 		.. code-block:: bash

.. 			$​ sudo apt-get update
.. 			$​ sudo apt-get install ros-melodic-driver-base

.. #. Make all the Python scripts executable.

.. 	.. code-block:: bash

.. 		$​ ​cd​ f1tenth_ws
.. 		$​ find . -name “*.py” -exec chmod +x {} \;

.. #. Move to your workspace folder and compile the code (catkin_make does more than code compilation - see online reference).

.. 	.. code-block:: bash

.. 		$​ catkin_make

.. #. Finally, source your working directory into your shell using

.. 	.. code-block:: bash

.. 		$​ source devel/setup.bash

.. ..
.. 	Workspace Content Breakdown
.. 	^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. 	Examine the contents of your workspace and you will see 3 folders. In the ROS world we call these **meta-packages** since they contain package.

.. 		* algorithms
.. 		* simulator
.. 		* system

.. 	#. Algorithms contains the brains of the car which run high level algorithms, such as wall following, pure pursuit, localization.
.. 	#. Simulator contains racecar-simulator which is based off of MIT Racecar’s repository and includes some new worlds such as Levine 2nd floor loop. Simulator also contains f1_10_sim which contains some message types useful for passing drive parameters data from the algorithm nodes to the VESC nodes that drive the car.
.. 	#. System contains code from MIT Racecar that the car would not be able to work without. For instance, System contains ackermann_msgs (for Ackermann steering), racecar (which contains parameters for max speed, sensor IP addresses, and teleoperation), serial (for USB serial communication with VESC), and vesc (written by MIT for VESC to work with the racecar).

.. 	We will be focusing on the **System** folder in this section. :ref:`Going Forward <doc_going_forward_intro>` will utilize the firsit two folders - **Algorithms** and **Simulator**.

.. _software_stack:
2. Setting up the Driver Stack inside a Container
-------------------------------------------------------

First, clone the repo in a convenient place (we'll be using the home directory)

.. code-block:: bash

	cd
	git clone https://github.com/f1tenth/f1tenth_system.git

Next, run the script ``run_container.sh`` in the ``scripts`` directory in the repo. This pulls the pre-built image built for the car with ROS 2, and will create a persistent container on your car.

.. code-block:: bash

	cd f1tenth_system/scripts
	./run_contaianer.sh

You can find more details on how the drivers are set up in the README of the `f1tenth_system repo <https://github.com/f1tenth/f1tenth_system>`_.

If you need multiple bash sessions into the container, you can use ``tmux``, which is included in the docker image. See `a quick start guide on tmux <https://tmuxcheatsheet.com/>`_.

Starting the container will also bind mount a ROS 2 workspace ``f1tenth_ws`` created in the car's home directory. 

.. _teleop_setup:

3. Launching Teleop and Testing the LiDAR
----------------------
This section assumes that the lidar has already been plugged in (either to the USB hub or to the ethernet port). If you are using the Hokuyo 10LX or a lidar that is connected via the ethernet port of the Orbitty, make sure that you have completed the :ref:`Hokuyo 10LX Ethernet Connection <doc_firmware_hokuyo10>` section before preceding.

Before the bringup launch, you'll have to set the correct parameters according to which LiDAR you're using in the params file ``sensors.yaml``. Depending on how you've set up docker, you might need root access to write to files in ``f1tenth_ws`` since it's shared between the host and the container. All parameter files are located in the following location on your host:

.. code-block:: bash
	
	$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/

And

.. code-block:: bash

	/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/

In the container.

A. If you're using an ethernet based LiDAR, set the ``ip_address`` field to the corresponding ip address of your LiDAR.

B. If you're using a USB based LiDAR, comment out the ``ip_address`` field, and uncomment the line with the ``serial_port`` field. And set the value to the correct udev name from :ref:`udev rules set up <udev_rules>`.

In your running container, run the following commands to source the ROS 2 underlay and our workspace's overlay:

.. code-block:: bash

	source /opt/ros/foxy/setup.bash
	source /f1tenth_ws/install/setup.bash

Then, you can launch the bring up with:

.. code-block:: bash

	ros2 launch f1tenth_stack bringup_launch.py

Running the bringup launch will start the VESC drivers, the LiDAR drivers, the joystick drivers, and all necessary packages for running the car. To see the LaserScan messages, in a new bash session inside the container, run

.. code-block:: bash

	source /opt/ros/foxy/setup.bash
	source /f1tenth_ws/install/setup.bash
	rviz2

The rviz window should show up. Then you can add a LaserScan visualization in rviz on the ``/scan`` topic.

.. Once you’ve set up the lidar, you can test it using urg_node/hokuyo_node (replace the hokuyo_node by the urg_node if you have 10LX with Ethernet connection: https://github.com/ros-drivers/urg_node.git), rviz, and rostopic.

.. A. If you're using the 10LX:

.. 	* Start ``roscore​`` in a terminal window.
.. 	* In another (new) terminal window, run ``rosrun urg_node urg_node _ip_address:="192.168.0.10"​``. Make sure to supply the urg node with the correct port number for the 10LX.
.. 	* This tells ROS to start reading from the lidar and publishing on the ​/scan​ topic. If you get an error saying that there is an “error connecting to Hokuyo,” double check that the Hokuyo is physically plugged into a USB port. You can use the terminal command ``lsusb​to`` check whether Linux successfully detected your lidar. If the node started and is publishing correctly, you should be able to use ``rostopic echo /scan​`` to see live lidar data.
.. 	* In the racecar config folder under ``lidar_node`` set the following parameter in sensors.yaml: ``ip_address: 192.168.0.10``. In addition in the ``sensors.launch.xml`` change the argument for the lidar launch from ``hokuyo_node`` to ``urg_node`` do the same thing for the ``node_type`` parameter.

.. B. If you're using the 30LX:

.. 	* Run ``roslaunch racecar teleop.launch`` in a sourced terminal window, by default, the launch file brings up the hokuyo node.

.. Once your lidar driver node is running, open another terminal and run ``rosrun rviz rviz​`` or simply ``rviz`` to visually see the data. When ``rviz​`` opens, click the “Add” button at the lower left corner. A dialog will pop up; from here, click the *By topic* tab, highlight the *LaserScan* topic, and click *OK*. You might have to switch from viewing in the ``\map`` frame to the ``laser`` frame. If the laser frame is not there, you can type in ``laser`` in the frame text field.

.. ``rviz`` will now show a collection of points of the lidar data in the gray grid in the center of the screen. You might have to change the size and color of the points in the LaserScan setting to see the points clearer.

.. 	* Try moving a flat object, such as a book, in front of the lidar and to its sides. You should see a corresponding flat line of points on the ​rviz​ grid.
.. 	* Try picking the car up and moving it around, and note how the lidar scan data changes,

.. You can also see the lidar data in text form by using ​``rostopic echo /scan`` ​. The type of message published to it is sensor_msgs/LaserScan​, which you can also see by running ``rostopic info /scan​`` . There are many fields in this message type, but for our course, the most important one is ​ranges​, which is a list of distances the sensor records in order as it sweeps from its rightmost position to its leftmost position.

.. With all of the parts connected now, we can move on to driving with a joystick!

.. .. image:: img/drive01.gif
.. 	:align: center
.. 	:width: 200pt
