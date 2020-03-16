.. _doc_build_car_firmware:

Installing Firmware
====================
Now that the physical car is built and the software has been setup, we can start to installing the firmware needed on the car.

.. Tuning the FOCbox’s PID Gains
.. ------------------------------
.. In this section we use the words FOCbox and VESC interchangeably.

.. .. warning:: 
.. 	**Important Safety Tips**

.. 	* Make sure you hold on to the car while testing the motor to prevent it from flying off the stand.
.. 	* Make sure there are no objects (or people) in the vicinity of the wheels while testing.
.. 	* It’s a good idea to use a fully-charged LiPO battery instead of a power supply to ensure the motor has enough current to spin up.

.. #. Put your car on an elevated stand so that its wheels can turn without it going anywhere. If you don’t have an RC car stand, you can use the box that came with your Jetson.
.. #. Connect the host laptop to the FOCbox using a USB cable.
.. #. Download bldc tool from `JetsonHacks <https://github.com/jetsonhacks/installBLDC>`_, following his instructions for installation.
.. #. Open BLDC Tool and click the “Connect” button at the top right of the window to connect to the VESC.

.. 	* If you get the error “Device not found”, try running the command ​lsusb​ in a terminal. You should see an entry for “STMicroelectronics STMF407” or something similar. If you don’t, try unplugging and plugging in the USB cable on both ends. If the problem doesn’t go away, try rebooting the Jetson.

.. 	.. image:: img/tuning1.jpg

.. 	* If you are using a VESC 4.12 (including a FOCbox), ensure the firmware version is 2.18.

.. 	.. image:: img/tuning2.jpg

.. #. Disable keyboard control by clicking the “KB Ctrl” button at the lower right. This will prevent your keyboard’s arrow keys from controlling the motor and is important to prevent damage to the car from it moving unexpectedly.

.. 	.. image:: img/tuning3.jpg

.. #. Start plotting the realtime RPM data by clicking the “Realtime Data” tab, and checking the “Activate sampling” checkbox at the bottom left of the window. Click the “RPM” tab above the graph.

.. 	* We will keep referring to this plot of the motor’s RPM as we tune the PID gains. Out goal is to get the motor to spin up as quickly as possible when we set it to a certain RPM. We also don’t want the motor to cog (not spin) or overshoot the target speed if possible.

.. 	.. image:: img/tuning4.jpg

.. #. Test the motor first (without PID speed control) by setting the “Duty Cycle” to 0.20. This will spin the motor up to approximately 16,000 - 17,000 RPM. Let this run for a few seconds, and then press the “Release Motor” button at the bottom right to stop it.

.. 	* Observe the RPM graph. If the motor is spinning backwards (the RPM is negative), try reversing two of the connections from the VESC to the motor. (It doesn’t matter which wires you reverse.)
.. 	* If the wheels don’t spin and the motor makes no noise, check to make sure all connections to the motor are tight.
.. 	* If the wheels don’t spin and the motor does, ensure the motor’s gear is attached correctly to the gearbox at the back of the car. Spin both front wheels with your hand to verify that the gear is making good contact. You should feel some resistance when turning the wheels.
.. 	* If the motor doesn’t spin and makes a humming or hissing sound, you might need to replace the motor. If this doesn’t work, try replacing the VESC.

.. 	.. image:: img/tuning5.jpg

.. #. Click the “Motor Configuration” tab at the top and the “Advanced” tab on the left. Set Ki and Kd to 0.00000, and set Kp to 0.00001. Click the “Write Configuration” button at the bottom, go back to the data plotting tab and run the car at 3000 RPM.

.. 	* You will notice that the car won’t even make it close, as it only goes up to around 1200 RPM. (High steady-state error.)
.. 	* Try turning Kp up to 0.00002, 0.00004, and 0.00008. (Don’t forget to write the configuration each time.) The motor will start to cog out at higher Kp values.
	
.. 	.. image:: img/tuning6.jpg

.. #. Set Kp back to 0.00002, and set Ki to 0.00002, and run the car at 3000 RPM again. Notice how the car slowly reaches the 3000 RPM target. (This is because adding Ki helps to eliminate steady-state error.) Keep increasing Ki; set it to 0.00005 and then double that value a few times until the car is able to reach 3000 RPM without overshooting or cogging out.

.. #. Now, try increasing the speed to 6000 RPM. The motor might cog out and overshoot. If it does, try halving Kp.

.. #. Increase the speed to 10,000 RPM and then 20,000 RPM. ​Make sure you hold the car! If the motor cogs out and overshoots, halve Kp until it doesn’t. It may also help to halve Ki if halving Kp doesn’t work. If done correctly, the motor should not overshoot to more than 2 times the set RPM. (That is, if the RPM is set to 15,000, its peak value should not exceed 30,000.)


Hokuyo 10LX Ethernet Connection Setup
-----------------------------------------
.. Coming Soon: Add pictures and snippets.
.. note::
	If you have a 30LX or a LIDAR that connects via USB, you can skip this section.

In order to utilize the 10LX you must first configure the eth0 network. From the factory the 10LX is assigned the following ip: ``192.168.0.10``. Note that the lidar is on subnet 0.

First create a new wired connection.

In the ipv4 tab add a route such that the eth0 port on the Jetson is assigned ip address ``192.168.0.15``, the subnet mask is ``255.255.255.0``, and the gateway is ``192.168.0.1``. Call the connection Hokuyo. Save the connection and close the network configuration GUI.

When you plug in the 10LX make sure that the Hokuyo connection is selected. If everything is configured properly you should now be able to ping ``192.168.0.1``.

In the racecar config folder under ``lidar_node`` set the following parameter: ``ip_address: 192.168.0.10``. In addition in the ``sensors.launch.xml`` change the argument for the lidar launch from ``hokuyo_node`` to ``urg_node`` do the same thing for the ``node_type`` parameter.

Workspace Setup
--------------------------
On your host computer (e.g., your laptop), setup your ROS workspace (for the driver nodes onboard the vehicle) by following these steps.

Clone the following repository into a folder on your computer.

.. code-block:: bash

	$​ ​cd​ ~/sandbox (or whatever folder you want to work ​in​)
	$​ git ​clone​ https://github.com/f1tenth/f110_system

Create a workspace folder if you haven’t already, here called f110_ws, and copy the f110_system folder into it:

.. code-block:: bash

	$​ mkdir -p f110_ws/src
	$​ cp -r f110_system f110_ws/src/

You might need to install these packages. For ROS Kinetic:

.. code-block:: bash

	$​ sudo apt-get update
	$​ sudo apt-get install ros-kinetic-driver-base

For ROS Melodic:

.. code-block:: bash

	$​ sudo apt-get update
	$​ sudo apt-get install ros-melodic-driver-base


Make all the Python scripts executable.

.. code-block:: bash

	$​ ​cd​ f110_ws
	$​ find . -name “*.py” -exec chmod +x {} \;

Move to your workspace folder and compile the code (catkin_make does more than code compilation - see online reference).

.. code-block:: bash

	$​ catkin_make

Finally, source your working directory into your shell using

.. code-block:: bash

	$​ source devel/setup.bash

Congratulations! Your onboard driver workspace is all set up.

.. Now if you examine the contents of your workspace, you will see 3 folders (In the ROS world we call them meta-packages since they contain packages): algorithms, simulator, and system. Algorithms contains the brains of the car which run high level algorithms, such as wall following, pure pursuit, localization. Simulator contains racecar-simulator which is based off of MIT Racecar’s repository and includes some new worlds such as Levine 2nd floor loop. Simulator also contains f1_10_sim which contains some message types useful for passing drive parameters data from the algorithm nodes to the VESC nodes that drive the car.
.. Lastly, System contains code from MIT Racecar that the car would not be able to work without. For instance, System contains ackermann_msgs (for Ackermann steering), racecar (which contains parameters for max speed, sensor IP addresses, and teleoperation), serial (for USB serial communication with VESC), and vesc (written by MIT for VESC to work with the racecar).

Udev Rules Setup
-------------------
When you connect the VESC and a USB LIDAR to the Jetson, the operating system will assign them device names of the form ``/dev/ttyACMx``, where x is a number that depends on the order in which they were plugged in. For example, if you plug in the LIDAR before you plug in the VESC, the LIDAR will be assigned the name ``/dev/ttyACM0​``, and the VESC will be assigned ``/dev/ttyACM1​``. This is a problem, as the car’s ROS configuration scripts need to know which device names the LIDAR and VESC are assigned, and these can vary every time we reboot the Jetson, depending on the order in which the devices are initialized.

Fortunately, Linux has a utility named ​udev​ that allows us to assign each device a “virtual” name based on its vendor and product IDs. For example, if we plug a USB device in and its vendor ID matches the ID for Hokuyo laser scanners (15d1), ​udev​ could assign the device the name ``/dev/sensors/hokuyo`` instead of the more generic ``/dev/ttyACMx​``. This allows our configuration scripts to refer to things like ``/dev/sensors/hokuyo`` and ``/dev/sensors/vesc​``, which do not depend on the order in which the devices were initialized. We will use udev to assign persistent device names to the LIDAR, VESC, and joypad by creating three configuration files (“rules”) in the directory ``/etc/udev/rules.d``.

First, as root, open ``/etc/udev/rules.d/99-hokuyo.rules`` in a text editor to create a new rules file for the Hokuyo. Copy the following rule exactly as it appears below and save it:

.. code-block:: bash

	KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", SYMLINK+="sensors/hokuyo"

Next, open ``/etc/udev/rules.d/99-vesc.rules`` and copy in the following rule for the VESC:

.. code-block:: bash
	
	KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5749", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"

Then open ``/etc/udev/rules.d/99-joypad-f710.rules`` and add this rule for the joypad:

.. code-block:: bash

	KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"

Finally, trigger (activate) the rules by running

.. code-block:: bash

	$ sudo ​udevadm control --reload-rules
	$ sudo udevadm trigger​

Reboot your system, and you should find three new devices by running

.. code-block:: bash

	$ ls /dev/sensors
	$ hokuyo​    vesc

and:

.. code-block:: bash

	$ ls /dev/input
	$ joypad-f710​

If you want to add additional devices and don’t know their vendor or product IDs, you can use the command

.. code-block:: bash

	$ sudo ​udevadm info --name=<your_device_name> --attribute-walk

making sure to replace ``<your_device_name>`` with the name of your device (e.g. ttyACM0 if that’s what the OS assigned it. The Unix utility ​dmesg​ can help you find that). The topmost entry will be the entry for your device; lower entries are for the device’s parents.

Manual Control
-----------------
Before we can get the car to drive itself, it’s a good idea to test the car to make sure it can successfully drive on the ground under human control. Controlling the car manually is also a good idea if you’ve recently re-tuned the VESC or swapped out a drivetrain component, such as the motor or gears. Doing this step early can spare you a headache debugging your code later since you will be able to rule out lower-level hardware issues if your code doesn’t work.

Before you begin:

* Make sure you have the car running off its LIPO battery and that you have a Logitech F710 joypad handy with its receiver (i.e., USB dongle) plugged into the Jetson’s USB hub.
* Make sure you have the VESC connected!
* Ensure that both your car and laptop are connected to a wireless access point if you need the car connected to the Internet while you drive it. Otherwise, follow this tutorial​ so your laptop and phone can connect directly to the car.
* Make sure you’ve cloned the ``f110_system`` repository and set up your working directory (as explained ​here​)
* This tutorial uses the program ``tmux`` (available via apt-get) to let you run multiple terminals over one SSH connection. You can also use VNC​ if you prefer a GUI.

Now, we’re ready to begin.

#. Open a terminal and SSH into the car from your computer. Once you’re in, run ​tmux so that you can spawn new terminal sessions over the same SSH connection.
#. In your tmux session, spawn a new window (using ``Ctrl-B`` and then ``C``) and run ​roscore​ to start ROS.
#. Navigate to other free terminal using ``Ctrl-B`` and then ``P`` or ``N`` by switch to previous or next session, or using ``Ctrl-B`` and then the number of the session, navigate to your workspace that we set up before, run ``$ catkin make`` and source the directory using ``$ source devel/setup.bash``.
#. Run ``$ roslaunch racecar teleop.launch​`` to launch the car. If you see an error like this: ``[ERROR] [1541708274.096842680]: Couldn't open joystick force feedback!`` It means that the joystick is connected. If this gives you a segmentation error, and it’s caused by compiling the joy package (which you can check by running the joy node on its own), this could be because you are using the joy package from the ROS distribution (i.e., installed with apt-get). Remove that by ``sudo apt-get remove joy`` and ``catkin_make`` in your workspace, and sourcing the setup bash again. This should compile and use the joy package that’s in the repo.

#. Hold the LB button on the controller to start controlling the car. Use the left joystick to move the car forward and backward and the right joystick for steering.
	
	* If nothing happens or if the right joystick is not mapped to steering, your might joystick might be in a different mode, press the *mode* button to change the mode.
	* If nothing happens, one reason can be that the joy_node is listening for inputs on the js0 port, but the OS has assigned a different port to it, like js1. Edit the yaml file which specifies which port to listen to. You can tell what file that is by reading the launch file (and following the call tree to other launch files).
	* Note that the LB button acts as a “dead man’s switch,” as releasing it will stop the car. This is for safety in case your car gets out of control.
	* You can see a mapping of all controls used by the car in ``f110_system/racecar/racecar/config/racecar-v2/joy_teleop.yaml``. For example, in the default configuration, axis 1 (left joystick’s vertical axis) is used for throttle, and axis 2 (right joystick’s horizontal axis) is used for steering. If you need to check which axis correspond to what buttons/axis on the joystick, run the joy node, when the joystick is connected, you can see the index of the button/axis that's changed.

Troubleshooting
^^^^^^^^^^^^^^^^^^^
* If you’re getting "VESC out of sync errors", check that the VESC is connected. If the error persists, make sure you're using the right VESC driver node. Currently, the ``vesc`` package in the f110_system repo only supports VESC 6+. If you have an older implementation of VESC (for example the FOCBox), use the repo `here <https://github.com/mit-racecar/vesc>`_ instead.
* If you're getting "Serial port busy" error, your VESC might have just booted up, give it a few seconds and try again.
* If you get “SerialException” types of messages, ​and you’re using the 30LX Hokuyo​, the errors might be due to a port conflict: e.g., suppose that the lidar was assigned the (virtual serial bi-directional) port ttyACM0 by the OS. And suppose that the ``vesc_node`` is told the VESC is connected to port ttyACM0 (as per vesc.yaml). Then when the ``vesc_node`` receives joystick commands from ``joy_node`` (via ROS), it pushes them to ACM0 - so these messages actually go to the lidar, and the VESC gets garbage back. So change the ``vesc.yaml`` port entry to ``ttyACM1``. (This whole discussion remains valid if you switch 0 and 1, i.e. if the OS assigned ACM1 to the lidar and your ``vesc.yaml`` lists ACM1). Note that everytime you power down and up, the OS will assign ports from scratch, which might again break your config files. So a better solution is to use udev rules, as explained in `this <firmware.html#udev-rules-setup>`_ section​. (See the source code of joy node for the default port for the joystick. You can over-ride that using a parameter in the launch file. See the joy documentation for what parameter that is).
* If you get ``urg_node`` related error messages, check the ports (e.g. an ip address in sensors.yaml can only be used by 10LX, not 30LX, and vice-versa for the /dev/ttyACM​n​).
* If you get ``razor_imu`` errors, delete the IMU entry from the launch file - we’re not using an IMU in this build.

Tuning the VESC Parameters
------------------------------
You may want to fine tune your VESC parameters to match them to your car. Why? You might notice that your car with the default parameters drifts slightly to the side, or the odometry doesn't match what the car has driven. In order to tune your VESC parameters, navigate to ``f110_system/racecar/racecar/config/racecar-v2/vesc.yaml``. The vesc.yaml file is a configuration file where you can set parameters for erpm gain, steering angle offset, speed_min, speed_max, etc.

If you want to modify the maximum speed, under ``vesc_driver`` you can change the ``speed_min`` and ``speed_max``. These numbers represent the erpm of the car. By default they are set to +/- 23250 but you can set them higher. There is also a hard cap in the VESC firmware settings, you can open up VESC Tool and find the max settings for RPM. Your speed request you go through the setting in ``vesc.yaml`` first and capped by that setting, and then go through the setting in VESC firmware and capped again.

If your car’s motor is using a smaller or larger gear ratio, you will want to compensate for this by adjusting the ``speed_to_erpm_gain``. But generally, you would still want to adjust this value until your odometry is accurate. The ``speed_to_erpm_gain`` is used to convert requested m/s speed to requested motor RPM for the VESC. 

To tune the VESC odometry, first set up your car with a measuring tape as shown in **TODO PIC**. Start the racecar teleop to enable manual control. Note that you need to place the rear axle of the car at 0 before you start teleop since the odometry sets the location you start the car as zero. In a new terminal window, echo the ros topic ``/vesc/odom``. Drive staright along the tape measure for around 3 ~ 5 meters, and take a reading at the car's rear axle. Compare the reading from ROS echo for x-axis displacement and the measurement in real life. If the real life measurement is lower than the ROS echo reading, that means the ``speed_to_erpm_gain`` is too low, and vice versa. You should adjust this param until the two measurements are as close as possible. The tuning of this parameter is essential to having an accurate odometry.

If you notice that your car is not going straight, then you will want to modify your ``steering_angle_to_servo_offset``. By default the value is around 0.53, and you’ll want to increase or decrease this slightly until the car is going straight. You can drive the car with teleop straight ahead slowly without any steering input to see if there's any steering offset.

Testing the Lidar
---------------------------------
Once you’ve set up the LIDAR, you can test it using ​urg_node​/hokuyo_node, ​rviz​, and ​rostopic​.

#. Connect the LIDAR to the power board (see section ​Connecting the LIDAR​), and plug the USB cable into a free port on your hub if you're using a 30LX, or plug the ethernet cable into the TX2 if you're using a 10LX.
#. If you're using the 10LX:

	* Start ``roscore​`` in a terminal window. 
	* In another (new) terminal window, run ``rosrun urg_node urg_node​``. Make sure to supply the urg node with the correct port number for the 10LX.
	.. This tells ROS to start reading from the LIDAR and publishing on the ​/scan​ topic. If you get an error saying that there is an “error connecting to Hokuyo,” double check that the Hokuyo is physically plugged into a USB port. You can use the terminal command ``lsusb​to`` check whether Linux successfully detected your LiDAR. If the node started and is publishing correctly, you should be able to use ``rostopic echo /scan​`` to see live LIDAR data.

#. If you're using the 30LX:
	
	* Run ``roslaunch racecar teleop.launch`` in a sourced terminal window, by default, the launch file brings up the hokuyo node.

#. Once your LIDAR driver node is running, open another terminal and run ``rosrun rviz rviz​`` or simply ``rviz`` to visually see the data. When ``rviz​`` opens, click the “Add” button at the lower left corner. A dialog will pop up; from here, click the “By topic” tab, highlight the “LaserScan” topic, and click OK. You might have to switch from viewing in the ``\map`` frame to the ``laser`` frame. If the laser frame is not there, you can type in ``laser`` in the frame text field.
#. ``rviz`` will now show a collection of points of the LIDAR data in the gray grid in the center of the screen. You might have to change the size and color of the points in the LaserScan setting to see the points clearer.
	
	* Try moving a flat object, such as a book, in front of the LIDAR and to its sides. You should see a corresponding flat line of points on the ​rviz​ grid.
	* Try picking the car up and moving it around, and note how the LIDAR scan data changes,
#. You can also see the LIDAR data in text form by using ​``rostopic echo /scan`` ​. The type of message published to it is sensor_msgs/LaserScan​, which you can also see by running ``rostopic info /scan​`` . There are many fields in this message type, but for our course, the most important one is ​ranges​, which is a list of distances the sensor records in order as it sweeps from its rightmost position to its leftmost position.

Recording Bag Data on the Car
--------------------------------
ROSbags​ are useful for recording data from the car (e.g. LIDAR, odometry) and playing it back later. This feature is useful because it allows you to capture data from when the car is running and later study the data or perform analysis on it to help you develop and implement better racing algorithms.

One great thing about ROSbags compared to just recording the data into something simpler (like a CSV file) is that data is recorded along with the topics it was originally sent on. What this means is that when you later ​play​ the bag, the data will be transmitted on the same topics that it was originally sent on, ​and *any code that was listening to these topics can run, as if the data was being generated live​*.

For example, suppose I record LIDAR data being broadcasted on the ​/scan​ topic. When I later play the data back, the ``rostopic list`` and ``rostopic echo`` commands will show the LIDAR data being transmitted on the ​/scan​ topic as if the car was actually running.

Here’s a concrete example of how to use ROSbags to acquire motor telemetry data and play it back.

#. Make sure both your computer and car are connected to the ​same network. On your laptop, open a terminal and SSH into the car. Once you’re in, run ​tmux​ so that you can spawn new terminal sessions over the same SSH connection.
#. In your tmux session, spawn a new window and run ​``roscore​`` to start ROS.
#. In the other free terminal, navigate to your working directory, run ​catkin make​, and source the directory using ​source devel/setup.bash​.
#. Run ``roslaunch racecar teleop.launch​`` to launch the car. Place the car on the ground or on a stand and press the center button on your joystick so you can control the car.
#. In your tmux session, spawn a new window and examine the list of active ROS topics using ​rostopic list​. Make sure that you can see the ``/vesc/sensors/core​`` topic , which contains drive motor parameters.
#. Here’s where ROSbags come into play. Run ``rosbag record /vesc/sensors/core​`` to start recording the data. The data will start recording to a file in the current directory with naming format ``YYYY-MM-DD-HH-MM-SS.bag​`` . Recording will continue until you press Control-C to kill the rosbag process.

	* If you get an error about low disk space, you can specify the directory to record to (e.g. on a USB flash drive or hard drive) after the topic name). For example, ``rosbag record /vesc/sensors/core -o /path/to/external/drive`` to record into an external hard drive.
	* Note that ​rosbag​ also supports recording multiple topics at the same time. For example, I could record both laser scan and motor data using rosbag record ``/vesc/sensors/core /scan`` 

#. Let the recording run for about 30 seconds. Drive the car around during this time using the controller and then hit ``Ctrl-C`` to stop recording.
#. Play the rosbag file using ``rosbag play <your rosbag file>​``. While the bag is playing, examine the topics list, and you will see a list of all topics that were recorded into the bag. Note that in addition to the topics you specified, ROS will also record the ``rosout​``, ``rosout_agg``, and ``clock`` topics, which can be useful for debugging.
#. View that recorded motor data by echoing the ``/vesc/sensors/core​`` topic. Pay attention to how the motor RPM changed as you drove the car around. When the bag is out of data, it will stop publishing.


