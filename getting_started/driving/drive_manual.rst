.. _drive_manualcontrol:

Manual Control
=================
**Equipment Required:**
	* Fully built F1TENTH  vehicle
	* Pit/Host computer
	* Logitech F710 joypad

**Approximate Time Investment:** 30 minutes - ∞ There is no time limit on fun!

Overview
------------
Before we can get the car to drive itself, it’s a good idea to test the car to make sure it can successfully drive on the ground under human control. Controlling the car manually is also a good idea if you’ve recently re-tuned the VESC or swapped out a drivetrain component, such as the motor or gears. Doing this step early can spare you a headache debugging your code later since you will be able to rule out lower-level hardware issues if your code doesn’t work.

**You MUST connect to the TX2 via SSH for this section.**

1. Vehicle Inspection
-----------------------
We want to minimize the number of accidents so before we begin, let's first inspect our vehicle.

#. Make sure you have the car running off its LIPO battery.
#. Plug the USB dongle receiver of the **Logitech Joypad** into the **USB hub**.
#. Make sure you have the VESC connected.
#. Ensure that both your car and laptop are connected to a wireless access point if you need the car connected to the Internet while you drive it. Otherwise, go back and go through :ref:`System Configuration <doc_software_setup>`.
#. Make sure you’ve cloned the ``f110_system`` repository and set up your working directory as explained in the :ref:`previous section <doc_drive_workspace>`.
#. This section uses the program ``tmux`` (available via apt-get) to let you run multiple terminals over one SSH connection. You can also use VNC​ if you prefer a GUI.

2. Driving the Car
----------------------
#. Open a terminal on the **Pit** laptop and SSH into the car from your computer.
#. Once you’re in, open a terminal window and run ​``$tmux`` so that you can spawn new terminal sessions over the same SSH connection.
#. In your tmux session, spawn a new window (using ``Ctrl-B`` and then ``C``) and run ​``$roscore``​ to start ROS.
#. Navigate to other free terminal using ``Ctrl-B`` and then ``P`` or ``N`` by switch to previous or next session, or using ``Ctrl-B`` and then the number of the session, navigate to your workspace that we set up before, run ``$ catkin make`` and source the directory using ``$ source devel/setup.bash``.
#. Run ``$ roslaunch racecar teleop.launch​`` to launch the car. 
	* If you see an error like this: ``[ERROR] [1541708274.096842680]: Couldn't open joystick force feedback!`` It means that the joystick is connected. 
	* If this gives you a segmentation error and it’s caused by compiling the joy package (which you can check by running the joy node on its own), this could be because you are using the joy package from the ROS distribution (i.e., installed with apt-get). Remove that by ``sudo apt-get remove joy`` and ``catkin_make`` in your workspace, and sourcing the setup bash again. This should compile and use the joy package that’s in the repo.

#. Hold the LB button on the controller to start controlling the car. Use the left joystick to move the car forward and backward and the right joystick for steering.
	
	* If nothing happens or if the right joystick is not mapped to steering, your might joystick might be in a different mode, press the *mode* button to change the mode.
	* If nothing happens, one reason can be that the ``joy_node`` is listening for inputs on the ``js0`` port, but the operating system has assigned a different port to it, like ``js1``. Edit the ``yaml`` file which specifies which port to listen to. You can tell what file that is by reading the launch file (and following the call tree to other launch files).
	* Note that the LB button acts as a “dead man’s switch,” as releasing it will stop the car. This is for safety in case your car gets out of control.
	* You can see a mapping of all controls used by the car in ``f110_system/racecar/racecar/config/racecar-v2/joy_teleop.yaml``. For example, in the default configuration, axis 1 (left joystick’s vertical axis) is used for throttle, and axis 2 (right joystick’s horizontal axis) is used for steering. If you need to check which axis correspond to what buttons/axis on the joystick, run the joy node, when the joystick is connected, you can see the index of the button/axis that's changed.

Troubleshooting
------------------
Here are some common errors:

* **VESC out of sync errors**: Check that the VESC is connected. If the error persists, make sure you're using the right VESC driver node. Currently, the ``vesc`` package in the f110_system repo only supports VESC 6+. If you have an older implementation of VESC (for example the FOCBox), use the repo `here <https://github.com/mit-racecar/vesc>`_ instead.
* **Serial port busy errors**: Your VESC might have just booted up, give it a few seconds and try again.
* **SerialException errors** ​and you’re using the 30LX Hokuyo​, the errors might be due to a port conflict: e.g., suppose that the lidar was assigned the (virtual serial bi-directional) port ``ttyACM0`` by the operating system. And suppose that the ``vesc_node`` is told the VESC is connected to port ``ttyACM0`` (as per ``vesc.yaml``). Then when the ``vesc_node`` receives joystick commands from ``joy_node`` (via ROS), it pushes them to ``ACM0`` - so these messages actually go to the lidar, and the VESC gets garbage back. So change the ``vesc.yaml`` port entry to ``ttyACM1``. (This whole discussion remains valid if you switch 0 and 1, i.e. if the OS assigned ACM1 to the lidar and your ``vesc.yaml`` lists ACM1). Note that everytime you power down and up, the OS will assign ports from scratch, which might again break your config files. So a better solution is to use udev rules, as explained in `this <firmware.html#udev-rules-setup>`_ section​. (See the source code of joy node for the default port for the joystick. You can over-ride that using a parameter in the launch file. See the joy documentation for what parameter that is).
* **urg_node related errors**: Check the ports (e.g. an ip address in sensors.yaml can only be used by 10LX, not 30LX, and vice-versa for the /dev/ttyACM​n​).
* **razor_imu errors**: Delete the IMU entry from the launch file - we’re not using an IMU in this build.

Congratulations on building the car, configuring the system, installing the firmware, and driving the car! You've come a long way. Pat yourself on the back and high five your other hand.

.. image:: img/drive02.gif
	:align: center
	:width: 400pt

