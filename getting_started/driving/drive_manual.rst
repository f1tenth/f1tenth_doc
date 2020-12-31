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
#. Make sure you’ve cloned the ``f1tenth_system`` repository and set up your working directory as explained in the :ref:`previous section <doc_drive_workspace>`.
#. This section uses the program ``tmux`` (available via ``apt-get``) to let you run multiple terminals over one SSH connection. You can also use VNC if you prefer a GUI.

2. Driving the Car
----------------------
#. Open a terminal on the **Pit** laptop and SSH into the car from your computer.
#. Once you’re logged in, open a terminal window and run ``tmux`` so that you can spawn new terminal sessions over the same SSH connection.
#. In your ``tmux`` session, spawn a new window (using ``Ctrl-B`` and then ``C``) and run ``roscore`` to start ROS.
#. Navigate to other free terminal using ``Ctrl-B`` and then ``P`` or ``N`` to switch to the previous or next session, or using ``Ctrl-B`` and then the number of the session.
#. Navigate to your workspace that we set up before.
#. Run ``catkin_make`` and source the directory using ``source devel/setup.bash``.
#. Run ``roslaunch f1tenth_racecar teleop.launch`` to launch the nodes for tele-operation.
	* If you see a warning like this: ``[WARN] [1541708274.096842680]: Couldn't open joystick force feedback!``, it means that the joystick is connected. 
#. Hold the LB button on the controller to start controlling the car. Move the left joystick up and down to move the car forward and backward and to the right and left to steer.
	* If nothing happens or if the right joystick is not mapped to steering, your joystick might be in a different mode. Press the *mode* button to change the mode.
	* Note that the LB button acts as a “dead man’s switch,” as releasing it will stop the car. This is for safety in case your car gets out of control.
	* You can see a mapping of all controls used by the car in ``f1tenth_system/f1tenth_racecar/config/joy_teleop.yaml``.
          For example, in the default configuration, axis 1 (left joystick’s vertical axis) is used for throttle, and axis 2 (left joystick’s horizontal axis) is used for steering.
          If you need to check which axis correspond to what buttons/axis on the joystick, run ``rostopic echo /vesc/joy`` and watch which axes change when you move the controls on the joystick.

Troubleshooting
------------------
Here are some common errors:

* **VESC out of sync errors**: Check that the VESC is connected.
If the error persists, make sure you're using the right VESC driver node.
Currently, the ``vesc`` package in the ROS repositories only supports VESC 6+.
If you have an older implementation of VESC (for example the FOCBox), clone the repo `here <https://github.com/mit-racecar/vesc>`_ into your workspace and build it again to use this version.
* **Serial port busy errors**: Your VESC might have just booted up, give it a few seconds and try again.
* **SerialException errors** when using the Hokuyo UTM-30LX and not using ``udev`` rules: These errors might be due to a port conflict: e.g., suppose that the lidar was assigned the (virtual serial bi-directional) port ``ttyACM0`` by the operating system and suppose that the ``vesc_node`` is told the VESC is connected to port ``ttyACM0`` (as per ``vesc.yaml``).
When the ``vesc_node`` receives joystick commands from ``joy_node`` via ROS, it pushes them to ``ACM0`` - so these messages actually go to the lidar, and the VESC gets garbage back.
To resolve this, change the ``vesc.yaml`` port entry to ``ttyACM1``.
Note that every time you power-down and -up, the OS will assign ports from scratch, which might again break your config files.
This is why using ``udev`` rules as explained in `this <firmware.html#udev-rules-setup>`_ section is recommended.
* **urg_node-related errors**: Check the ports (e.g. an ip address in ``sensors.yaml`` can only be used by 10LX, not 30LX, and vice-versa for the serial port).

Congratulations on building the car, configuring the system, installing the firmware, and driving the car!
You've come a long way. Pat yourself on the back and high five your other hand.
You can head over to `Learn <https://f1tenth.org/learn.html>`_ and try out some of the labs there.

.. image:: img/drive02.gif
	:align: center
	:width: 300px

