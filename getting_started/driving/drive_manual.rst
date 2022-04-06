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

**You MUST connect to the Jetson via SSH or Remote Desktop for this section.**

1. Vehicle Inspection
-----------------------
We want to minimize the number of accidents so before we begin, let's first inspect our vehicle.

#. Make sure you have the car running off its LIPO battery.
#. Plug the USB dongle receiver of the **Logitech Joypad** into the **USB hub**.
#. Make sure you have the VESC connected.
#. Ensure that both your car and laptop are connected to a wireless access point if you need the car connected to the Internet while you drive it. Otherwise, go back and go through :ref:`Configure Jetson and Peripherals <doc_software_setup>`.
#. Make sure you’ve cloned the ``f110_system`` repository and set up your docker container as explained in the :ref:`previous section <doc_drive_workspace>`.
#. This section uses the program ``tmux`` (available via apt-get) to let you run multiple terminals over one SSH connection, and multiple terminals inside the container. You can also use the remote desktop if you prefer a GUI.

2. Driving the Car
----------------------
#. Open a terminal on the **Pit** laptop and SSH into the car from your computer.
#. Run the ``run_container.sh`` script in the ``f1tenth_system`` repo to start the Docker container.
#. Inside the bash session inside the container, run ``tmux`` and spawn several new windows by using ``ctrl+b`` then ``c`` multiple times. You can navigate through these windows with ``ctrl+b`` then ``p`` or ``n``. This is one way to add and navigate through windows, you can also check the tmux cheatsheet for creating and navigating panes, and using mouse mode. You can always create more windows if you need. These will come in handy when you need to run more than one node, or launch more than one launch file.
#. In one bash session, first source the ROS 2 underlay with ``source /opt/ros/foxy/setup.bash``. Then, make sure you're in our ROS 2 workspace ``/f1tenth_ws`` and run ``colcon build`` to build the workspace. Then source the workspace overlay with ``source install/setup.bash``.
#. Lastly, run ``ros2 launch f1tenth_stack bringup_launch.py`` to bring up the F1TENTH driver stack.
	* If you see an error like this: ``[ERROR] [1541708274.096842680]: Couldn't open joystick force feedback!`` It means that the joystick is connected and you can ignore the error.

#. Hold the LB button (Dead man's switch) on the controller to start controlling the car. Use the left joystick to move the car forward and backward and the right joystick for steering. If you're using Logitech F710, switch the switch at the back of the joystick to X. The mode light in the front of the joystick should **not** be constantly on. If it is, press the mode button once.

Troubleshooting
------------------

* During teleop, if the **joystick is not mapped correctly**, you can change the mapping in ``/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`` in the container. To identify the mapping, you can launch the bringup launch, and echo the ``/joy`` topic. Move the joystick axis around, you should change in the echoed message. The index of the changed value in the array when you move a joystick in on direction is the axis id to that joystick direction. After identifying the correct indices, change the yaml to reflect it under ``human_control``.
* If **nothing happens**, one reason can be that the driver name is listening on the wrong port for the joystick. To double check, you can check ``joy_teleop.yaml`` again for the `device_name` pararmeter. If you're using the Logitech joystick, the name should match with the udev name we've set up before. If you're using another joystick and did not set udev rules, you can check the assigned name by running ``ls /dev/input/*``. It'll usually follow the format ``/dev/input/js*``, for example ``/dev/input/js0``.
* Note that the **LB button acts as a “dead man’s switch”,** as releasing it will stop the car. This is for safety in case your car gets out of control.
* You can see a **mapping of all controls** used by the car in the ``joy_teleop.yaml`` file. For example, in the default configuration, axis 1 (left joystick’s vertical axis) is used for throttle, and axis 2 (right joystick’s horizontal axis) is used for steering. These might be different joystick to joystick.
* **Motor rotation direction negated.** If you're car is driving backwards when commanded driving forward, move to the :ref:`next section <doc_calib_odom>` to see how to reverse it.
* **VESC out of sync errors**: Check that the VESC is connected. If the error persists, make sure you're using the right VESC firmware.
* **Serial port busy errors**: Your VESC might have just booted up, give it a few seconds and try again.
* **SerialException errors** and you’re using the 30LX Hokuyo, the errors might be due to a port conflict: make sure you've set up udev rules, as explained in :ref:`this section <udev_rules>`.
* **urg_node related errors**: Check the ports (e.g. an ip address in ``sensors.yaml`` can only be used by 10LX, not 30LX, and vice-versa for the udev name ``/dev/sensors/hokuyo``).

.. Congratulations on building the car, configuring the system, installing the firmware, and driving the car! You've come a long way. Pat yourself on the back and high five your other hand. You can head over to `Learn <https://f1tenth.org/learn.html>`_ and try out some of the labs there.

.. .. image:: img/drive02.gif
.. 	:align: center
.. 	:width: 300px

