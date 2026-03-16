.. _drive_manualcontrol:

Manual Control
=================
**Equipment Required:**
	* Fully built RoboRacer  vehicle
	* Pit/Host computer
	* Logitech F710 joypad

**Approximate Time Investment:** 30 minutes - ∞ There is no time limit on fun!

Overview
------------
Before we can get the car to drive itself, it’s a good idea to test the car to make sure it can successfully drive on the ground under human control. Controlling the car manually is also a good idea if you’ve recently re-tuned the VESC or swapped out a drivetrain component, such as the motor or gears. Doing this step early can spare you a headache debugging your code later since you will be able to rule out lower-level hardware issues if your code doesn’t work.

**You MUST connect to the Jetson via SSH or Remote Desktop for this section.**

1. Vehicle Inspection
-----------------------
Before beginning manual control, inspect the vehicle to reduce the chance of unexpected behavior or hardware issues.

#. Make sure the car is powered using its LiPo battery.
#. Plug the USB dongle receiver for the **Logitech F710 joypad** into the **USB hub** on the car.
#. Make sure the **VESC** is connected properly.
#. If Internet access is required while driving, ensure that both the car and the laptop are connected to the same network. Otherwise, go back and complete :ref:`Configure Jetson and Peripherals <doc_software_setup>`.
#. Make sure the ``f1tenth_system`` repository has already been set up in the workspace and that the required Docker container has been configured as explained in the :ref:`previous section <doc_drive_workspace>`.
#. This section uses ``tmux`` to let you run multiple terminals over one SSH connection and multiple terminals inside the container. You may also use Remote Desktop if you prefer a GUI.

Finding the Jetson IP Address
-------------------------------

Before connecting to the car through SSH, you need to determine the IP address of the Jetson.

From the Jetson Terminal
^^^^^^^^^^^^^^^^^^^^^^^^^

If you have access to the Jetson terminal, either directly through a monitor or through Remote Desktop, run:

``hostname -I``

Example output:

``172.16.61.134``

This number is the Jetson IP address.

You can now connect from your laptop using:

``ssh <username>@<jetson_ip>``

Example:

``ssh tristan@172.16.61.134``

From Your Laptop
^^^^^^^^^^^^^^^^^^

If the Jetson is connected to the same network as your laptop through WiFi or Ethernet, you can also try identifying its IP address from your laptop.

Run:

``arp -a``

This command lists devices currently connected to the network. Look for an unfamiliar IP address that appeared after powering on the car.

Notes
^^^^^^^^

* Your laptop and the Jetson must be connected to the **same network** for SSH to work.
* The first time you connect, SSH may ask:

  ``Are you sure you want to continue connecting (yes/no)?``

  Type ``yes`` and press Enter.
* When entering the password, **no characters will appear on the screen**. This is normal terminal behavior.

2. Driving the Car
----------------------

#. Open a terminal on the **Pit / Host** laptop and SSH into the Jetson.

   Example:

   ``ssh <username>@<jetson_ip>``

#. Go to the workspace:

   ``cd ~/f1tenth_ws``

#. Source the workspace:

   ``source install/setup.bash``

#. Verify that the joystick is detected:

   ``ls /dev/input/js*``

   If the controller is connected correctly, you should see something like:

   ``/dev/input/js0``

#. Launch the RoboRacer driver stack:

   ``ros2 launch f1tenth_stack bringup_launch.py``

#. Before testing on the ground, lift the drive wheels off the ground and verify that the controls respond correctly.

#. Hold the **LB** button on the controller to enable manual control. The **LB** button acts as the dead man's switch. Use the **left joystick** to move the car forward and backward and the **right joystick** to steer.


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

.. Congratulations on building the car, configuring the system, installing the firmware, and driving the car! You've come a long way. Pat yourself on the back and high five your other hand. You can head over to `Learn <https://roboracer.ai/learn.html>`_ and try out some of the labs there.

.. .. image:: img/drive02.gif
.. 	:align: center
.. 	:width: 300px

