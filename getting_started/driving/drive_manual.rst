.. _drive_manualcontrol:

Manual Control
=================
**Equipment Required:**
	* Fully built RoboRacer  vehicle
	* Pit/Host computer
	* Logitech F710 joypad, or another supported controller

**Approximate Time Investment:** 30 minutes - ∞ There is no time limit on fun!

Overview
------------
Before we can get the car to drive itself, it’s a good idea to test the car to make sure it can successfully drive on the ground under human control. Controlling the car manually is also a good idea if you’ve recently re-tuned the VESC or swapped out a drivetrain component, such as the motor or gears. Doing this step early can spare you a headache debugging your code later since you will be able to rule out lower-level hardware issues if your code doesn’t work.

**You MUST connect to the Jetson via SSH or Remote Desktop for this section.**

1. Vehicle Inspection
-----------------------
We want to minimize the number of accidents so before we begin, let's first inspect our vehicle.

#. Make sure you have the car running off its LIPO battery.
#. Plug the USB dongle receiver of the **joypad** into the **USB hub**.
#. Make sure you have the VESC connected.
#. Ensure that both your car and laptop are connected to a wireless access point if you need the car connected to the Internet while you drive it. Otherwise, go back and go through :ref:`Configure Jetson and Peripherals <doc_software_setup>`.
#. Make sure you’ve cloned the ``f1tenth_system`` repository and built the workspace as explained in the :ref:`previous section <doc_drive_workspace>`, or set up your container as explained in the :ref:`Docker section <doc_drive_workspace_docker>`.
#. This section uses the program ``tmux`` (available via apt-get) to let you run multiple terminals over one SSH connection, and multiple terminals inside the container. You can also use the remote desktop if you prefer a GUI.

Finding the Jetson IP Address
-------------------------------
Before you can SSH into the car, you need the Jetson's IP address. Your laptop and the Jetson must be on the **same network**.

* **From the Jetson** (via a monitor or Remote Desktop), run ``hostname -I`` — the first address it prints is the Jetson's IP.
* **From your laptop**, run ``arp -a`` and look for an unfamiliar IP address that appears after powering on the car.

Then connect with ``ssh <username>@<jetson_ip>`` (for example, ``ssh tristan@172.16.61.134``).

.. note:: If your car has a SICK ethernet LiDAR, do **not** use ``192.168.0.15`` here — that address is on the isolated LiDAR subnet and is not reachable from your laptop. Use the Jetson's WiFi address.

2. Before the Wheels Touch the Ground
---------------------------------------
.. warning:: **Keep the car on a stand until teleoperation is verified.** A mis-set speed scale or a reversed axis can send the car into a wall faster than you can release the deadman switch.

Bring up the driver stack following :ref:`Launching the Driver Stack <doc_drive_workspace>`, then open a second terminal and watch the teleop command topic while you work through the checklist:

.. code-block:: bash

    ros2 topic echo /teleop

#. ``/teleop`` publishes **only** while the manual deadman button is held, and stops immediately on release.
#. Pushing the left stick forward produces a **positive** speed value. If it is negative, flip the sign on the ``drive-speed`` scale in ``joy_teleop.yaml``.
#. Pushing the right stick right steers the wheels right.
#. The ``drive-speed`` scale is ``1.0``, not ``5.0``.

Only once all four hold should you move the car to the floor, and then raise the speed scale gradually.

.. note:: Remember to rebuild after changing anything in ``config/``. See the *Rebuild after editing any config or launch file* step of your :ref:`driver stack setup <doc_drive_workspace>`.

3. Driving the Car
----------------------
#. Open a terminal on the **Pit** laptop and SSH into the car from your computer.
#. Launch teleop following the instructions for launching and testing teleop and the LiDAR in either :ref:`driver stack setup <doc_drive_workspace>` or :ref:`driver stack setup inside a docker container <doc_drive_workspace_docker>` depending on your setup.
#. Hold the manual deadman button (LB on the Logitech F710) to start controlling the car. Use the left joystick to move the car forward and backward and the right joystick for steering.
#. If you're using a Logitech F710, switch the switch at the back of the joystick to **D**. The mode light in the front of the joystick should **not** be constantly on. If it is, press the mode button once.

.. note:: On the **8BitDo Ultimate 2C** the deadman is button **6** rather than button 4, which means ``joy_teleop.yaml`` has to be edited before teleop will respond at all. See step 6 of your :ref:`driver stack setup <doc_drive_workspace>` for the full mapping.

.. #. Run the ``run_container.sh`` script in the ``f1tenth_system`` repo to start the Docker container.
.. #. Inside the bash session inside the container, run ``tmux`` and spawn several new windows by using ``ctrl+b`` then ``c`` multiple times. You can navigate through these windows with ``ctrl+b`` then ``p`` or ``n``. This is one way to add and navigate through windows, you can also check the tmux cheatsheet for creating and navigating panes, and using mouse mode. You can always create more windows if you need. These will come in handy when you need to run more than one node, or launch more than one launch file.
.. #. In one bash session, first source the ROS 2 underlay with ``source /opt/ros/foxy/setup.bash``. Then, make sure you're in our ROS 2 workspace ``/f1tenth_ws`` and run ``colcon build`` to build the workspace. Then source the workspace overlay with ``source install/setup.bash``.
.. #. Lastly, run ``ros2 launch f1tenth_stack bringup_launch.py`` to bring up the RoboRacer driver stack.
.. 	* If you see an error like this: ``[ERROR] [1541708274.096842680]: Couldn't open joystick force feedback!`` It means that the joystick is connected and you can ignore the error.

Troubleshooting
------------------

Teleoperation does nothing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Walk the command chain in order. The first silent link is the fault:

.. code-block:: bash

    ros2 topic echo /joy               # gamepad -> ROS
    ros2 topic echo /teleop            # joy_teleop -> mux (requires the deadman held)
    ros2 topic echo /ackermann_drive   # mux -> VESC

If ``/joy`` updates but ``/teleop`` stays silent, the **deadman index is wrong** — recheck it against ``/joy``, counting from zero. If both are silent while the gamepad works in ``jstest``, ``joy_node`` has opened a different device.

The joystick is not mapped correctly
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Change the mapping in ``f1tenth_stack/config/joy_teleop.yaml``. To identify the mapping, launch the bringup and echo the ``/joy`` topic. Move a joystick axis around and watch which entry of the array changes — the index of that entry is the axis id for that joystick direction. After identifying the correct indices, change the yaml to reflect it under ``human_control``, and rebuild.

You can see a **mapping of all controls** used by the car in ``joy_teleop.yaml``. In the default configuration, axis 1 (the left joystick's vertical axis) is used for throttle and axis 2 (the right joystick's horizontal axis) for steering. These differ from joystick to joystick.

The gamepad is not detected at all
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Confirm the kernel sees the device before blaming ROS:

.. code-block:: bash

    ls -l /dev/input/          # expect js0 and event* entries
    sudo apt install -y joystick
    jstest /dev/input/js0

If ``jstest`` responds but ``/joy`` does not, the problem is ROS-side. If neither responds, check the controller's mode — on the 8BitDo Ultimate 2C, hold **Start + X** for a few seconds to force XInput mode, then replug the dongle. Switch mode in particular does not enumerate cleanly on Linux.

.. warning:: ``jstest`` and ``joy_node`` numbering can disagree, because ``joy`` uses SDL2 while ``jstest`` reads raw joydev. **Always take indices from** ``ros2 topic echo /joy``.

On ROS 2 Humble, if nothing happens at all, one reason can be that the driver is listening on the wrong port for the joystick. Check ``joy_teleop.yaml`` for the ``device_name`` parameter — if you're using the Logitech joypad, the name should match the udev name set up earlier. If you're using another joystick and did not set udev rules, you can check the assigned name by running ``ls /dev/input/*``; it usually follows the format ``/dev/input/js*``.

Other common problems
^^^^^^^^^^^^^^^^^^^^^^^
* Note that the **deadman button acts as a "dead man's switch",** as releasing it will stop the car. This is for safety in case your car gets out of control.
* **Motor rotation direction negated.** If your car is driving backwards when commanded to drive forward, move to the :ref:`next section <doc_calib_odom>` to see how to reverse it.
* **Steering off-center or reversed.** This is calibration, not gamepad mapping. See ``config/vesc.yaml``: ``steering_angle_to_servo_offset`` (default ``0.5304``) and ``steering_angle_to_servo_gain`` (default ``-1.2135``) are per-vehicle values. ``speed_to_erpm_gain`` (``4614.0``) likewise depends on your motor and gearing.
* **VESC out of sync errors**: Check that the VESC is connected. If the error persists, make sure you're using the right VESC firmware.
* **Serial port busy errors**: Your VESC might have just booted up, give it a few seconds and try again.
* **SerialException errors** and you're using the 30LX Hokuyo, the errors might be due to a port conflict: make sure you've set up udev rules, as explained in the :ref:`driver stack setup <doc_drive_workspace>`.
* **urg_node related errors**: Check the ports (e.g. an ip address in ``sensors.yaml`` can only be used by 10LX, not 30LX, and vice-versa for the udev name ``/dev/sensors/hokuyo``).
* **Configuration edits appear to have no effect**: you did not rebuild. Run ``colcon build --packages-select f1tenth_stack`` and confirm the change landed in ``install/``.

.. Congratulations on building the car, configuring the system, installing the firmware, and driving the car! You've come a long way. Pat yourself on the back and high five your other hand. You can head over to `Learn <https://roboracer.ai/learn.html>`_ and try out some of the labs there.

.. .. image:: img/drive02.gif
.. 	:align: center
.. 	:width: 300px
