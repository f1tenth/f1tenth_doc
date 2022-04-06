.. _doc_calib_odom:

Calibrating the Odometry
=========================
.. note:: This section assumes that you have already completed :ref:`Building the Car <doc_build_car>`, :ref:`System Configuration <doc_software_setup>`, :ref:`Installing Driver Stack <doc_build_car_firmware>`, and :ref:`Manual Control <drive_manualcontrol>`.

One final step that's crucial to get an accurate estimate of the car's current velocity, and accurate localization and mapping later on is to calibrate the odometry estimation. On the F1TENTH vehicle, the odometry is estimated from the motor's ERPM and the current angle of the servo.

**Required Equipment:**
	* Fully built F1TENTH vehicle
	* Pit/Host computer
	* Logitech F710 joypad
	* Tape measure
	* Tape

**Difficulty Level:** Intermediate

**Approximate Time Investment:** 1 hour

Calibrating the steering and odometry
-----------------------------------------

Now that everything is built, configured, and installed, the odometry of the vehicle needs to be calibrated. The VESC receives input velocities in m/s and steering angles in radians. However the motor and servo requires commands in revolution per minute (RPM) and servo positions. The conversion parameters will need to be tuned to your specific car.

1. The parameters in `vesc.yaml <https://github.com/f1tenth/f1tenth_system/blob/foxy-devel/f1tenth_stack/config/vesc.yaml>`_ need to be calibrated. This yaml file is located at:

.. code-block:: bash

	$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml

on the host, and at:

.. code-block:: bash

	$/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml

in the container.

2. In the first few steps, make sure you've lifted the car up with a pit stand or a box, so the wheels can spin freely.

3. First, we need to check if our motor is rotating in the right direction. If when given a positive velocity, or commanded moving forward with the joystick, the motor is spinning in the reverse direction, **swap 2 of the 3 connections from the vesc to the BLDC motor**. Before you do this, **make sure you've disconnected the battery** to the vesc.

4. Next, we'll also need to check if the vesc driver is interpreting the motor rotation direction correctly. To do this, after launching teleop with the bringup launch, echo the ``/odom`` topic in a new bash window inside the container with this command: ``ros2 topic echo --no-arr /odom``. The ``--no-arr`` argument hides the large covariance matrices when echoing the odometry message. Give a positive velocity command (forward throttle with the joystick), and pay close attention to the ``pose/pose/position/x`` field of the echoed messages. If it is increasing in the reverse direction, we'll have to fix that. If it is reversed, modified the source code located at:

.. code-block:: bash

	$HOME/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp

in the host, and

.. code-block:: bash

	/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp

in the container.

Modify line ``100`` so it reads:

.. code-block:: cpp

	  double current_speed = -(-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;

After changing the source code, you'll have to go back to the workspace at ``/f1tenth_ws``, and call `colcon build` again. Also remember that whenever you rebuild the workspace, you'll have to call the commands to source the underlay and the overlay again.

.. note::
	In the following steps that adjust parameters in the yaml file, you'll have to call ``colcon build`` after every change before launching teleop again.

5. After the motor directions are fixed, and the odometry is in the right direction, we can now start tuning the steering and the odometry gains. The first to be tuned is the **Steering Offset**. This parameter will determine the offset we put on the servo, and whether the car can drive straight when given no steering input. Again, start teleop with the bringup launch. Drive the car in a straight line a couple times with no steering input for a couple meters, and see if it's driving straight. Adjust the ``steering_angle_to_servo_offset`` parameter in ``vesc.yaml`` if it's not. Make small adjustment everytime (in the magnitude of 0.1). Repeat until you find the correct offset so the car drives straight.

.. note::
	In the following steps, you'll need to lay down the tape measure straight on the ground. It is helpful to tape the tape measure so it doesn't move around.

6. Next, we'll tune the **Steering Gain**. This parameter determines the smallest turning radius of the car. We'll determine the desired turning radius given the maximum steering angle of the car we're setting, which is ``0.36 radians`` in both directions. The corresponding turning radius could be then calculated with :math:`R=L/(2\sin(\beta))`, where :math:`L` is the wheelbase of the car, which is around 0.33 meters; :math:`\beta` is calculated as :math:`\arctan(0.5\tan(\delta))`, with :math:`\delta` being the maximum steering angle of the car. After calculations, when turning a half circle, the desired diameter of the half circle should be ``1.784 meters``.

* Place the car at the 0 of the tape measure such that the rear axle of the car is parallel to the tape measure, and the x-axis (roll axis) of the car coincides with the tape measure at 0.
* Start teleop. Set the steering angle to the maximum to one side depending on your setup (e.g. if the rest of the tape measure is on the left side of the car, turn left).
* Hold the steering, and drive forward slowly and steadily until the car runs over the tape measure again and the rear axle realigns with the tape measure. Now the car should be in the opposite direction to where you started.
* Take a measurement on the tape measure. The goal is 1.784 meters. If the measurement overshoots, increase the gain slightly (0.1 at a time). If it undershoots, decrease the gain. Repeat the process until you've hit 1.784 meters.
* If you notice that the wheels turn to different angles on the two directions when give maximum steering angles, adjust the ``servo_min`` and ``servo_max`` numbers until they're symmetric.

7. Next, we'll tune the **ERPM gain**. This parameter determines the conversion from desired velocity in meters/second to desired motor ERPM.

* Place the car at the 0 of the tape measure such that the rear axle of the car coincides with 0 and the x-axis (roll axis) of the car is parallel to the tape measure.
* Start teleop. Open another bash window in the container, and run ``ros2 topic echo --no-arr /odom``. We're particularly interested in the ``pose/pose/position/x`` value. Before giving any driving commands on the joystick, this value should be zero. If it is not zero, kill teleop and restart teleop.
* If you notice this value is drifting even when the car is stationary. Change the ``speed_to_erpm_offset`` value so that it stops drifting.
* Drive slowly and steadily forward without any steering input for more than 3 meters. Record the distance traveled by the car on the tape measure. Note that you'll have to take the reading from the rear axle.
* Compare the measured value to the value shown in the echoed message. If the distance reported by echo is larger, decrease the ``speed_to_erpm_gain`` value. Otherwise increase the gain. The change is usually on the order of thousands. Note that changing this value also changes the forward speed of teleop. Please drive carefully once the velocity is calibrated. If the forward speed when teleoping is too high, change the scale in ``human_control`` for ``drive-speed`` in ``joy_teleop.yaml``.
* Repeat the process until these values are within 2-3 cm.

Changing the software speed limit
--------------------------------------

If you wish to change the top speed of the car and has already followed the instructions to change the hardware limit in :ref:`the vesc firmware section <doc_firmware_vesc>`. All you'll need to do is also change the ``speed_min`` and ``speed_max`` values in ``vesc.yaml``. Note that the corresponding max speed in meters/second will be the max erpm value divided by the erpm gain. (e.g. ``speed_max/speed_to_erpm_gain``)

.. tip::
  If you have any build and/or setup questions, post to the `forum <https://f1tenth.discourse.group>`_.
