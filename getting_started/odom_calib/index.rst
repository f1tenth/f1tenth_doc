.. _doc_drive:

Calibrating the Odometry
=========================
.. note:: This section assumes that you have already completed :ref:`Building the Car <doc_build_car>`, :ref:`System Configuration <doc_software_setup>`, :ref:`Installing Firmware <doc_build_car_firmware>`, and :ref:`Driving the Car <doc_drive>`.

One final step.

**Required Equipment:**
	* Fully built F1TENTH vehicle 
	* Pit/Host computer
	* Logitech F710 joypad
	* Tape measure

**Difficulty Level:** Intermediate

**Approximate Time Investment:** 1 hour

Now that everything is built, configured, and installed, the odometry of the vehicle needs to be calibrated. The VESC receives input velocities in m/s and steering angles in radians. However the motor and servo requires commands in revolution per minute (RPM) and servo positions. The conversion parameters will need to be tuned to your specific car.

#. The parameters in `vesc.yaml <https://github.com/f1tenth/f1tenth_system/blob/master/racecar/racecar/config/racecar-v2/vesc.yaml>`_ need to be calibrated.

#. Follow this great `Tuning Guide <https://mushr.io/tutorials/tuning/>`_ that `Mushr <https://mushr.io/about/>`_ put together.

.. tip:: 
  If you have any build and/or setup questions, post to the `forum <http://f1tenth.org/forum.html>`_.