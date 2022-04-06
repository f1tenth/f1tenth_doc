.. _doc_drive_autonomous:

Autonomous Control
=====================

.. note:: This section assumes that you have already completed :ref:`Building the Car <doc_build_car>`, :ref:`System Configuration <doc_software_setup>`, :ref:`Installing Driver Stack <doc_build_car_firmware>`, :ref:`Manual Control <drive_manualcontrol>`, and :ref:`Odometry Calibration <doc_calib_odom>`.

This section goes through how to subscribe to sensor topics, and how to publish drive topic to control the car.

**Required Equipment:**
    * Fully built F1TENTH vehicle
    * Pit/Host computer
    * Logitech F710 joypad

**Difficulty Level:** Intermediate

**Approximate Time Investment:** 1 hour

Sensor Topics
---------------
#. ``/scan``: this topic maintains the ``LaserScan`` messages published by the LiDAR.
#. ``/odom``: this topic maintains the ``Odometry`` messages published by the VESC.

Drive Topic
---------------
``/drive``: this topic is listened to by the VESC, needs ``AckermannDriveStamped`` messages. The ``speed`` and ``steering_angle`` fields in the ``drive`` field of these messages are used to command desired steering and velocity to the car.

Developing your own Node for Autonomous Control
--------------------------------------------------
Since we're using a docker container, there are two options when it comes to where to put your new custom node. In the following section, we'll go over all the options in detail.

1. Developing directly in the driver stack container.

2. Create your own docker container.