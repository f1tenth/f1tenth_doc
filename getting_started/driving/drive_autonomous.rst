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
* ``/scan``: this topic maintains the ``LaserScan`` messages published by the LiDAR.
* ``/odom``: this topic maintains the ``Odometry`` messages published by the VESC.
* ``/

Drive Topic
---------------
* ``/drive``: this topic is listened to by the VESC, needs ``AckermannDriveStamped`` messages. The ``speed`` and ``steering_angle`` fields in the ``drive`` field of these messages are used to command desired steering and velocity to the car.

Developing your own Node for Autonomous Control
--------------------------------------------------
Since we're using a docker container, there are two options when it comes to where to put your new custom node. In the following section, we'll go over all the options in detail.

1. **Developing directly in the driver stack container.** This is the most straightforward approach since most of the dependencies are already set up for you. When you create a new package alongside ``f1tenth_system``, you'll need to define your dependencies in ``package.xml`` (and ``CMakeLists.txt`` if you're using C++). Then, use rosdep to install your dependencies with the following commands:

.. code-block:: bash
    
    cd /f1tenth_ws
    rosdep install -i --from-path src --rosdistro foxy -y

This will install the dependencies you declared in ``package.xml`` from all the packages in the ``src`` directory in your workspace. Then, run ``colcon build`` to build your packages. After you've added your custom package, you can either create your own launch file to launch your nodes, or add to the bringup launch file we provided to launch your nodes.

2. **Create your own docker container.** This is the more portable solution when you need to put your code on another car.
If you're an advanced user of Docker, the recommended way to do this is to create your own Dockerfile with ROS 2 included (you can use ROS's official image). If you **need GPU access** in your custom Dockerfile, you can use the image we provided (``f1tenth/focal-l4t-foxy:f1tenth-stack``) as the parent image. The containers (your custom container and the driver stack container) will be put on the same network by default by docker, so communication through ROS 2 should work automatically.