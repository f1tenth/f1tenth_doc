.. _doc_drive_workspace:
.. _lidar_setup:
.. _doc_firmware_hokuyo10:

2. RoboRacer Driver Stack Setup
=================================
**Equipment Required:**
	* Fully built RoboRacer vehicle
	* Pit/Host computer OR
	* External monitor/display, HDMI cable, keyboard, mouse

**Approximate Time Investment:** 1.5 - 2 hours

.. warning:: **Before you proceed**, this section sets up the driver stack **natively** for Jetson Xavier and above running **JetPack 5.0 or newer** (Ubuntu 20.04+) with ROS 2. For JetPack versions below 5.0 and Jetsons before Xavier, go to :ref:`Driver Stack Setup with Docker Containers <doc_drive_workspace_docker>` and follow the instructions there instead.

By the end of this section you will have ROS 2 installed, udev rules in place for the sensors, the RoboRacer driver stack built, your LiDAR and joypad configured, and the car brought up for the first time. Everything is done **on the Jetson**, so you'll need to connect to it via SSH from the Pit laptop or plug in the monitor, keyboard, and mouse.

**The ROS 2 distribution you install is decided by the Ubuntu version your JetPack image is built on** — you do not get to choose the two independently. Find your row in the table below, then follow **only** that page. Each one is a complete, self-contained procedure covering the same seven steps in the same order, so you should never need to switch between them.

.. list-table::
   :header-rows: 1
   :widths: 18 24 16 42

   * - JetPack
     - Ubuntu
     - ROS 2
     - Follow this page
   * - 7.x (L4T R39)
     - 24.04 "Noble"
     - **Jazzy**
     - :ref:`1. Ubuntu 24.04 - ROS 2 Jazzy <doc_drive_workspace_jazzy>`
   * - 5.x - 6.x
     - 20.04 / 22.04
     - **Humble**
     - :ref:`2. Ubuntu 22.04 - ROS 2 Humble <doc_drive_workspace_humble>`
   * - before 5.0
     - 18.04
     - Foxy
     - :ref:`3. Driver Stack Setup with Docker Containers <doc_drive_workspace_docker>`

Check which one you have with:

.. code-block:: bash

    lsb_release -a

.. toctree::
   :maxdepth: 1
   :name: Driver Stack Setup
   :hidden:

   drive_workspace_jazzy
   drive_workspace_humble

.. note:: **LiDAR support is the same on both pages.** Step 5 of whichever page you follow covers the **USB Hokuyo** (e.g. the 30LX, which needs nothing beyond the udev rule in step 3), the **Hokuyo 10LX over ethernet**, and the **SICK ethernet LiDAR** (e.g. the TiM5xx). The procedure is identical apart from the name of the driver package — ``ros-jazzy-sick-scan-xd`` or ``ros-humble-sick-scan-xd``.

.. note:: **Why the Jazzy path needs an extra patch.**

    The ROS buildfarm only ever produced Jammy (22.04) packages for Humble, and that will not change — Humble reaches end of life in May 2027 without ever targeting Noble. On Ubuntu 24.04 the corresponding distribution is Jazzy.

    ``f1tenth_system``, however, has no Jazzy branch. The repository offers ``foxy-devel``, ``humble-devel``, ``melodic`` and ``braking``, and its ``vesc`` and ``ackermann_mux`` submodules stop at ``humble`` and ``master`` respectively. The procedure on the Jazzy page is therefore to **clone** ``humble-devel`` **and patch it**.

    Every apt dependency the stack needs — ``sick_scan_xd``, ``ackermann_msgs``, ``joy``, ``joy_teleop``, ``rosbridge_suite``, ``serial_driver``, ``asio_cmake_module``, ``diagnostic_updater`` — is already released for Jazzy, so only the source packages need modifying, and only one of those changes is substantive.

.. tip:: **On Ubuntu 24.04, consider Docker instead.**

    If your car needs to interoperate with other RoboRacer machines in your lab, or you plan to use ``f1tenth_gym_ros`` or the ``f1tenth_labs`` packages, those are Humble-targeted as well and you will end up patching each one. Running a ``ros:humble-ros-base`` container on the same Noble host gives you the stock, documented stack and parity with everyone else's cars. The native Jazzy port is the better choice if the vehicle is standalone.

You can find ROS 2 tutorials for `Humble <https://docs.ros.org/en/humble/Tutorials.html>`__ and `Jazzy <https://docs.ros.org/en/jazzy/Tutorials.html>`__ in the official documentation.
