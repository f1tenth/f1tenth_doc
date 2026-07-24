.. _doc_autoware_intro:

Autoware on the RoboRacer Car
=============================

`Autoware <https://autoware.org/>`_ is an open-source autonomous driving stack built on ROS 2.
The `autoware.roboracer <https://github.com/mlab-upenn/autoware.roboracer>`_ project (based on
Autoware release 1.6.0, ROS 2 Humble) adds the interfaces, vehicle/sensor models, and configurations
needed to run Autoware on the **RoboRacer Off-Road** platform, with the **Jetson AGX Orin** as the
target compute.

This page is a **quickstart**: it walks through installing Autoware on the Jetson and running it on a
physical RoboRacer car. It is intentionally condensed.

.. note::
   For the full documentation, including x86 host installation, manual (non-scripted) setup,
   software- and hardware-in-the-loop simulation, map creation and calibration, the launch modes,
   architecture overview, and the track tuning guide, see the complete
   `autoware.roboracer documentation <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/Home.md>`_.

.. _doc_autoware_installation:

Installation (Jetson AGX Orin)
------------------------------

These steps install Autoware on a Jetson AGX Orin running **JetPack 6.2.1** on **Ubuntu 22.04**.
The approximate total time investment is roughly a day, most of it in the workspace build.
For the full guide, see
`Installation, Jetson AGX Orin <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/installation/jetson-agx-orin.md>`_.

1. **Flash JetPack 6.2.1** to the Jetson AGX Orin. The recommended method is the
   `NVIDIA SDK Manager <https://developer.nvidia.com/sdk-manager>`_ (be sure to also select
   *Jetson SDK Components*). See the
   `JetPack 6.2.1 documentation <https://developer.nvidia.com/embedded/jetpack-sdk-621>`_ for details.

2. **Install the ZED camera drivers and SDK.** The RoboRacer Off-Road car uses a ZED camera, so before
   proceeding install both the capture card driver and the ZED SDK, matching the versions to your card
   and JetPack version. Follow the
   `Stereolabs driver guide <https://www.stereolabs.com/docs/embedded/zed-link/install-the-drivers>`_
   and the `ZED SDK for Jetson guide <https://www.stereolabs.com/docs/development/zed-sdk/jetson>`_.

3. **Install the JetPack packages** (CUDA, cuDNN, and TensorRT), which Autoware requires, then reboot.

   .. code-block:: bash

      sudo apt update && sudo apt upgrade -y
      sudo apt install -y nvidia-jetpack
      sudo reboot

4. **Clone the repository and install the Autoware dependencies.** The ``setup-dev-env.sh`` script
   installs the system dependencies Autoware needs. The ``--jetson`` flag makes it use a
   Jetson-compatible OpenCV, build ``spconv`` and ``cumm`` from source, and skip the CUDA/NVIDIA
   drivers already provided by JetPack. Afterwards, install the cuDNN and TensorRT CMake modules.

   .. code-block:: bash

      cd ~
      git clone -b roboracer_humble https://github.com/mlab-upenn/autoware.roboracer.git autoware
      cd autoware
      ./setup-dev-env.sh --jetson
      sudo apt install -y ros-humble-cudnn-cmake-module ros-humble-tensorrt-cmake-module

5. **Set up the workspace.** Autoware uses
   `vcstool <https://github.com/dirk-thomas/vcstool>`_ to assemble the workspace. Create the ``src``
   directory and clone all of the required repositories into it.

   .. code-block:: bash

      cd ~/autoware
      mkdir -p src
      vcs import src < repositories/autoware.repos

6. **Install the ROS package dependencies** for everything in the workspace using ``rosdep``.

   .. code-block:: bash

      source /opt/ros/humble/setup.bash
      sudo apt update && sudo apt upgrade
      rosdep update
      rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

7. **Build the workspace.** Autoware uses `colcon <https://colcon.readthedocs.io/>`_ to build.

   .. code-block:: bash

      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

   Ignore the ``stderr`` warnings during the build.

   .. tip::
      Building Autoware is memory-hungry and can crash on systems with limited RAM. If the build
      fails, add 16 to 32 GB of swap before rebuilding:

      .. code-block:: bash

         sudo fallocate -l 32G /swapfile
         sudo chmod 600 /swapfile
         sudo mkswap /swapfile
         sudo swapon /swapfile
         sudo bash -c 'echo "/swapfile swap swap defaults 0 0" >> /etc/fstab'

8. **Configure the network and DDS settings** by following the official
   `Autoware DDS documentation <https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/>`_
   before running.

.. _doc_autoware_on_vehicle:

Running on the Car
------------------

This runs Autoware on the physical RoboRacer Off-Road car. It assumes you have
completed the installation above and have a **point cloud map and Lanelet2 map** for your environment
(see
`Map Creation & Calibration <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/map-creation.md>`_).
For the full guide, see
`On-Vehicle Operation <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/on-vehicle-operation.md>`_.

1. **Physical setup.** Power the Jetson and vehicle electronics, verify the ZED camera is recognized
   by the OS, and confirm the USB connection to the motor/servo controller is present (the
   ``roboracer_interface_node`` uses the f1tenth stack to talk to the hardware).

2. **Source the workspace** on the Jetson.

   .. code-block:: bash

      source /opt/ros/humble/setup.bash
      source ~/autoware/install/setup.bash

3. **(Optional) Test manual control** without launching Autoware. This brings up the vehicle
   interface and joystick teleop only, which is useful for map recording and hardware verification.
   Manual driving requires holding the dead-man button (RB) on the controller.

   .. code-block:: bash

      ros2 launch f1tenth_stack no_lidar_bringup.launch.py

4. **Launch Autoware.** Pick a launch mode for your use case (see
   `Launch Modes <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/launch-modes.md>`_
   for the full comparison), and set ``map_path`` to your map directory.

   Full stack, for real-world operation with obstacle avoidance:

   .. code-block:: bash

      ros2 launch offroad_launch autoware.launch.xml \
        vehicle_model:=roboracer_offroad \
        sensor_model:=roboracer_offroad_sensor_kit \
        map_path:=/path/to/your/map/ \
        launch_vehicle_interface:=true

   Minimal racing stack, for a closed circuit with the circuit planner and lowest overhead:

   .. code-block:: bash

      ros2 launch offroad_launch_minimal autoware_minimal.launch.xml \
        vehicle_model:=roboracer_offroad \
        sensor_model:=roboracer_offroad_sensor_kit \
        map_path:=/path/to/your/map/ \
        launch_vehicle_interface:=true

5. **Initialize localization.** By default the ``pose_initializer`` node uses the ZED's onboard pose
   estimate to initialize automatically. Otherwise, use the **2D Pose Estimate** tool in RViz to click
   the vehicle's approximate position and heading on the map.

6. **Set a goal and engage.** For the full and lite stacks, use the **2D Goal Pose** tool in RViz to
   set a destination; once the route is shown and localization is stable, press **Auto**. The minimal
   stack uses the circuit route planner, which needs no goal, so it drives laps continuously; just
   press **Auto** once localization initializes. Just as manual driving requires holding the RB
   dead-man button, autonomous driving requires holding a different dead-man button (LB) on the
   controller; releasing it stops the vehicle.

   .. warning::
      To stop the vehicle at any time, release the dead-man button on the controller, press the
      **Emergency Stop** button in RViz, or publish to ``/system/emergency/control_cmd``.

For hardware-in-the-loop simulation (Autoware on the Jetson driving the off-road simulator on a
separate x86 host), see
`Hardware-in-the-Loop Simulation <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/simulation/hardware-in-loop.md>`_.
