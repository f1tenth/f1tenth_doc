Linux/ROS Installation
============================

ROS only runs natively in Linux so we are only supporting using the simulator in Ubuntu at this time. If you do not have ROS Melodic installed, follow the instructions from `<http://wiki.ros.org/melodic/Installation/Ubuntu>`_. 

Dependencies
------------------
You will need the following dependences:
  
  - tf2_geometry_msgs
  - ackermann_msgs
  - joy
  - map_server

Install them using

.. code-block:: bash
  
  sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server

The full list of dependencies can be found in the ``package.xml`` file.

Package
------------
To install the simulator package, clone the simulator repository into your catkin workspace:

.. code-block:: bash

  cd ~/catkin_ws/src
  git clone https://github.com/f1tenth/f1tenth_simulator.git

Then run catkin_make to build it:

.. code-block:: bash

  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash

Quick Start
---------------
To run the simulator on its own, run:

.. code-block:: bash

  roslaunch f1tenth_simulator simulator.launch

This will launch everything you need for a full simulation: roscore, the simulator, a preselected map, a model of the racecar, and the joystick server.

.. figure:: img/sim_install.png
  :align: center

  Full simulation launched.

