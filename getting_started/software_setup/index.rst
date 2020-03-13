.. _doc_software_setup:

System Configuration
=====================

Now that you have a working physical F1TENTH car, you'll want to program it to do something useful, otherwise it's just a glorified and expensive car model at this point.

The image below represents the flow of information on the F1TENTH Autonomous Vehicle System.

.. image:: img/f1tenth_sys_flow.png

The **TX2** is the main brain of the entire system. It gives commands to the **VESC** which controls the **Servo** and the **Brushless Motor**. The **TX2** also receives information from the **Lidar**. The **Pit** laptop is where we can connect remotely to the **TX2**. 

.. note:: 
  If you have any build and/or setup questions, post to the `forum <http://f1tenth.org/forum.html>`_.

This System Configuration section will walk you through how to configure the TX2 so that you will be able to run programs on the F1TENTH Autonomous Vehicle System and communicate with the Vehicle.

This section has four subsections:

.. toctree::
   :maxdepth: 1
   :name: Software Setup

   software_jetson
   software_host
   software_combine
   software_vesc

#. :ref:`Jetson setup <doc_software_jetson>` goes over how to set up the Jetson TX2.
#. :ref:`Host setup <doc_software_host>` details how to install ROS and the simulators on your laptop. 
#. :ref:`Combine setup <doc_software_combine>` goes over how to communicate between your host laptop and the TX2 once you have already completed the above two sections.
#. :ref:`Configuring the VESC <doc_software_vesc>` goes over how to tune the VESC.