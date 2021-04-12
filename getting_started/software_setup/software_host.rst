.. _doc_software_host:

1. Pit/Host Setup
==================
**Equipment Used:**
	* Laptop/Computer

**Approximate Time Investment:** 1-2 hours

Overview
----------

The F1TENTH vehicle runs on a Linux operating system. You will need to have Linux and ROS (Robot Operating System) on your **Pit** laptop. This Laptop is used as your main developer environment as well as a debug support. We will assume that you already have some basic knowledge of Linux and ROS as explaining either systems in depth is beyond the scope of this documentation. If you are not familiar with Linux we can recommend you the following links:

	* `Introduction to Linux Operating System (OS) <https://www.guru99.com/introduction-linux.html>`_
	* `The Linux Command Line for beginners <https://ubuntu.com/tutorials/command-line-for-beginners#1-overview>`_

.. important:: We currently support **Ubuntu Xenial 16.04/ROS Kinetic** and **Bionic 18.04/ROS Melodic**.

We will refer to the **Pit** computer as **Pit** or **Host** computer/laptop interchangeably.

In this **Configure F1TENTH System** section, the **Pit** laptop is first used to flash the NVIDIA JetPack Software onto the **NVIDIA Jetson NX**. Afterwards we use the laptop to SSH into the **NVIDIA Jetson NX**.

1. Installing Ubuntu
---------------------
If you do not already have Linux running natively on your laptop, we suggestion you dual boot or install Ubuntu on an external hard drive. You can download the Ubuntu image `here <https://ubuntu.com/download/desktop>`_. Instructions for installing Ubuntu can be found `here <https://ubuntu.com/tutorials/tutorial-install-ubuntu-desktop#1-overview>`_.

If you use a Virtual Machine (VM) for your Mac or Windows Laptop instead, see :ref:`Appendix A <doc_appendix_a>` on sharing folders.

Installing Ubuntu may take a while so you may want to get a cup of coffee or tea and settle in.

2. Installing ROS
------------------
Afterwards we are installing ROS on your Host Laptop. You can follow the instructions `here <https://wiki.ros.org/ROS/Installation>`_ to install the supported ROS versions Kinetic or Melodic displayed above for your Ubuntu System. We can recommend to install the **-desktop-full** version so you have everything you need on your Host Laptop.

If you have never used ROS before, ROS has many in-depth and useful tutorials which can be found here. `here <https://wiki.ros.org/ROS/Tutorials>`_ that you may want to try after installing.

With a Linux operating system on the **Pit/Host** computer, you're ready to move on to setting up the **NVIDIA Jetson NX** on your F1TENTH car.

.. image:: img/host/host01.gif
	:align: center
	:width: 300px
