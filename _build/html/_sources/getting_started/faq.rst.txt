.. meta::
    :keywords: FAQ

.. _doc_faq:

FAQ
==========================
This will be updated as we get new questions. Please post questions in the forum. Answers to common problems will be compiled here.

General
----------------
Where can I find additional working examples of autonomous control code?

Please see this `repository <https://github.com/f1tenth/F110CPSWeek2018>`_ of past competitors submissions.

Mechanical
----------------
Do I have a broken drive train? How can I fix it?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Remove the rear housing on the vehicle’s differential. There is only a single screw securing it.

Once the housing is removed confirm that the gear on the output shaft is in contact and properly meshed with the slipper clutch gear.

If it is not loosen the screw located in the slot of the motor housing so that the output shafts position may be adjusted. Once the gears are again properly meshed hold the assembly in place and secure the loosened screw.

Before putting the cover back on test the system by rol>ng the car back and forth. You should see the center drive shaft turn etc. It is normal to have some noise on startup (this is the slipper clutch engaging).

When you are finished place the cover back on the gearbox assembly and secure it. You need to slide one side in between the motor and the edge of the car first.

Differential makes excessive noise...
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use the ​Lock, Rock, and Roll​ method shown in this video to adjust the pressure on the slipper clutch​

I’m not able to steer the car, no response from the steering servo...
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Double check that the servo wires are properly and securely connected. Confirm that you built the servo_out firmware for the VESC

System identification failure and VESC tuning...
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First check that you have the correct firmware and hardware version installed.

Second check that you have properly updated the parameters for system identification in the FOC tab. See the image in the VESC tuning/setup instruction section.

Third check your battery voltage. Are you below the cutoff level? Are you using recommended batteries (e.g. 7 cell NiMH or 3S LiPO).

If you have confirmed these things and system identification still fails consider the following suggestion...

Excessive motor vibration and inertia from the drivetrain can negatively affect the system identification process. One option is to loosen the screw holding the motor and rotate the motor such that the gear on the output shaft is no longer in contact with the main gear connecting the motor to the drivetrain. You may consider a small piece of foam or similar to dampen the motor vibrations after loosening the screw.

Removing the stock ESC housing for additional space...

It initially appears that there is no way to remove this piece. In fact their is a screw underneath the black esc module contained in the blue housing. The black esc module is secured by double sided tape. Pry the module loose with a flat head screwdriver or similar and remove the screw. It should now be possible to remove the blue housing.

Printing and laser cutting replacement parts...

The base plate is a simple laser cut piece. The CAD files are provided here. For the laser cut pieces you will need the dwg files. While your machines may not be exactly the same as the ones at Penn, every laser cutter I used is similar to the ones described here and here.

In addition there are two small 3d printed parts, even a low cost printer like a maker bot will be fine. You can find the 3d printed parts here. Basic information about using a MakerBot can be found here.

Wireless Network
--------------------------------
High packet-loss, excessive latency on wireless network...
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
One common source of network latency is the physical connection between the antenna and the Jetson TX2 module. Please double check that the wires/connectors are properly seated on the Jetson TX2 module. In addition we recommend a high-quality router such as __ in order to improve range and network throughput. Note that streaming images from onboard devices such as a camera will generally be slow no matter what.

Increasing range of the gamepad
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Adding an antenna

Use the antenna (extra usb cable) provided with the Logitech controller.

Using other remote controls

Erwin Coumans of Google Brain provides this library for more typical RF based RC controllers. Uses the Quanum RC control with Teensy 3.2 as better joystick. A switch on the remote switches between human control, OFF and self-driving. (so you don't need to hold the buttons). We can also easily program it to keep a a number of constant speeds, nice for data collection.

Software
----------------
VESC serial failures, VESC hardware and software revisions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you are using the VESC-x or the VESC-6 the data serialization specification has changed, you need to recompile the vesc_driver with alternate headers. Branches can be found here ___ for the VESC-x and here __ for the VESC-6

LIDAR variants
^^^^^^^^^^^^^^^^
If you are using the Hokuyo 10LX please confirm that you properly configured the wired network connection as described here.

Installing pyTorch
^^^^^^^^^^^^^^^^^^^
#. Make sure that you system path includes CUDNN

.. code-block:: bash

   $ sudo python -c 'import os; print(os.getenv("CUDNN_LIB_DIR"))'

#. Trun sample bash script to install pyTorch. You have to build from source because pyTorch does not have any arm64 binaries (due to its use of anaconda).

.. code-block:: bash

   #!/usr/bin/env bash
   # install jetson-utils prerequisites
   sudo apt-get update
   sudo apt-get install libglew-dev glew-utils libgstreamer1.0-dev
   libgstreamer-plugins-base1.0-dev libglib2.0-dev
   sudo apt-get install python-pip
   sudo apt-get install python-tk python-gi-cairo
   sudo apt-get install libfreetype6-dev

   # upgrade pip
   pip --version
   pip install --upgrade pip==9.0.1
   pip --version

   sudo pip install matplotlib
   sudo pip install pyglet==1.3.1      # lock pyglet for patch

   sudo sed -i 's/_have_getprocaddress = True/_have_getprocaddress =
   False/' /usr/local/lib/python2.7/dist-packages/pyglet/gl/lib_glx.py

   # setproctitle extension used by A3G
   sudo pip install setproctitle

   # install numpy
   sudo pip install numpy

   # clone pyTorch repo
   git clone https://github.com/pytorch/pytorch
   cd pytorch
   git tag
   git checkout v0.3.0
   git branch
   git submodule update --init

   # install prereqs
   sudo pip install -U setuptools
   sudo pip install -r requirements.txt

   # Develop Mode:
   python setup.py build_deps
   sudo python setup.py develop

   cd torch
   ln -s _C.so lib_C.so
   cd lib
   ln -s libATen.so.1 libATen.so
   cd ../ ../

   git clone https://github.com/pytorch/vision
   cd vision
   sudo python setup.py install

#. Run these commands to test

.. code-block::  bash

   python # Open a REPL
   import torch
   torch.backends.cudnn.is_acceptable(torch.cuda.FloatTensor(1))
   # if this returns true you are ready to go!

Additional Resources
""""""""""""""""""""""""
See the following pages:

`https://github.com/dusty-nv/jetson-reinforcement <https://github.com/dusty-nv/jetson-reinforcement>`_
`https://github.com/andrewadare/jetson-tx2-pytorch <https://github.com/andrewadare/jetson-tx2-pytorch>`_

Request for feedback...

Does this work for you? Please list your Jetpack version, CUDA version, and CUDNN version. If you encountered any difficulties were you able to solve them? How?

Installing Tensorflow
First double check which Jetpack version and which CUDA version you have installed on your TX2. You should be able to determine the Jetpack version from the GUI that you used when flashing your board. If you are unsure of the CUDA version open a terminal and inspect the results of nvcc --version.

Follow the instructions posted here, note that the wheels provided are quite old and may not work with your Jetpack/CUDA version...

Updated wheel files available here:

A quick google search will likely yield your desired variant. Here are some alternate options for convenience. Add the wheel files to the appropriate installTensoFlowJetsonTX directory and proceed.

Tensorflow Version 1.1 with JetPack 3.3
Tensorflow Version 1.6 with JetPack 3.1 or 3.2
Using gstreamer and image processing pipeline

Recording video from sensors like the Zed camera on the Jetson TX2 can be slow. This github gist details a solution using gstreamer.

Kernel
----------------
USB doesn’t work...

If you are using the Jetson TX2 you need to build the board support package for the Orbitty carrier. See here.

USB works, but LIDAR and VESC do not work...

First check that you are opening the correct device. It is highly recommended that you setup udev rules as described here.

If this fails to work then there is a strong chance that you need to install the ttyACM module. For a convenient installer visit ​here.

Simulation and Experiments without Hardware
------------------------------------------------
Virtual Machine Setup...
Coming soon.