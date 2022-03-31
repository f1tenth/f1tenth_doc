.. meta::
    :keywords: FAQ

.. _doc_faq:

FAQ
==========================
This will be updated as we get new questions. Please post questions in the `F1TENTH Discord <https://f1tenth.discourse.group/>`_. Answers to common problems will be compiled here.

General
----------------
Where can I learn more about autonomous racing?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
There are different places where you find autonomous racing explanations and papers:
1. `Autonomous Racing Survey <https://arxiv.org/abs/2202.07008>`_ Complete survey paper on the field of autonomous racing that covers all software and hardware aspects.
2. `Autonomous Racing Survey Paper Repository <https://github.com/JohannesBetz/AutonomousRacing_Literature>`_ List of all papers in the field of autonomous racing.

Where can I find additional working examples of autonomous racing code?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
There are different places where you find autonomous racing code:
1. `F1TENTH Github Repository <https://github.com/f1tenth>`_ official F1TENTH github respository.
2. `Learn to Drive (and Race!) Autonomous Vehicles <https://github.com/f1tenth/ESweek2021_educationclassA3>`_ ESWeek 2021 tutorial.
3. `Raceline Generation <https://github.com/TUMFTM/global_racetrajectory_optimization>`_ Optimal Raceline generation Code from TUM.
4. `Graph based Planner <https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner>`_ Graph-based Trajectory Planner from TUM.

Material or components are not available
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Yes, we are aware that components are sometimes out of stock. Please be patient here and contact a few vendors. In addition there are many parts that are interchangeable like the NVIDIA Jetson components or the Traxxas chassis


Mechanical
----------------
Do I have a broken drive train? How can I fix it?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Remove the rear housing on the vehicle’s differential. There is only a single screw securing it.

Once the housing is removed confirm that the spur gear on the motor output shaft is in contact and properly meshed with the slipper clutch gear.

If the gears are not properly meshed, loosen the screw located in the slot of the motor housing so that the output shafts position may be adjusted. Once the gears are again properly meshed hold the assembly in place and secure the loosened screw. The gears should be meshed where they're just touching and not slipping. Do not force the two gears together such that they're exerting too much pressure on each other.

Before putting the cover back on test the system by rolling the car back and forth. You should see the center drive shaft turn. It is normal to have some noise on startup (this is the slipper clutch engaging).

When you are finished place the cover back on the gearbox assembly and secure it. You need to slide one side in between the motor and the edge of the car first.

Differential makes excessive noise
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use the ​Lock, Rock, and Roll​ method shown in `this video <https://youtu.be/C2iw9A7O_xk>`_ to adjust the pressure on the slipper clutch​.

I’m not able to steer the car, no response from the steering servo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Double check that the servo wires are properly and securely connected. Be careful about the polarity of the three pin connectors.
2. Confirm that you built the servo_out firmware for the VESC as shown `here <firmware/firmware_vesc.html#updating-the-firmware-on-the-vesc>`_.

System identification failure and VESC tuning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First check that you have the correct firmware and hardware version installed.

Second check that you have properly updated the parameters for system identification in the FOC tab. See the image in the VESC tuning/setup instruction section `here <firmware/firmware_vesc.html#detecting-and-calculating-motor-parameters>`_.

Third check your battery voltage. Are you below the cutoff level? Are you using recommended batteries (e.g. 7 cell NiMH or 3S LiPO).

If you have confirmed these things and system identification still fails consider the following suggestion:

Excessive motor vibration and inertia from the drivetrain can negatively affect the system identification process. One option is to loosen the screw holding the motor and rotate the motor such that the gear on the output shaft is no longer in contact with the main gear connecting the motor to the drivetrain. You may consider a small piece of foam or similar to dampen the motor vibrations after loosening the screw.

Printing and laser cutting replacement parts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The base plate is a simple laser cut piece. The CAD files are provided `here <https://drive.google.com/drive/u/1/folders/1o3jRww0UwfmjTBDACD8qu7SDabRzpr5g>`_.

Wireless Network
--------------------------------
High packet-loss, excessive latency on wireless network
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
One common source of network latency is the physical connection between the antenna and the Jetson NX module. Please double check that the wires/connectors are properly seated on the Jetson NX module. In addition we recommend a high-quality router in order to improve range and network throughput. Note that streaming images from onboard devices such as a camera will generally be slow no matter what.


Wireless interference with the gamepad
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Sometimes when using a USB 3 camera, the cable might create interference with the Logitech gamepad's usb dongle, and drastically decreasing the effective range of the gamepad. Cameras that are known to have this issue are StereoLab's ZED, and the DVS event-based cameras. Another scenario that the wireless gamepad fails is an environment with a large number of Wifi access points (e.g. a large conference).

In these cases, it is possible and recommended to use a bluetooth gamepad like the Gamepad for PlayStation 4. You'll need a usb bluetooth dongle too. Instructions on how to connect the gamepad via bluetooth can be found `here <https://youtu.be/v_neNpfQ38Q?t=386>`_.

Software
----------------
LIDAR variants
^^^^^^^^^^^^^^^^
If you are using the Hokuyo 10LX please confirm that you properly configured the wired network connection as described :ref:`here <doc_firmware_hokuyo10>`.


USB works, but LIDAR and VESC do not work
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First check that you are opening the correct device. It is highly recommended that you :ref:`setup udev rules <udev_rules>`.

If this fails to work then there is a strong chance that you need to install the ttyACM module. For a convenient installer visit `here <https://github.com/jetsonhacks/installACMModule>`_.
