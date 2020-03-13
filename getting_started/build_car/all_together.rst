.. _doc_build_all_together:


Putting it all together
========================

Now that we have the autonomy elements attached to the upper level chassis, we are going to attach the upper level chassis to the lower level chassis. This part may be a tad unwieldy due to the amount of wires and cables that have to be contained.

Mounting the Upper Level Chassis to the Lower Level Chassis
------------------------------------------------------------
Gently place the upper level chassis on top of the standoffs of the lower level chassis. The VESC should be towards the back of the car. Thread the ppm cable from the lower level chassis, through one of the Platform Deck slot, and plug it into the ppm slot on the VESC.

.. image:: img/together/together01.JPG  

.. image:: img/together/together03.JPG  

.. image:: img/together/together02.JPG  

Use three M3 x 10mm (these are the ones that were removed from the chassis during the Lower Level Chassis build section) screws to attach the Platform Deck to the standoffs on the lower level chassis.

It may be useful to use a zip tie to secure the USB cable from the lidar to the platform.

.. warning::
	The driveshaft that runs along the length of the chassis rotates when the car moves. You can manage the cables and wires in whatever manner you like but make sure that you keep them away from any rotating assemblies, including the driveshaft. If you don't, then the rotating assemblies will pull on all the cables and the last 1-2 hours of your life will have been in vain.

Connecting the Brushless Motor to the VESC
-------------------------------------------
Take three 3mm to 3.5mm bullet adapters.

.. image:: img/together/together07.png

Attach the adapters to the blue, yellow, and white wires of the Brushless Motor.

 .. image:: img/together/together08.JPG

 
The VESC also has three wires labelled "A", "B", and "C".

 .. image:: img/together/together10.jpg

Now, we are going to connect these to the VESC. This part is a tad tricky. Connect the middle yellow cable to the cable connected to the label "B" on the VESC.

.. image:: img/together/together09.JPG

For now, connect the middle YELLOW wire to the VESC wire "B", the WHITE wire to "A", and BLUE wire to "C". After you flash the firmware on the VESC, if the vehicle runs backwards to the expected motion, simply swap the WHITE wire to "C" and BLUE wire to "A".

Final Touches
------------------------------
Almost there!

Attach the two wires for the Jetson Wi-Fi antenna to the two gold-colored connectors near the fan connector on the heat sink (the order of the wires doesn’t matter). This can be a little tricky, so you might want to use a flathead screwdriver to ensure the connections are tight. ​ Don’t press too hard​ , however as you can easily damage the connectors if you use excessive force!

.. image:: img/together/together02.JPG  

Connect a micro USB from the VESC to the USB hub.

.. image:: img/together/together06.JPG  

Finally, screw on the antennas included with the Jetson TX2 Kit to the Antenna Terminals. 

Voila!
-------
Your final vehicle should look like the following:

 .. image:: img/together/together04.JPG  


Now we're ready to start driving!

.. image:: img/together/together05.gif
   :align: center