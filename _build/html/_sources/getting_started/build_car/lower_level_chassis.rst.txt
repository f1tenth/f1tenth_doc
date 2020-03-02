.. _doc_build_lower_level:


Lower Level Chassis
====================

We begin with setting up the lower level chassis, which serves as the foundation of the entire vehicle. 

Removing Traxxas Stock Components
-----------------------------------
Take the Traxxas from its box and remove the outer shell so you are left with just this:

.. image:: img/llchassis/llchassis01.jpg

We are going to remove everything except for the servo, which is the little blue box in the upper left. There are three hex keys that come with the traxxas. You will use this to remove and/or install almost all of the screws on the chassis. You may want to have a bowl or container of sorts nearby to hold all the screws that you’ll be removing as these screws will come in handy later. 

Remove the servo box by first unscrewing the lid of the box.

.. figure:: img/llchassis/llchassis02.jpg

Once the top is open, use a screwdriver to pry the black box below out. This will also help with removal of the blue antenna. There are screws beneath the black box that we have to access in order to completely remove the ESC box.

.. image:: img/llchassis/llchassis03.jpg

Once the ESC has been removed, you will see the two screws underneath. Unscrew these and then you’ll be able to remove the box.

.. image:: img/llchassis/llchassis04.jpg

Remove the motor and the rest of the other components so that your final lower level chassis looks like the following:

.. image:: img/llchassis/llchassis05.jpg

The plastic fork will also need be removed. 

Installing the Brushless Motor
-------------------------------
To install the brushless motor, first remove the blue plate and the spur gear connected to the brushed (stock) motor. The spur gear is attached to the brushed motor with a set screw. Loosen the set screw with one of the hex keys provided in the Traxxas kit then pull the spur gear off. If it feels a bit stuck, carefully use a flathead screwdriver to push it off.

.. image:: img/llchassis/llchassis06.jpg

.. image:: img/llchassis/llchassis07.jpg

Install the blue plate and spur gear onto the brushless motor so your brushless motor now looks like this:

.. image:: img/llchassis/llchassis08.jpg

We have to align the spur gear with the rest of the gears on the lower level chassis. Remove the gear casing on the lower level chassis so you can see if the gears align or not.

.. image:: img/llchassis/llchassis09.jpg

.. image:: img/llchassis/llchassis10.jpg

Adjust the position of the spur gear accordingly so that when you move the car back and forth on the table, the movement should feel smooth and you can see the gears mesh and move without slipping or skipping teeth. Replace the covering once you are satisfied.

.. image:: img/llchassis/llchassis11.jpg

Once the brushless motor has been installed and all other components except the servo has been removed, the final lower level chassis should look like this:

.. image:: img/llchassis/llchassis12.jpg

Attaching the Standoffs
-------------------------
Install three M3 screws and three 45mm M3 FF standoffs into the lower chassis.

.. image:: img/llchassis/llchassis13.jpg

We only install three standoffs so that we can have easy access to the battery. You may want to use `thread lock <https://www.amazon.com/Loctite-Heavy-Duty-Threadlocker-Single/dp/B000I1RSNS/ref=sxin_1_ac_d_pm?ac_md=1-0-VW5kZXIgJDEw-ac_d_pm&cv_ct_cx=thread+lock&keywords=thread+lock&link_code=qs&pd_rd_i=B000I1RSNS&pd_rd_r=94268c5a-3e09-4447-a20e-0f4af52ac1b2&pd_rd_w=zvAiv&pd_rd_wg=WpfTu&pf_rd_p=516e6e17-ed95-417b-b7a4-ad2c7b9cbae3&pf_rd_r=ZPGZWZ9518Z8FR6860B5&psc=1&qid=1583189105>`_ to secure these standoffs as the vibrations of the car during movement may loosen them.

Setting Up the Battery
-----------------------
.. warning:: 
	**LIPO (LITHIUM POLYMER) BATTERY SAFETY WARNING**
	
	LiPO batteries allow your car to run for a long time, but they are not something to play with or joke about. They store a large amount of energy in a small space and can damage your car and cause a fire if used improperly. With this in mind, here are some safety tips for using them with the car.

	* When charging batteries, always monitor them and place them in a fireproof bag on a non-flammable surface clear of any other objects.
	* Do not leave a LIPO battery connected to the car when you’re not using it. The battery will discharge and its voltage will drop to a level too low to charge it safely again.
	* Unplug the battery from the car immediately if you notice any popping sounds, bloating of the battery, burning smell, or smoke.
	* Never short the battery leads.
	* Do not plug the battery in backwards. This will damage the VESC and power board (and likely the Jetson as well) and could cause a short circuit.
	* See ​this `video <https://www.youtube.com/watch?v=gz3hCqjk4yc>`_ for an example of what might happen if you don’t take care of your batteries. Be safe and don’t let these happen to you!

Place the battery into the compartment opposite of the motor.

.. image:: img/llchassis/llchassis14.jpg

Plug the `charge adapter <https://www.amazon.com/gp/product/B078P9V99B/ref=crt_ewc_title_huc_1?ie=UTF8&psc=1&smid=A87AJ0MK8WLZZ>`_ into the battery plug,

.. image:: img/llchassis/llchassis15.jpg

Then, connect the other side of the charge adapter to a TRX to XT90 cable.

.. image:: img/llchassis/llchassis16.jpg

It should look like this:

.. image:: img/llchassis/llchassis17.jpg

Attaching the PPM Cable
-------------------------
Take 3 header pins,

.. image:: img/llchassis/llchassis18.jpg

Plug it into the servo wires.

.. image:: img/llchassis/llchassis19.jpg

Connect the ppm cable with the servo wire.

.. image:: img/llchassis/llchassis20.jpg

The lower level chassis is now set up and we can move on to the autonomy elements.