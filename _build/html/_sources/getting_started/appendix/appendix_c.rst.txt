.. _doc_appendix_c:

Appendix C: Soldering the Power Board
======================================
The power board provides a stable 12V power source for the Jetson, LiDAR, and other peripherals. This is necessary since the battery’s voltage will drop as it is used. Please note that the power board does ​ not​ charge the battery; for this, you will need a Traxxas EZ-Peak charger, as listed in the F110 BOM.

Note: the LiPO protection module and some green connectors are currently unused and are legacy from previous iterations of F1/10 that used the Teensy microcontroller as motor driver.
Make sure you have the following equipment ready before you begin:

Soldering iron or station with adjustable temperature (recommended: Weller WES51​ or Hakko FX-888D​)
Lead-free solder wire
Brass tip cleaner
Solder sucker
Liquid flux pen
“Helping hands” with alligator clips
Power supply for testing the board
Optional: Isopropyl alcohol and toothbrush for cleaning the board
In addition, you’ll need to have a blank power board ready along with the components from the power board BOM​. Follow the ordering instructions above to obtain a blank PCB.

Some other notes and tips before we begin:

Apply flux to ​every pad​ and ​every pin​ of every component before soldering it! This is especially important since this PCB has large ground planes that make it difficult to solder if the surfaces aren’t cleaned beforehand.
Soldering requires patience and practice. Don’t do it on a whim, or if you’re tired or hungry. If you’ve never soldered before, practice on a perfboard or scrap PCB first, or get help from a friend. Expect to spend up to 3 hours soldering this board.
If you find that the iron isn’t melting solder well, try cleaning it using the brass tip cleaner and applying a thin coating of solder to the tip (known as “tinning”).
Don’t leave the iron at high heat for more than a few minutes at a time when you’re not using it, as it may cause excessive oxidation of the tip that makes it harder to clean and tin later.
When reflowing (melting) solder you’ve already applied to the board, ensure that ​all​ of the solder is melted before you apply more or remove the iron. Not doing this will create a “cold joint” that results in a high-resistance bridge between the solder and pad, which can result in failing connections later on as the board is used.
“Medium heat” as defined in this guide is 650°F. “High heat” is 800°F.
In general, with a few exceptions, we’ll be soldering components with the flattest profile first (the LiPO protection module) and those with the tallest profile last (switches U$3 - U$5). The image below shows the order we’ll be moving in.


Let’s get started!

LiPO Protection Module (U$2)
--------------------------------
Apply flux to each of the 7 pads of the LiPO protection module. Turn the soldering iron to medium heat and apply a thin coating of solder to each pad. You’ll know you’ve applied enough when the entire pad is covered and the solder makes a very slight bulge upward.


Flux and apply a similar coating of solder to the corresponding 7 pads on the power board. Again, make sure the entire pad is covered in a thin layer of solder.



Place the protection module on the power board (pay careful attention to orientation) and heat one of the pads with the iron until ​ all ​ of the solder on that pad is uniformly melted. Once this occurs, apply enough additional solder to the pad until the solder bridges both the pad on the PCB and the corresponding pad on the LiPO protection module.
It’s also a good idea to hold the iron in place for several seconds after you’ve applied enough solder to ensure the solder on the protection module melts and fuses to the solder on the board. This helps to avoid cold joints.
Don’t apply so much solder as to make it bulge above the plane of the protection module.


NVIDIA Power Supply Jack (U$7)
--------------------------------
Apply flux to the 4 pads (both front and back) of the power jack, as well as the power jack pins. Insert the power jack into the power board, turn the board over, and use a clip on the “helping hands” to hold the board in place on the opposite end of the board. Make sure that the power jack is flush with the surface of the board before soldering.
Apply enough solder to each of the pads on the power jack until the pad is covered in solder and makes a conical tent shape.
One of the pads is closer to a ground plane (the lighter-colored area) than the other three and might take a little longer to solder. If the solder doesn’t stick after holding the iron for several seconds, try turning the temperature to high heat. You can also apply more solder to the pad and suck it up with the solder sucker until it sticks to the pad; just make sure that the solder isn’t in a large ball when you’re finished.


Terminal Blocks (X1 - X9) and Some Small Capacitors (C8, C9)
----------------------------------------------------------------
Since we have a large number of terminal blocks, we’ll break this down into several steps, with a few terminal blocks in each step. Start with terminal blocks X2 - X4 by applying flux to the pads and pins of the blocks. Place them onto the power board and use the helping hand to balance the board so the blocks lie flush with the board.
Make sure the terminal blocks are oriented with the wire holes facing the ​outside edge​ of the board, not the center.
Solder the 12V terminals (the ones with a gap between the pin and ground plane) of each​ of the blocks first to ensure all of them remain stuck to the PCB if you accidentally nudge the board. Then solder the remaining pins (the GND pins) that are closer to the ground plane. You might need to use high heat to solder the remaining pins. In both cases, you’ll want to apply just enough solder to make a conical tent shape as pictured.
Repeat step 7 for the remaining terminal blocks (X5 - X8) on that side of the board.


You might be tempted to solder blocks X1 and X9 right now. Don’t do this yet! Instead, grab two 100nF capacitors, apply flux to the pads and pins for C8 and C9 (near X1 and X9), and solder those capacitors on.
It doesn’t matter which way you put these capacitors in since they’re non-polarized.
It’s normal (and beneficial) to have some solder flow to the other side of the board and attach to the opposite pad since it improves the mechanical strength and electrical conductivity of the joint, but it isn’t strictly necessary for a good connection. Don’t sweat it if you can’t get it.


Repeat step 7 to solder X1 and X9 to the power board. You shouldn’t need to use the helping hand at this point since the terminal blocks on the other side of the board will naturally balance the board.
To avoid repeating directions, the following steps will assume you’ve properly fluxed the pins and pads of each component before soldering them. This step is ​critical​; make sure you do it every time!

Small Slider Switches (SW1, SW2), Battery Power Jack (U$10), and Remaining Small Capacitors (C3, C4, C5, and C6)
--------------------------------------------------------------------------------------------------------------------------------
Apply solder to the outer (larger) pads of both switches ​ first​ before soldering the smaller pads. This is to help you ensure the switches have a good mechanical connection to the board and are straightened out before you finish soldering them.
It’s fine for these switches if the solder is “pointier” and less cone-shaped than for the terminal blocks since the solder has a tendency to flow to the opposite side of the board for these switches.
Don’t apply too much solder to these switches since the plastic in them can melt if heated for an extended period of time. Stop applying solder once you notice it sinking to the opposite side.


Solder the battery power jack similarly to how you soldered terminal blocks X2 - X4. You’ll want to first start with the pins separated from the ground plane and then solder the GND pin connected to the ground plane, using high heat if necessary.
The battery jack should stay in place by friction alone, so you shouldn’t need a helping hand or alligator clip.


Grab three 100nF capacitors and one 330nF capacitor. Solder the remaining small capacitors according to step 9. For C3 and C4, solder the ground pin last and use higher heat if necessary.
Make sure you place the capacitors into the correct pads on the board. It’s easy to get confused when you flip the board over, especially for C5 and C6. Also, note that C5 is 330nF and not 100nF.


Resistors (R1, R2, R3) and Headers (U$1, CON1, CON2, CON3, CON4)
----------------------------------------------------------------
Solder the resistors using the same technique you used for the small capacitors. Cut the excess leads off the bottom of the board when you’re finished.


Solder the four 3-pin male headers first. Use a long female header to align the male headers while you solder.


Optional​: Solder header JP1 (near the LiPO protection module). We don’t recommend this since this header lies over a large ground plane (making it harder to solder) and isn’t used for anything except for probing with a multimeter.
Insert the two 14-pin Teensy headers into the board and solder the ​two outermost pins of each header before soldering the rest. This makes realigning the headers, if necessary, much easier than if you soldered all of them at once.


Battery DC to 12V DC Converter
--------------------------------
This is probably the most ​time-consuming​ part of soldering this board, so take a break if you’re tired or hungry. Don’t proceed with this component until you feel ready!

Flux both the ​pads​ and the ​pins of the power converter​ before beginning. This is especially important for this component since both sides with pins lie over large ground planes, and the component itself acts like a large heat sink.
Place a spare header or other small object (non-flammable) in the space underneath the power converter, and then place the power converter on top. The purpose of this step is two-fold:
It mitigates the effects of the PCB and component acting as a heat sink, allowing solder to flow more freely into the pads.
It makes it much easier to remove the component later if you make a mistake, which is important because these are the most expensive components on the power board.


Clean your iron using the tip cleaner and turn it to high heat. Apply solder to the tip and heat one of the four pads and pins that isn’t directly attached to a ground plane. Don’t apply additional solder until you’ve heated the pad/pin for at least 15 seconds.
Apply enough additional solder to make a cone shape. Make sure it sticks to the whole surface of the pad and not just the pin itself. It helps to keep the iron on one side of the pin and apply solder to the gap between the pad and pin on the opposite side so the solder doesn’t just stick to the iron.
Repeat steps 20 and 21 for the five remaining pins. Make sure you solder the non-grounded pins first (they’re much easier).
If solder isn’t sticking to the ground pads, you can try alternately applying a ball of solder and using the solder sucker on the excess until it does stick. You can also try refluxing the pins and pads.


Remove the object you placed between the PCB and power converter, and turn the iron back down to medium heat.

12V to 8V Converter (U1), LEDs (D1, D2, and D3), and Large Capacitors (C1, C2, and C7)
------------------------------------------------------------------------------------------------
Place the 12V to 8V converter into its pad, making sure the ridged side faces terminal blocks X2 - X8 and the flat side faces the LiPO converter. Bend the two outer pins slightly to ensure it stays in the board when you solder it.

Solder the power converter, aiming for a cone shape on each pin as you’ve done before.
Solder the three LEDs onto the board, paying attention to orientation. The LEDs have a flat ridge on their negative sides, which should match up with the silkscreened images on the PCB.
Place capacitors C1, C2, and C7 into the board, and ​make sure the orientation is correct​. These capacitors are polarized, and inserting them backwards could cause them to explode!
C1 should be oriented with its negative​ end facing towards the large, square DC-to-DC converter.
C2 should be oriented with its ​positive​ end facing towards the NVIDIA power jack near the edge of the board.
C7 should be oriented with its ​negative​ end facing the terminal blocks X2 - X8
Solder the non-grounded pin of each capacitor first to hold them in place, then solder the grounded pins similarly to how you soldered the large DC-to-DC converter, using higher heat if necessary. Above all, make sure solder covers the entire pad and is roughly cone-shaped to ensure a good connection.


Large Toggle Switches (U$3, U$4, and U$5)
----------------------------------------------------------------
Starting with U$5 (the switch closest to the board’s edge), place the switch into the board and solder one of its pins. These pads are larger than most pads, so they will probably require more solder to be covered completely. Straighten the switch if necessary and then solder the other pad.
If you find it hard to align the switch properly before soldering, use a helping hand on the other side of the board to balance it.
Toggling the switches may also help to align them.
Repeat step 28 for U$4 and U$3, in that order.


Testing the Board
--------------------------------
Connect a regulated DC power supply (​not a battery​) to the board, with the positive terminal going to the BATT connection on the white power jack and the negative terminal going to the GND_BATT connection. Set the voltage to 11.1V.
Apply power to the board, turn switch U#3 on, and monitor the current being drawn. With no load, the current drawn should not exceed roughly 20 mA.
If the current is too high, check the orientation of the LiPO battery protector and the capacitors. Make sure there are no stray pieces of solder on the board and no solder bridges between adjacent pins.
If no current is drawn, make sure the solder joints for the battery connector and power converter completely cover the pads.


Cleaning the Board (Optional)
--------------------------------
To give the board a clean and professional finish, you can clean the flux residue from the board using isopropyl alcohol and a toothbrush. Make sure your soldering iron is off first (isopropyl alcohol is very flammable), apply some alcohol to the toothbrush, and scrub both sides of the board in a circular motion. Allow the alcohol to dry before you use the board. It’s normal for the board to feel a little sticky after the alcohol dries; this should go away within a few hours.
