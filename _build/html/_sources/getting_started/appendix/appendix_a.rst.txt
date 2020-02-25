.. _doc_appendix_a:

Appendix A: Flashing Jetpack via a VM
==========================================
(From ​https://github.com/KleinYuan/tx2-flash)

Prerequisites
A mac desktop connected with Wi-Fi
macOS Sierra 10.12.5 + (Not necessary this version, but this is what I tested)
access to internet and power (You can't do this locally)
NVIDIA TX2
A monitor, HDMI Cable, Ethernet Cable, one Keyboard (USB plugged in), one mouse (preferrable), one USB hub (preferrable)
Install Virtual Box and Extensions
1.1 Install Virtual Box
---------------------------------
Download Virtual Box for Mac from here​and install it first;

Then download Virtual Box extension here​and install it;

Extension is needed to enable USB-2/USB-3 connection/communications between any physical USB device and the virtual machine.

1.2 Spin up an Ubuntu VM
---------------------------------
Download Ubuntu 14.04 iso image from ​here​.

(Ubuntu 16.04 may also work, even I haven't tried it on my own. But here's a proof that Ubuntu 16.04 should also work.)

Then, create an ubuntu machine with following settings:

Storage is larger than 50GB
Go to Settings --> Network --> Adapter 1, change ​Attached to to ​ Bridged Adapter​, and name to whatever under Wi-Fi
Go to Settings --> Ports --> USB, ensure Enable USB Controller​ is under USB 3.0 (xHCI) Controller
Last, load the image that you just downloaded and spin up an VM.

1.3 Download NVIDIA JetPack
---------------------------------
In the VM, open firefox browser and go to NVIDIA's official ​website​and join them as a member so that you can download the JetPack.

In my case, I downloaded JetPack 3.1 (includes TensorRT2.1, cuDNN 6.0, VisionWorks 1.6, CUDA 8.0, Multimedia API, L4T, Development tools).

You are expected to find a file called ​JetPack-L4T-3.1-linux-x64.run​ under Downloads folder.

1.4 Install JetPack
---------------------------------
Open a terminal and navigate to Downloads folder, then change .run file as executable:

cd
cd ~/Downloads
chmod +x ./JetPack-L4T-3.1-linux-x64.run
Then, run the .run file:

./JetPack-L4T-3.1-linux-x64.run
Then just follow all the steps (basically from 2 to 12 without 9 in ​here​): choose TX2, L4T 3.1, full-install, then it will take a while to downloads all the dependencies and then a window will pop up to ask you to confirm the installation.

And when download/installation on host machine is finished, you just click all the way to pop up a terminal says :



And now, here's the tricky part:

Plug in the USB with a USB hub, then connect a mouse, keyboard to TX2
Connect via HDMI (HDMI-HDMI, no VGA adapters ok?) between TX2 and monitor so that you can see what's going on
Follow the descriptions, to power off TX2, connect to host-machine, power on, press ..... (just follow what they said), but just hold on Press Enter on host machine
Before you Press Enter on host machine's terminal, open settings of your VM, and go to Settings --> Ports --> USB, and click ​ Add new USB filters with all .....(blahblahblah)​, then add NVIDIA Corp. APX
Then, go to VM, click bottom right corner the button with shape an USB (pbbly forth one), and select NVIDIA Corp. APX​, (it would be great to unplug any other USB devices from your Mac)
Last, go back to your VM, do Press Enter​ on the terminal
And, go and grab a coffee, it will take around 30 min to complete the entire process.

1.5 Validation Flash
---------------------------------
Go to TX2, open terminal:

# Validate NVCC
nvcc -V
# Validate cuda
ls /usr/local | grep cuda
# Run example
# You can find this easily online, just google it.
Ta Da!