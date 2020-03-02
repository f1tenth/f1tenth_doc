.. _doc_appendix_b:

Appendix B: Shared Folders between Host and VM
=================================================
If you don’t have a host computer running Ubuntu, you may do most of the above using a Virtual Machine (VM). If so, it is useful to setup a shared folder between the ​host​ (your default OS on your laptop) and the ​guest ​ (Ubuntu running in the VM).

The following are for an Ubuntu 16.04 guest on Virtualbox, with the host a Mac OS X El Capitan (other Mac OSes should work too).

We assume you have already installed Virtualbox on your host, and installed Ubuntu 16.04 on it. More detailed instructions exist online, this is one way of doing it.

On your host, create the folder you wish to share. We’ll call it sfVM and assume it lives at ~/sfVM.
Start VirtualBox.
Make sure the VM is not on (shut it down if it is).
Select the Ubuntu VM.
Click Settings -> Shared Folders -> Click the ‘+’ sign -> browse to sfVM which you created above, and check “Auto-mount” and “Make permanent”.
Start the VM
Install the Guest Additions: in the VBox menu, click Devices -> Install Guest Additions -> ... ->
Run Software
Mount the shared folder manually by running:
$​ mkdir ~/guest_sfVM (or whatever you want to call it)
$​ id
uid=1000(houssam) gid=1000(houssam)
$​ sudo mount -t vboxsf -o uid=1000,gid=1000 sfVM ~/guest_sfVM
To check that this was successful, on your host, put some file in the shared folder ~/sfVM. Then in your guest, you should see that file appear in ~/guest_sfVM