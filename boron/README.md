
Compile:

  particle compile boron boron.ino

Can constrain to a target:

  particle compile boron --target 1.5.2 boron.ino
  particle compile boron --target 2.0.1 boron.ino


Hold down both and then release reset until blinking
yellow. Then do:

    particle flash --usb  2.0.1/boron/release/boron-bootloader@2.0.1.bin

    particle flash --usb boron_fc_good_firmware_2.0.1.bin



The file:

   boron_good_firmware_1.5.2.bin

has known good code that we used on many devices through 2019 and 2020 and
into early 2021 together with the particle firmware 1.5.2 for the boron.


In March 2021 we added the forceConfig (FC) capability to the code and
bumped it up to work with 2.0.1 particle firmware. That file is:

    boron_fc_good_firmware_2.0.1.bin

