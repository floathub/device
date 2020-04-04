

Compile:

  particle compile boron boron.ino

Can constrain to a target:

  particle compile boron --target 1.5.0 boron


With a fresh Boron out of the box, you want to:

  1. Hold down Mode button until blinking blue LED
  2. Run: particle identify

If the firmware is _not_ 1.5.0:

  1. Press and hold both the RESET/RST and MODE/SETUP buttons simultaneously.
  2. Release only the RESET/RST button while continuing to hold the MODE/SETUP button.
  3. Release the MODE/SETUP button once the device begins to blink yellow.
  4. Run: particle update

Once you are on 1.5.0, you can flash the actual FloatHub firmware:

  1. Hold down Mode/Setup button until blinking blue LED
  2. Run: particle flash --serial boron_firmware.bin

To see debugging output:

  particle serial monitor

