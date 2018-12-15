

Compile:

  particle compile electron electron.ino

Can constrain to a target:

  particle compile electron --target 0.7.0 electron




With a fresh Electron out of the box, you want to:

  1. Hold down Mode button until blinking blue LED
  2. Run: particle identify

If the firmware is _not_ 0.7.0:

  1. Press and hold both the RESET/RST and MODE/SETUP buttons simultaneously.
  2. Release only the RESET/RST button while continuing to hold the MODE/SETUP button.
  3. Release the MODE/SETUP button once the device begins to blink yellow.
  4. Run: particle update

Once you are on 0.7.0, you can flash the actual FloatHub firmware:

  1. Hold down Mode/Setup button until blinking blue LED
  2. Run: particle flash --serial electron.bin

If it is succesful, the easiest indication is that the battery LED will stop
blinking furiously (meaning it is actually running FloatHub code).

To see debugging output:

  particle serial monitor

