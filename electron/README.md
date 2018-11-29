

Compile:

  particle compile electron electron.ino

Can constrain to a target:

  particle compile electron --target 0.7.0 electron

To flash firmware:

  particle flash --serial electron.bin


To see debugging output:

  particle serial monitor


To check firmware version:

  particle identify
  