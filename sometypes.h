/*

  Because we want to pass these as arguments to functions, we have to
  declare them in a separate header file.  Such is the genius of the Arduino
  IDE

*/

typedef enum pump_state
{
  unknown,
  on,
  off
};

      