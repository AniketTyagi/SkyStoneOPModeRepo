# SkyStoneOPModeRepo
Summary:
This is the code repository for Milpitas Xtreme Robotic's FTC Skystone Robot. Alongside the general OPmode, this also includes a mini library, consistent of a series of developer function that can ease development time for processes such like the image recognition for stones, autonomous mode management and creation, and just general optimization, tweaking, and addition to the already existing OPmode. 

NOTE for developers planning to add to the OPmode:
- You may add onto the code so long as what you add on is in functional format.
  This is so future programmers and those who want to add onto the code aren't hindered
  by your conventions. By using functional format, ie: you create a method to run your
  code along with a description of what it does and what the inputs are, programmers will be
  able to further add to the code and also understand what you wrote.
- Each method has a description of what it does, along with
  a note of the input-output structure, ie: What values go in and what values
  come out.
- Try to make your method as independent of external values as possible and more dependent on input variables.
  If that cannot be achieved, remember to document what external variable you are accessing for use and why.
- The following is an example of a developer method that is in the OPMode:
  ```java
  // Function that controls drive base motors for a mecanum-drive
      // Input-Output Structure: (x1, y1, x2, m, angle-offset) => (void)
      public void mecanumDrive(double x1, double y1, double x2, double m, double theta) {
          // x1 and y1 are the x and y values from the joystick which controls the robot's drive
          // x2 is the x value from the joystick that will control the robot's left-right rotation
          // m is the speed modifier for the motors, making spin slower or faster
          // theta is the orientation you would like the robot to be in

          // Finds new orientation coordinates
          double nx = planarRotation(x1, y1, theta)[0];
          double ny = planarRotation(x1, y1, theta)[1];

          // Set power to motors accordingly
          flMotor.setPower((-(nx + ny) + 0.75 * -x2) * m);
          brMotor.setPower((-(nx + ny) + 0.75 * x2) * m);
          blMotor.setPower((-(nx - ny) + 0.75 * x2) * m);
          frMotor.setPower((-(nx - ny) + 0.75 * -x2) * m);
      }
  ```
Thank you for reading this README and remember programmers. Gracious proffesionalism. Happy developing!
