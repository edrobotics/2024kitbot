// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
  private XboxController driveControl1 = new XboxController(Constants.CONTROL_1_ID);

  //GetDriverRawAxis gets the value of the given axis
  //  axis: 0 - left stick X, 1 - left stick Y, 2 - left trigger, 3 - right trigger, 4 - right stick X, 5 - right stick Y
  //  returns a value between -1.0 and 1.0
  //GetDriverRawButton gets the value of the given button
  //  button: 1 - square, 2 - X, 3 - circle, 4 - triangle, 5 - left bumper, 6 - right bumper, 7 - back, 8 - start, 9 - left stick press, 10 - right stick press
  //  returns either true or false
  //GetDriverPOV gets the value of the D-pad
  //  returns -1 if not pressed, else the number of degrees (starting at 0 for up, going clockwise)

  public double GetDriverRawAxis(int axis) {
    return driveControl1.getRawAxis(axis);
  }
  public boolean GetDriverRawButton(int button) {
    return driveControl1.getRawButton(button);
  }
  public int GetDriverPOV() {
    return driveControl1.getPOV();
  }
}
