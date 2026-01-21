// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
  private XboxController driveControl1 = new XboxController(Constants.X_BOX_1);

  public double GetDriverRawAxis(int axis) {
    return driveControl1.getRawAxis(axis);
  }
}
