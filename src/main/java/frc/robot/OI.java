// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class OI {
  private GenericHID pilotController = new GenericHID(Constants.CONTROL_PILOT_ID);
  private GenericHID copilotController = new GenericHID(Constants.CONTROL_COPILOT_ID);

  //GetDriverPOV gets the value of the D-pad
  //  returns -1 if not pressed, else the number of degrees (starting at 0 for up, going clockwise)

  public double GetPilotRawAxis(int axis) {
    return pilotController.getRawAxis(axis);
  }
  public boolean GetPilotRawButton(int button) {
    return pilotController.getRawButton(button);
  }
  public int GetPilotPOV() {
    return pilotController.getPOV();
  }
  public double GetCopilotRawAxis(int axis) {
    return copilotController.getRawAxis(axis);
  }
  public boolean GetCopilotRawButton(int button) {
    return copilotController.getRawButton(button);
  }
  public int GetCopilotPOV() {
    return copilotController.getPOV();
  }
}
