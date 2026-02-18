// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;

public class PickUpFuel extends Command {
  public PickUpFuel() {
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(Constants.copilotControllerType == "ps4") {
      boolean circleButton = Robot.m_oi.GetCopilotRawButton(Constants.ps4_buttonCircle);
      boolean squareButton = Robot.m_oi.GetCopilotRawButton(Constants.ps4_buttonSquare);
      //If circle is pressed, it runs in positive direction
      //If square is pressed, it runs in negative direction
      //If both are pressed, it does not run
      Robot.intake.setIntakeMotor(squareButton ^ circleButton ? (squareButton ? -1 : 1) : 0);
    }
    else if(Constants.copilotControllerType == "logitech") {
      boolean buttonB = Robot.m_oi.GetCopilotRawButton(Constants.logitech_buttonB);
      boolean buttonX = Robot.m_oi.GetCopilotRawButton(Constants.logitech_buttonX);
      //If B is pressed, it runs in positive direction
      //If X is pressed, it runs in negative direction
      //If both are pressed, it does not run
      Robot.intake.setIntakeMotor(buttonB ^ buttonX ? (buttonB ? 1 : -1) : 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.setIntakeMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
