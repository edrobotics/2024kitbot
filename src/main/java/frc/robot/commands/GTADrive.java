// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class GTADrive extends Command {
  public GTADrive() {
    addRequirements(Robot.driveTrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.controllerType == "ps4")
    {
      double steering = Robot.m_oi.GetDriverRawAxis(Constants.ps4_leftStickX);
      steering = steering*Math.abs(steering);
      double leftTrigger = Robot.m_oi.GetDriverRawAxis(Constants.ps4_leftTrigger);
      double rightTrigger = Robot.m_oi.GetDriverRawAxis(Constants.ps4_rightTrigger);
      double speed = (rightTrigger-leftTrigger)/2;

      Robot.driveTrain.setLeftMotors(speed+steering);
      Robot.driveTrain.setRightMotors(speed-steering);
    }
    else if(Constants.controllerType == "logitech")
    {
      double steering = Robot.m_oi.GetDriverRawAxis(Constants.logitech_leftStickX);
      double leftTrigger = Robot.m_oi.GetDriverRawButton(Constants.logitech_buttonLT) ? 1 : 0;
      double rightTrigger = Robot.m_oi.GetDriverRawButton(Constants.logitech_buttonRT) ? 1 : 0;
      double speed = rightTrigger-leftTrigger;

      Robot.driveTrain.setLeftMotors(speed+steering);
      Robot.driveTrain.setRightMotors(speed-steering);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setLeftMotors(0);
    Robot.driveTrain.setRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
