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

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(Constants.pilotControllerType.equals("ps4"))
    {
      double steering = Robot.m_oi.GetPilotRawAxis(Constants.ps4_leftStickX);
      steering = steering*Math.abs(steering);
      double leftTrigger = Robot.m_oi.GetPilotRawAxis(Constants.ps4_leftTrigger);
      double rightTrigger = Robot.m_oi.GetPilotRawAxis(Constants.ps4_rightTrigger);
      double speed = (rightTrigger-leftTrigger)/2;

      Robot.driveTrain.setLeftMotors(speed+steering);
      Robot.driveTrain.setRightMotors(speed-steering);
    }
    else if(Constants.pilotControllerType.equals("logitech"))
    {
      double steering = Robot.m_oi.GetPilotRawAxis(Constants.logitech_leftStickX);
      double leftTrigger = Robot.m_oi.GetPilotRawButton(Constants.logitech_buttonLT) ? 1 : 0;
      double rightTrigger = Robot.m_oi.GetPilotRawButton(Constants.logitech_buttonRT) ? 1 : 0;
      double speed = rightTrigger-leftTrigger;

      Robot.driveTrain.setLeftMotors(speed+steering);
      Robot.driveTrain.setRightMotors(speed-steering);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
