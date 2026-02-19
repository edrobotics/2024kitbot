// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class TankDrive extends Command {
  public TankDrive() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(Constants.pilotControllerType.equals("ps4")) {
      double leftStickY = Robot.m_oi.GetPilotRawAxis(Constants.ps4_leftStickY);
      double rightStickY = Robot.m_oi.GetPilotRawAxis(Constants.ps4_rightStickY);
  
      Robot.driveTrain.setLeftMotors(leftStickY);
      Robot.driveTrain.setRightMotors(rightStickY);
    }
    else if(Constants.pilotControllerType.equals("logitech")) {
      double leftStickY = Robot.m_oi.GetPilotRawAxis(Constants.logitech_leftStickY);
      double rightStickY = Robot.m_oi.GetPilotRawAxis(Constants.logitech_rightStickY);

      Robot.driveTrain.setLeftMotors(leftStickY);
      Robot.driveTrain.setRightMotors(rightStickY);
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
