// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class GTADrive extends Command {
  public GTADrive() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double steering = Robot.m_oi.getPilotSteering();
    double speed    = Robot.m_oi.getPilotThrottle();

    Robot.driveTrain.setLeftMotors(speed + steering);
    Robot.driveTrain.setRightMotors(speed - steering);
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
