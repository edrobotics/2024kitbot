// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TankDrive extends Command {
  public TankDrive() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.driveTrain.setLeftMotors(Robot.m_oi.getPilotLeftStickY());
    Robot.driveTrain.setRightMotors(Robot.m_oi.getPilotRightStickY());
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
