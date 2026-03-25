// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Functions;
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
    double speed = Robot.m_oi.getPilotThrottle();
    double boost = Robot.m_oi.getPilotBoost() ? 2 : 1;

    Robot.driveTrain.setLeftMotorsBoosted(speed * boost - steering);
    Robot.driveTrain.setRightMotorsBoosted(speed * boost + steering);

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
