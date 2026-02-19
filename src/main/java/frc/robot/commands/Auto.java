// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class Auto extends Command {
  public Auto() {
    addRequirements(Robot.driveTrain);
  }

  private long startTime = 0;

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    long currentTime = System.currentTimeMillis();
    if (currentTime - startTime < Constants.AUTO_DRIVE_TIME_MS) {
      Robot.driveTrain.setLeftMotors(Constants.AUTO_DRIVE_SPEED);
      Robot.driveTrain.setRightMotors(Constants.AUTO_DRIVE_SPEED);
    } else {
      Robot.driveTrain.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
