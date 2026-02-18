// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Auto extends Command {
  public Auto() {
    addRequirements(Robot.driveTrain);
  }

  private long startTime = 0;

  @Override
  public void initialize() {
    startTime = new java.util.Date().getTime();
  }

  @Override
  public void execute() {
    double lastTime = 0; // always set lastTime in the different time intervals so the motors get turned off instantly once the autonomous commands have been run
    long currentTime = new java.util.Date().getTime();
    if(currentTime-startTime < 3000) {
      Robot.driveTrain.setLeftMotors(0.5);
      Robot.driveTrain.setRightMotors(0.5);
      lastTime = 3000;
    }
    else if(currentTime-startTime > lastTime){
      Robot.driveTrain.setLeftMotors(0);
      Robot.driveTrain.setRightMotors(0);
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
