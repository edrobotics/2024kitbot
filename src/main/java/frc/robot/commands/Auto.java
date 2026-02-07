// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Auto extends Command {
  /** Creates a new Auto. */
  public Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  private long startTime = 0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //int startTime = new java.util.Date().getTime();
    startTime = new java.util.Date().getTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lastTime = 0; // always set lastTime in the different time intervalls so the motors get turned off instantly once the autonomous commands have been run
    long currentTime = new java.util.Date().getTime();
    if(currentTime-startTime < 3000) {
      Robot.driveTrain.setLeftMotors(0.5);
      Robot.driveTrain.setRightMotors(0.5);
      lastTime = 3000;
    }
    float degrees = 90;
    if(currentTime-startTime >= 3000 && currentTime-startTime < 3000 + ((degrees*Constants.wheelBaseWidth*Math.PI)/(360*Constants.leftWheelVelocity))*1000) {
      Robot.driveTrain.setLeftMotors(1);
      Robot.driveTrain.setRightMotors(-1);
      lastTime = 3000 + ((degrees*Constants.wheelBaseWidth*Math.PI)/(360*Constants.leftWheelVelocity))*1000;
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
