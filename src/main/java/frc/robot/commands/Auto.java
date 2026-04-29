// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.Functions;

public class Auto extends Command {
  public Auto() {
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  int currentWaypoint = 0;
  Double[][] waypoints = {{1.0,0.0},{-1.0,1.0}};

  private long startTime = System.currentTimeMillis();
  @Override
  public void execute() {
    if(currentWaypoint < waypoints.length)
    {
      double dist = Functions.pythagoranTheorem(Robot.deadReck.getRobotX(), Robot.deadReck.getRobotY(), waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
      double headingToWaypoint = Functions.headingTo(Robot.deadReck.getRobotX(), Robot.deadReck.getRobotY(), waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
      double headingDiff = Functions.angularDifference(headingToWaypoint, Robot.deadReck.getRobotHeading());
      if(headingDiff > 90)
      {
        Robot.driveTrain.setLeftMotors(-1);
        Robot.driveTrain.setRightMotors(-1+(180-headingDiff)/dist/45);
      }
      else if(headingDiff < -90)
      {
        Robot.driveTrain.setLeftMotors(-1-(-180-headingDiff)/dist/45);
        Robot.driveTrain.setRightMotors(-1);
      }
      else if(headingDiff < 0)
      {
        Robot.driveTrain.setLeftMotors(1+headingDiff/dist/45);
        Robot.driveTrain.setRightMotors(1);
      }
      else
      {
        Robot.driveTrain.setLeftMotors(1);
        Robot.driveTrain.setRightMotors(1-headingDiff/dist/45);
      }
      if(Functions.pythagoranTheorem(Robot.deadReck.getRobotX(), Robot.deadReck.getRobotY(), waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]) < 0.2) { currentWaypoint++; }
    }
    else
    {
      Robot.driveTrain.setLeftMotors(0);
      Robot.driveTrain.setRightMotors(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public long getAutoTime()
  {
    return System.currentTimeMillis()-startTime;
  }
}
