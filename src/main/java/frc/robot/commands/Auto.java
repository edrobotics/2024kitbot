// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

import frc.robot.AutoPath;

public class Auto extends Command {
  public Auto() {
    addRequirements(Robot.driveTrain, Robot.intake, Robot.intakeArms);
  }

                                      //0  1    2    3
  private double[] waypointsX1 =       {0, 0,   5.1, 5.1};
  private double[] waypointsY1 =       {0, 4.2, 4.2, 2.2};
  private double[] waypointsDegrees1 = {0, 90,  180, 180};
  private final AutoPath auto1 = new AutoPath(waypointsX1, waypointsY1, waypointsDegrees1, 0, 4.2);
                                      //0  1    2     3
  private double[] waypointsX2 =       {0, 0,   -5.1, -5.1};
  private double[] waypointsY2 =       {0, 4.2, 4.2,  2.2};
  private double[] waypointsDegrees2 = {0, -90, -180, -180};
  private final AutoPath auto2 = new AutoPath(waypointsX2, waypointsY2, waypointsDegrees2, 0, 4.2);

  private final AutoPath currentAuto = auto2; // change this to auto2 to run the other path

  private long startTime = 0;
  private double startHeading = -9750;
  private int currentWaypoint = 0;

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    // rotates the intake arms during auto, rotating them down from 4 seconds to 13 seconds into the match
    //Robot.intakeArms.setLeftIntakeArmsMotor(getTime() > 4000 && getTime() < 5000 ? -1 : (getTime() > 13000 && getTime() < 15000 ? 1 : 0.1));
    //Robot.intakeArms.setRightIntakeArmsMotor(getTime() > 4000 && getTime() < 5000 ? -1 : (getTime() > 13000 && getTime() < 15000 ? 1 : 0.1));
    

    //Robot.intake.setIntakeMotor(getTime() > 5000 && getTime() < 13000 ? 1 : 0);
    
    // Dead Reckoning code for auto
    
    double robotX = Robot.deadReck.getRobotX();
    double robotY = Robot.deadReck.getRobotY();
    double robotHeading = Robot.deadReck.getRobotHeading();
    double targetX = currentAuto.waypointsX[currentWaypoint];
    double targetY = currentAuto.waypointsY[currentWaypoint];
    double targetDegrees = currentAuto.waypointsDegrees[currentWaypoint];
    
    if(Math.abs(robotX-targetX) < 0.1 && Math.abs(robotY-targetY) < 0.1) {
      if(startHeading == -9750) { startHeading = robotHeading; }
      double deltaTargetDegrees = targetDegrees - startHeading;
      while(deltaTargetDegrees < -180) { deltaTargetDegrees += 360; }
      while(deltaTargetDegrees > 180) { deltaTargetDegrees -= 360; }
      double deltaDegrees = robotHeading - startHeading;
      while(deltaDegrees < -180) { deltaDegrees += 360; }
      while(deltaDegrees > 180) { deltaDegrees -= 360; }
      Robot.driveTrain.autoRotate(deltaTargetDegrees, deltaDegrees);
      if(Math.abs((robotHeading - targetDegrees) % 360) < Constants.AUTO_ANGULAR_DEADBAND) {
        currentWaypoint++;
        startHeading = -9750;
      }
    }
    else {
      Robot.driveTrain.autoDriveMotors(targetX, targetY, robotX, robotY, currentAuto.waypointsX[currentWaypoint-1], currentAuto.waypointsY[currentWaypoint-1]);
    }
    Functions.printInTerminal(robotX + ", " + robotY + ", " + robotHeading);
    Functions.printInTerminal(currentWaypoint);

    // Intake arm in auto
    if(Functions.pythagoranTheorem(robotX, robotY, currentAuto.intakeX, currentAuto.intakeY) < 0.3) {
      Robot.intakeArms.setLeftIntakeArmsMotor(0.75);
      Robot.intakeArms.setRightIntakeArmsMotor(0.75);
    }
    else {
      Robot.intakeArms.stopIntakeArmsMotors();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentAuto.waypointsX.length == currentWaypoint;
  }

  public long getTime() {
    return System.currentTimeMillis()-startTime;
  }
}
