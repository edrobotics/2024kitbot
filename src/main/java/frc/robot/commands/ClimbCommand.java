// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
/** Command that runs the climb motor while scheduled. */
public class ClimbCommand extends Command {

  public ClimbCommand() {
    addRequirements(Robot.climb);
  }
  /** Create a climb command with default speed 0.6. */
  /*public ClimbCommand(ClimbSubsystem climb) {
    this(climb, 0.6);
  }

  public ClimbCommand(ClimbSubsystem climb, double speed) {
    this.climb = climb;
    this.speed = speed;
    addRequirements(climb);
  }*/

  @Override
  public void execute() {
    int pov = Robot.m_oi.GetDriverPOV();
    double speed = 0;
    if(pov == 0) { speed = 1; }
    else if(pov == 180) { speed = -1; }
    //double speed = pov == 0 ? 1 : 
    Robot.climb.run(speed);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climb.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted
  }
}
