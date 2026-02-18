// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimbCommand extends Command {

  public ClimbCommand() {
    addRequirements(Robot.climb);
  }

  @Override
  public void execute() {
    int pov = Robot.m_oi.GetCopilotPOV();
    double speed = 0;
    if(pov == 0) { speed = 1; }
    else if(pov == 180) { speed = -1; }
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
