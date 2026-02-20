// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PickUpFuel extends Command {
  public PickUpFuel() {
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean intakeIn  = Robot.m_oi.getCopilotIntakeIn();
    boolean intakeOut = Robot.m_oi.getCopilotIntakeOut();
    // XOR: if both pressed at once, do nothing; otherwise run in the appropriate direction
    Robot.intake.setIntakeMotor(intakeIn ^ intakeOut ? (intakeIn ? 1 : -1) : 0);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.setIntakeMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
