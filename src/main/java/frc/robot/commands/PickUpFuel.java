// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PickUpFuel extends Command {
  public PickUpFuel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.controllerType == "ps4") {
      boolean circleButton = Robot.m_oi.GetDriverRawButton(Constants.ps4_circleButton);
      Robot.intake.setIntakeMotor(circleButton ? -1 : 0);
    }
    else if(Constants.controllerType == "logitech") {
      boolean buttonB = Robot.m_oi.GetDriverRawButton(Constants.logitech_buttonB);
      Robot.intake.setIntakeMotor(buttonB ? -1 : 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.setIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
