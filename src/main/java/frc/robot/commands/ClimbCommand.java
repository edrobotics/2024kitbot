// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class ClimbCommand extends Command {
  private boolean positiveDirection = true; // true for 90 degrees, false for -90 degrees

  public ClimbCommand() {
    addRequirements(Robot.climb);
  }
  boolean buttonEncoderMotorWasPressed = false;
  boolean climberPriority = false; // if true, the climb encoder motor will have priority over the intake arms encoder motor
  boolean toggle = true;

  @Override
  public void execute() {
    int pov = Robot.m_oi.getCopilotPOV();
    double speed = 0;
    if(pov == 0) { speed = Constants.CLIMB_UP_SPEED; }
    else if(pov == 180) { speed = Constants.CLIMB_DOWN_SPEED; }
    Robot.climb.runWinch(speed);

    boolean buttonEncoderMotor = Robot.m_oi.getCopilotIntakeArms();
    double position = Robot.climb.getPosition();

    double targetRotations;
    boolean intakePriority = Robot.intakeSmartArmsCommand.intakePriority;

    boolean intakeIn = Functions.roundToDecimalPlaces(Robot.intakeArms.getPosition(),2)==0;
    if (buttonEncoderMotor && !buttonEncoderMotorWasPressed) {
      climberPriority = !intakePriority && intakeIn; // if the intake arms are not in, the climb encoder motor will have priority when activated

      if (climberPriority && toggle) {
        targetRotations = Constants.CLIMBER_TARGET_ROTATIONS;
        toggle = false;
      } else {
        targetRotations = 0;
        toggle = true;
      }
    }

    buttonEncoderMotorWasPressed = buttonEncoderMotor;
    Robot.climb.runEncoderMotor(Math.abs(targetRotations - position) > Constants.CLIMBER_DEADBAND ? Constants.CLIMB_ENCODER_SPEED * (targetRotations - position) : 0);
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
