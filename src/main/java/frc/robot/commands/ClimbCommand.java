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
    addRequirements(Robot.climber);
  }
  boolean buttonEncoderMotorWasPressed = false;
  boolean climberPriority = false; // if true, the climb encoder motor will have priority over the intake arms encoder motor
  boolean toggle = true;

  @Override
  public void execute() {
    boolean buttonClimber = Robot.m_oi.getCopilotClimber();
    double position = Robot.climber.getPosition();

    //Change the direction if the button is pressed
    positiveDirection = buttonClimber ? !positiveDirection : positiveDirection;

    double targetRotations = positiveDirection ? Constants.CLIMBER_TARGET_ROTATIONS : 0;

    boolean climberPriority = Robot.intake.armPriority == Constants.ArmPriority.INTAKE;
    boolean intakeIn = Functions.roundToDecimalPlaces(Robot.intakeArms.getPosition(),2) == 0;
    boolean climbingAllowed = intakeIn || climberPriority;
    double winchSpeed = climbingAllowed ? (positiveDirection ? Constants.CLIMBER_WINCH_UP_SPEED : Constants.CLIMBER_WINCH_DOWN_SPEED) : Constants.CLIMBER_WINCH_HOLD_IN_PLACE;

    Robot.climber.runWinch(winchSpeed);
    Robot.climber.runEncoderMotor(Math.abs(targetRotations - position) > Constants.INTAKE_ARMS_DEADBAND ? -Constants.CLIMBER_ENCODER_SPEED * (targetRotations - position) : 0);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted
  }
}
