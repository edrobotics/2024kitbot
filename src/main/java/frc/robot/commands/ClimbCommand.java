// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class ClimbCommand extends Command {
  private boolean positiveDirection = false; // true for 90 degrees, false for -90 degrees

  public ClimbCommand() {
    addRequirements(Robot.climber);
  }
  boolean buttonEncoderMotorWasPressed = false;
  boolean climberPriority = false; // if true, the climb encoder motor will have priority over the intake arms encoder motor
  boolean toggle = true;

  @Override
  public void initialize() {
    positiveDirection = false;
  }

  @Override
  public void execute() {
    double manualDrive = Robot.m_oi.getCopilotManualClimber();
    
    if(Math.abs(manualDrive) > 0.1) {
      Robot.climber.runWinch(manualDrive*0.3);
      Robot.climber.runEncoderMotor(manualDrive*0.3);
      Robot.driveClimberManually = true;
    }
    else if(Robot.driveClimberManually) {
      Robot.climber.runWinch(0);
      Robot.climber.runEncoderMotor(0);
    }

    /*
    boolean buttonClimber = Robot.m_oi.getCopilotClimber();
    Functions.printInTerminal(buttonClimber);
    
    if(buttonClimber && Robot.driveClimberManually) {
      Robot.driveClimberManually = false;
      positiveDirection = false;
    }
    if(!Robot.driveClimberManually) {
      boolean intakeIn = Functions.roundToDecimalPlaces(Robot.intakeArms.getPosition(),2) == 0;

      if(!intakeIn && buttonClimber) {
        positiveDirection = false;
      }
      else if(intakeIn && buttonClimber) {
        positiveDirection = !positiveDirection;
      }
    */

      //Robot.climber.rotateClimbArms(positiveDirection, 0.75 * Constants.CLIMBER_MOTOR_GEARING);
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
