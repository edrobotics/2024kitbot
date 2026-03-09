package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class IntakeSmartArmsCommand extends Command {
    private boolean positiveDirection = true; // true for 90 degrees, false for -90 degrees

    public IntakeSmartArmsCommand() {
      addRequirements(Robot.intakeArms);
    }
          
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      boolean buttonIntakeArms = Robot.m_oi.getCopilotIntakeArms();
      double position = Robot.intakeArms.getPosition();

      positiveDirection = buttonIntakeArms ? !positiveDirection : positiveDirection; // toggle direction if button is pressed

      // Change besically everything with ...climber... to ...intake... and vise versa to make the intake arms command work with the same button as the climb command, but with priority given to the climb command when the intake arms are not in
      double targetRotations;
      boolean intakePriority = Robot.intakeSmartArmsCommand.intakePriority;

      boolean intakeIn = Functions.roundToDecimalPlaces(Robot.intakeArms.getPosition(),2)==0;
      if (buttonIntakeArms && !buttonEncoderMotorWasPressed) {
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
      
      //Sets either positive or negative target rotations
      double targetRotations = positiveDirection ? -Constants.INTAKE_ARMS_TARGET_ROTATIONS * Constants.Intake_Arms_GEARING : 0;

      Robot.intakeArms.setIntakeArmsMotors(Math.abs(targetRotations - position) > Constants.INTAKE_ARMS_DEADBAND ? Constants.INTAKE_ARMS_SPEED * (targetRotations - position) : 0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      Robot.intakeArms.stopIntakeArmsMotors();
    }
}