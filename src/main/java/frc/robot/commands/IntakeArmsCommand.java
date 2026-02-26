package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class IntakeArmsCommand extends Command {
    private double targetRotations = Constants.INTAKE_ARMS_TARGET_ROTATIONS;
    private double speed = Constants.INTAKE_ARMS_SPEED;
    private boolean isRunning = false;
    private boolean shouldStart = false;
    private boolean positiveDirection = true; // true for 90 degrees, false for -90 degrees
    private double startPosition;
    private double deltaPosition = 0;

    public IntakeArmsCommand() {
      addRequirements(Robot.intakeArms);
    }
          
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      boolean buttonIntakeArms = Robot.m_oi.getCopilotIntakeArms();
      
      //Sets either positive or negative speed
      speed = positiveDirection ? -Math.abs(speed) : Math.abs(speed);
      //Used to know if the intake arm motors will be started during this scheduler run
      shouldStart = (buttonIntakeArms && !isRunning);
      if (shouldStart) {
        isRunning = true;
        startPosition = Robot.intakeArms.getPosition();
        deltaPosition = 0;
      }
      if (isRunning) {
        deltaPosition = Math.abs(Robot.intakeArms.getPosition()-startPosition);
        Robot.intakeArms.setIntakeArmsMotors(speed*Constants.intakeSpeedReduction + speed*(targetRotations-deltaPosition)/targetRotations*(1-Constants.intakeSpeedReduction));
      }
      //Make so that the motors will stop next time
      if (deltaPosition >= Math.abs(targetRotations) && ((Robot.intakeArms.getPosition() - startPosition) > 0) == positiveDirection) {
        isRunning = false;
        Robot.intakeArms.stopIntakeArmsMotors();
        positiveDirection = !positiveDirection; // toggle direction for next time
      }
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