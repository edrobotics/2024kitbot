package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeArmsCommand extends Command {
    private double targetDegrees = Constants.INTAKE_ARMS_TARGET_DEGREES;
    private double speed = Constants.INTAKE_ARMS_SPEED;
    private boolean isRunning = false;
    private boolean shouldStart = true;
    private boolean wasPressed = false;
    private boolean positiveDirection = true; // true for 90 degrees, false for -90 degrees
    private double startPosition;

    public IntakeArmsCommand() {
      addRequirements(Robot.intakeArms);
    }
          
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean buttonIntakeArms = false;
        if (Constants.copilotControllerType.equals("ps4")) {
          buttonIntakeArms = Robot.m_oi.GetCopilotRawButton(Constants.ps4_buttonTriangle);
        } else if (Constants.copilotControllerType.equals("logitech")) {
          buttonIntakeArms = Robot.m_oi.GetCopilotRawButton(Constants.logitech_buttonY);
        }

        //Sets either positive or negative speed
        speed = positiveDirection ? Math.abs(speed) : -Math.abs(speed);
        //Used to know if the intake arm motors will be started during this scheduler run
        shouldStart = (buttonIntakeArms && !wasPressed && !isRunning);
        //So that we can know during the next scheduler run if the button was pressed this time or not
        wasPressed = buttonIntakeArms;
        if (shouldStart) {
            isRunning = true;
            startPosition = Robot.intakeArms.getPosition();
        }
        if (isRunning) {
            Robot.intakeArms.setIntakeArmsMotors(speed);
        }
        //Make so that the motors will stop next time
        if ((Math.abs(Robot.intakeArms.getPosition() - startPosition) >= Math.abs(targetDegrees / 360.0)) && ((Robot.intakeArms.getPosition() - startPosition) > 0) == positiveDirection) {
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