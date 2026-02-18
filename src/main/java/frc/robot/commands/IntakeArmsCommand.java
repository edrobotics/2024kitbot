package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeArmsCommand extends Command {
    private double targetDegrees = 90;
    private double speed = 0.5;
    private boolean isRunning = false;
    private boolean shouldStart = true;
    private boolean wasPressed = false;
    private boolean direction = true; // true for 90 degrees, false for -90 degrees
    private double startPosition;

    public IntakeArmsCommand() {
        addRequirements(Robot.intakeArms);
    }
          
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean buttonIntakeArms;
        if(Constants.copilotControllerType == "ps4") {
          buttonIntakeArms = Robot.m_oi.GetCopilotRawButton(Constants.ps4_buttonTriangle);
        }
        else if(Constants.copilotControllerType == "logitech") {
          buttonIntakeArms = Robot.m_oi.GetCopilotRawButton(Constants.logitech_buttonY);
        }

        speed = direction ? Math.abs(speed) : -Math.abs(speed);
        shouldStart = (buttonIntakeArms && !wasPressed && !isRunning);
        wasPressed = buttonIntakeArms;
        if (shouldStart) {
            isRunning = true;
            startPosition = Robot.intakeArms.getPosition();
        }
        if (isRunning) {
            Robot.intakeArms.setIntakeArmsMotors(speed);
        }
        if ((Math.abs(Robot.intakeArms.getPosition() - startPosition) >= Math.abs(targetDegrees / 360.0)) && ((Robot.intakeArms.getPosition() - startPosition) > 0) == direction) {
            isRunning = false;
            Robot.intakeArms.stopIntakeArmsMotors();
            direction = !direction; // toggle direction for next time
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