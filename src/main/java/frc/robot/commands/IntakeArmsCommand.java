package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeArmsCommand extends Command {
    private double targetDegrees = Constants.INTAKE_ARMS_TARGET_DEGREES;
    private double targetRotations = targetDegrees / 360.0; // converts degrees to rotations (1 rotation = 1.0)
    private double speed = Constants.INTAKE_ARMS_SPEED;
    private boolean isRunning = false;
    private boolean shouldStart = true;
    private boolean wasPressed = false;
    private boolean positiveDirection = true; // true for positive degrees, false for negative degrees
    private double startPosition;
    private double deltaPosition = Robot.intakeArms.getPosition() - startPosition;

    public IntakeArmsCommand() {
      addRequirements(Robot.intakeArms);
    }
          
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean buttonIntakeArms = false;
        if (Constants.copilotControllerType == "ps4") {
          buttonIntakeArms = Robot.m_oi.GetCopilotRawButton(Constants.ps4_buttonTriangle);
        } else if (Constants.copilotControllerType == "logitech") {
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
            if (positiveDirection) {
              Robot.intakeArms.setIntakeArmsMotors(Robot.functions.clamp((targetRotations - Math.abs(deltaPosition)) * speed, 0.01, 0.1)); // Gradually degreeses the motor velocity for more precise turning
            }
            if (!positiveDirection) {
              Robot.intakeArms.setIntakeArmsMotors(Robot.functions.clamp((targetRotations - Math.abs(deltaPosition)) * speed, -0.1, -0.01)); // Gradually degreeses the motor velocity for more precise turning
            }
        }

        // Debugging
        System.out.println("Intake Arms Position: " + (Robot.functions.roundToDecimalPlaces(deltaPosition, 3)));
        System.out.println("Intake Arms Reseted Position" + deltaPosition);
        System.out.println("Intake Arms Degrees: " + (Robot.intakeArms.getPosition() * 360));
        System.out.println("Procent of target degrees: " + Math.abs((deltaPosition * 100 / targetRotations)));
        System.out.println("Has reached the desired rotations?: " + (Robot.functions.roundToDecimalPlaces(deltaPosition, 3) == targetRotations));

        //Make so that the motors will stop next time
        if (Robot.functions.roundToDecimalPlaces(deltaPosition, 3) == targetRotations) { // Checks if the desired rotations has been met
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