package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Functions;

public class IntakeArmsCommand extends Command {
    private double negativeTargetDegrees = -90;
    private double positiveTargetDegrees = 720;
    private double speed = Constants.INTAKE_ARMS_SPEED;
    private boolean isRunning = false;
    private boolean shouldStart = true;
    private boolean wasPressed = false;
    private boolean positiveDirection = true; // true for positive degrees, false for negative degrees
    private double startPosition;

    public IntakeArmsCommand() {
      addRequirements(Robot.intakeArms);
    }
          
    @Override
    public void initialize() {
      Robot.intakeArms.resetEncoders();
      isRunning = false;
    }

    @Override
    public void execute() {
        double deltaPosition = Robot.intakeArms.getPosition() - startPosition;
        boolean buttonIntakeArms = false;
        if (Constants.copilotControllerType == "ps4") {
          buttonIntakeArms = Robot.m_oi.GetCopilotRawButton(Constants.ps4_buttonTriangle);
        } else if (Constants.copilotControllerType == "logitech") {
          buttonIntakeArms = Robot.m_oi.GetCopilotRawButton(Constants.logitech_buttonY);
        }

        //Sets either positive or negative speed
        speed = positiveDirection ? Math.abs(speed) : -Math.abs(speed);

        //Sets 'min' and 'max' values to either work either clockwise or counterclockwise
        double min = positiveDirection ? 0.01 : -0.1;
        double max = positiveDirection ? 0.1 : -0.01;
        double targetRotations = !positiveDirection ? positiveTargetDegrees/360 : negativeTargetDegrees/360; // converts degrees to rotations (1 rotation = 1.0; 90 degree rotation = 0.25 etc.)

        //Used to know if the intake arm motors will be started during this scheduler run
        shouldStart = (buttonIntakeArms && !wasPressed && !isRunning);
        //So that we can know during the next scheduler run if the button was pressed this time or not
        wasPressed = buttonIntakeArms;
        if (shouldStart) {
          isRunning = true;
          startPosition = Robot.intakeArms.getPosition();
        }
        if (isRunning) {
          Robot.intakeArms.setIntakeArmsMotors(Functions.clamp((targetRotations - Math.abs(deltaPosition)) * speed, min, max)); // Gradually degreeses the motor velocity for more precise turning
        }

        // Debugging
        System.out.println("Intake Arms Position: " + (Functions.roundToDecimalPlaces(deltaPosition, 1)));
        System.out.println("Intake Arms Reseted Position" + deltaPosition);
        System.out.println("Target Position" + targetRotations);
        System.out.println("Intake Arms Degrees: " + (Robot.intakeArms.getPosition() * 360));
        System.out.println("Procent of target degrees: " + (deltaPosition * 100 / targetRotations));
        System.out.println("Has reached the desired rotations?: " + (Functions.roundToDecimalPlaces(deltaPosition, 1) == targetRotations));

        //Make so that the motors will stop next time
        if (Functions.roundToDecimalPlaces(deltaPosition, 0) == targetRotations && isRunning) { // Checks if the desired rotations has been met
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