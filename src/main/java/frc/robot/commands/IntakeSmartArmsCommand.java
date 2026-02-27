package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class IntakeSmartArmsCommand extends Command {
    private double targetRotations;
    private double speed = Constants.INTAKE_ARMS_SPEED;
    private boolean isRunning = false;
    private boolean shouldStart = false;
    private boolean positiveDirection = true; // true for 90 degrees, false for -90 degrees
    private double Position;
    private int decimals = 1;
    private double gearing = Constants.Intake_Arms_GEARING;

    public IntakeSmartArmsCommand() {
      addRequirements(Robot.intakeArms);
    }
          
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      boolean buttonIntakeArms = Robot.m_oi.getCopilotIntakeArms();
      
      //Sets either positive or negative target rotations
      targetRotations = positiveDirection ? Constants.INTAKE_ARMS_TARGET_ROTATIONS*gearing : 0;
      //Used to know if the intake arm motors will be started during this scheduler run
      shouldStart = (buttonIntakeArms && !isRunning);
      if (shouldStart) {
        isRunning = true;
        Position = 0;
      }
      if (isRunning) {
        Position = Robot.intakeArms.getPosition();
        Robot.intakeArms.setIntakeArmsMotors(-speed*(targetRotations-Position));
      }

      //Make so that the motors will stop next time
      if (Functions.roundToDecimalPlaces(Position, decimals) == Functions.roundToDecimalPlaces(targetRotations, decimals)) {
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