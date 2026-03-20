package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;
 
public class IntakeSmartArmsCommand extends Command {
    public boolean positiveDirection = false; // true for 90 degrees, false for -90 degrees
 
    public IntakeSmartArmsCommand() {
      addRequirements(Robot.intakeArms);
    }
         
    @Override
    public void initialize() {
      positiveDirection = false;
    }
 
    @Override
    public void execute() {
      double manualDrive = Robot.m_oi.getCopilotManualIntakeArms();
      if(Math.abs(manualDrive) > 0.1) {
        Robot.intakeArms.setLeftIntakeArmsMotor(manualDrive);
        Robot.intakeArms.setRightIntakeArmsMotor(manualDrive);
        Robot.driveIntakeArmsManually = true;
      }
      else if(Robot.driveIntakeArmsManually) {
        Robot.intakeArms.setLeftIntakeArmsMotor(0);
        Robot.intakeArms.setRightIntakeArmsMotor(0);
      }
      
      boolean buttonIntakeArms = Robot.m_oi.getCopilotClimber();
      if(buttonIntakeArms && Robot.driveIntakeArmsManually) {
        Robot.driveIntakeArmsManually = false;
        positiveDirection = false;
      }
      //if(!Robot.driveIntakeArmsManually) {
        //double rightPosition = -Robot.intakeArms.getRightPosition();
        //double leftPosition = Robot.intakeArms.getLeftPosition();
        //boolean climberIn = Functions.roundToDecimalPlaces(Robot.climber.getWinchPosition(),2) == 0; // returns true when climber arms are retracted

        //if (buttonIntakeArms) {
          //positiveDirection = false;
        //}

        /*
        // makes the intake arms return to zero if climber arms is out
        if (!climberIn && buttonIntakeArms) {
          positiveDirection = false;
        }
        */

        // toggles the intake arms if climber arms are in
        /*
        if (climberIn && buttonIntakeArms) {
          positiveDirection = !positiveDirection;
        }
        */
        //double targetRotations = positiveDirection ? Constants.INTAKE_ARMS_TARGET_ROTATIONS : 0;
      
        //Robot.intakeArms.setRightIntakeArmsMotor(Math.abs(targetRotations - rightPosition) > Constants.INTAKE_ARMS_DEADBAND ? -Constants.INTAKE_ARMS_SPEED * (targetRotations + rightPosition) : 0);
        //Robot.intakeArms.setLeftIntakeArmsMotor(Math.abs(targetRotations - leftPosition) > Constants.INTAKE_ARMS_DEADBAND ? Constants.INTAKE_ARMS_SPEED * (targetRotations - leftPosition) : 0);
        

        //Robot.intakeArms.rotateIntakeArms(positiveDirection); // rotates intake arms to either be up (positiveDirection == false) or down (positiveDirection == true)
      //}
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