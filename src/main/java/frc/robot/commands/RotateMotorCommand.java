package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class RotateMotorCommand extends Command {
    private final Intake intake;
    private final double targetRotations;
    private final double speed;
    private double startPosition;

    /**
     * @param intake your motor subsystem
     * @param degrees how many degrees to rotate
     * @param gearRatio motor-to-shaft ratio
     * @param speed motor speed (0-1)
     */
    public RotateMotorCommand(Intake intake, double degrees, double gearRatio, double speed) {
        /*
        this.intake = intake;
        this.targetRotations = (degrees / 360.0) * gearRatio;
        this.speed = speed;
        */
        addRequirements(intake);
    }
          
    @Override
    public void initialize() {
        startPosition = intake.getPosition();
    }

    @Override
    public void execute() {
        intake.setIntakeArmMotors(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getPosition() - startPosition) >= Math.abs(targetRotations);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeArmMotors();
    }
}