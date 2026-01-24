// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class TankDrive extends Command {
  public TankDrive() {
    addRequirements(Robot.driveTrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // GTA-style driving controls:
    // - Right trigger (RT) = accelerate forward
    // - Left trigger (LT) = brake/reverse
    // - Left joystick X = steering
    double steering = Robot.m_oi.GetDriverRawAxis(Constants.LEFT_STICK_X);
    double rightTrigger = Robot.m_oi.GetDriverRawAxis(Constants.RIGHT_TRIGGER);
    double leftTrigger = Robot.m_oi.GetDriverRawAxis(Constants.LEFT_TRIGGER);

    // Calculate speed: RT accelerates, LT brakes/reverses
    double speed = rightTrigger - leftTrigger;

    // Apply arcade drive: speed with steering differential
    Robot.driveTrain.setLeftMotors(speed - steering);
    Robot.driveTrain.setRightMotors(speed + steering);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setLeftMotors(0);
    Robot.driveTrain.setRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
