// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax climbWinchMotor = new SparkMax(Constants.CLIMBER_WINCH_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex climbEncoderMotor = new SparkFlex(Constants.CLIMBER_ENCODER_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder climbEncoderMotorEncoder = climbEncoderMotor.getEncoder();

  public ClimbSubsystem() {}

  public double getPosition() {
    double position = climbEncoderMotorEncoder.getPosition();
    return position;
  }

  public void runWinch(double speed) {
    speed = Functions.clamp(speed);
    climbWinchMotor.set(speed);
  }

  public void runEncoderMotor(double speed) {
    speed = Functions.clamp(speed);
    climbEncoderMotor.set(speed);
  }

  public void rotateClimbArms(boolean positiveDirection) {
    double position = Robot.climber.getPosition();

    double targetRotations = positiveDirection ? Constants.CLIMBER_TARGET_ROTATIONS : 0;
    double winchSpeed = Functions.roundToDecimalPlaces(position, 2) == targetRotations ? Constants.CLIMBER_WINCH_HOLD_IN_PLACE * (positiveDirection ? 1 : 0) : (targetRotations > position ? Constants.CLIMBER_WINCH_UP_SPEED : Constants.CLIMBER_WINCH_DOWN_SPEED);

    Robot.climber.runWinch(winchSpeed);
    Robot.climber.runEncoderMotor(Math.abs(targetRotations - position) > Constants.INTAKE_ARMS_DEADBAND ? -Constants.CLIMBER_ENCODER_SPEED * (targetRotations - position) : 0);
  }

  public void stop() {
    climbWinchMotor.set(0);
    climbEncoderMotor.set(0);
  }

  @Override
  public void periodic() {}
}
