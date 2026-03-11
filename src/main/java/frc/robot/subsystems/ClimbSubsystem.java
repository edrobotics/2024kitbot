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

  public void stop() {
    climbWinchMotor.set(0);
    climbEncoderMotor.set(0);
  }

  @Override
  public void periodic() {}
}
