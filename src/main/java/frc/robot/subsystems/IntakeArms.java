// Copyright (c) FIRST and other WPILib contributors../
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArms extends SubsystemBase {
  private SparkMax leftIntakeArmsMotor = new SparkMax(Constants.leftIntakeMotorArmID, MotorType.kBrushless);
  private SparkMax rightIntakeArmsMotor = new SparkMax(Constants.rightIntakeMotorArmID, MotorType.kBrushless);

  // encoders - if using brushed motors you'll need external encoders instead
  private final RelativeEncoder leftEncoder = leftIntakeArmsMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightIntakeArmsMotor.getEncoder();
  public double getPosition() {
        // return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        return leftEncoder.getPosition();
    }
  
  public IntakeArms() {
    // any init if needed

    //encoder config
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(Constants.ENCODER_POSITION_CONVERSION);
    config.encoder.velocityConversionFactor(Constants.ENCODER_VELOCITY_CONVERSION);

    leftIntakeArmsMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightIntakeArmsMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();
  }

  @Override
  public void periodic() {}

  // Do NOT touch the following function unless it is needed. It makes sure both the motors run in the same direction
  public void setIntakeArmsMotors(double speed) {
    leftIntakeArmsMotor.set(-speed);
    rightIntakeArmsMotor.set(speed);
  }
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  public void stopIntakeArmsMotors() {
    leftIntakeArmsMotor.set(0);
    rightIntakeArmsMotor.set(0);
  }
}
