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

public class Intake extends SubsystemBase {
  // brushed motors
  private SparkMax intakeMotor = new SparkMax(Constants.intakeMotorId, MotorType.kBrushed);

  // brushless motors
  private SparkMax leftIntakeMotorArm = new SparkMax(Constants.leftIntakeMotorArmID, MotorType.kBrushless);
  private SparkMax rightIntakeMotorArm = new SparkMax(Constants.rightIntakeMotorArmID, MotorType.kBrushless);

  // encoders - if using brushed motors you'll need external encoders instead
  private final RelativeEncoder leftEncoder = leftIntakeMotorArm.getEncoder();
  private final RelativeEncoder rightEncoder = rightIntakeMotorArm.getEncoder();
  public double getPosition() {
        // return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        return leftEncoder.getPosition();
    }
  
  public Intake() {
    // any init if needed

    //encoder config
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(Constants.ENCODER_POSITION_CONVERSION);
    config.encoder.velocityConversionFactor(Constants.ENCODER_VELOCITY_CONVERSION);

    leftIntakeMotorArm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightIntakeMotorArm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Do NOT touch the following function unless it is needed. They make sure both the motors on the same side go in the same direction
  public void setIntakeMotor(double speed) {
    if(Constants.intakeConnected) {
      intakeMotor.set(speed);
    }
  }
  public void setIntakeArmMotors(double speed) {
    leftIntakeMotorArm.set(-speed);
    rightIntakeMotorArm.set(speed);
  }
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  public void stopIntakeArmMotors() {
    leftIntakeMotorArm.set(0);
    rightIntakeMotorArm.set(0);
  }
}
