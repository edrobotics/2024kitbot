// Copyright (c) FIRST and other WPILib contributors../
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//RevLib
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class IntakeArms extends SubsystemBase {
  private SparkMax leftIntakeArmsMotor = new SparkMax(Constants.leftIntakeMotorArmID, MotorType.kBrushless);
  private SparkMax rightIntakeArmsMotor = new SparkMax(Constants.rightIntakeMotorArmID, MotorType.kBrushless);

  // encoders - if using brushed motors you'll need external encoders instead
  private final RelativeEncoder leftEncoder = leftIntakeArmsMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightIntakeArmsMotor.getEncoder();
  public double getRightPosition() {
    return rightEncoder.getPosition();
  }
  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }
  public double getPosition() {
    return (leftEncoder.getPosition() - rightEncoder.getPosition())/2;
  }

  // rotates the intake arms to either be up (positiveDirection == true) or down (positiveDirection == false)
  public void rotateIntakeArms(boolean positiveDirection) {
    double rightPosition = Robot.intakeArms.getRightPosition();
    double leftPosition = Robot.intakeArms.getLeftPosition();
    double targetRotations = positiveDirection ? Constants.INTAKE_ARMS_TARGET_ROTATIONS : 0;
    
    Robot.intakeArms.setRightIntakeArmsMotor(Math.abs(targetRotations + rightPosition) > Constants.INTAKE_ARMS_DEADBAND ? -Constants.INTAKE_ARMS_SPEED * (targetRotations + rightPosition) : 0);
    Robot.intakeArms.setLeftIntakeArmsMotor(Math.abs(targetRotations - leftPosition) > Constants.INTAKE_ARMS_DEADBAND ? Constants.INTAKE_ARMS_SPEED * (targetRotations - leftPosition) : 0);
  }
  
  
  public IntakeArms() {
    // any init if needed

    //encoder config
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(Constants.INTAKE_ARMS_GEARING);

    leftIntakeArmsMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightIntakeArmsMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();
  }

  @Override
  public void periodic() {}

  // Do NOT touch the following function unless it is needed. It makes sure both the motors run in the same direction
  public void setRightIntakeArmsMotor(double speed) {
    rightIntakeArmsMotor.set(Functions.clamp(speed, -0.5, 0.5));
  }
  public void setLeftIntakeArmsMotor(double speed) {
    leftIntakeArmsMotor.set(Functions.clamp(speed, -0.5, 0.5));
  }
  public void setIntakeArmsMotors(double speed) {
    setRightIntakeArmsMotor(speed);
    setLeftIntakeArmsMotor(-speed);
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
