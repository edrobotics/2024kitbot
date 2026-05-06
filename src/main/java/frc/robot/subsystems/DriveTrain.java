// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//RevLib
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Functions;

public class DriveTrain extends SubsystemBase {

  // ── Motor Controllers ──────────────────────────────────────────────────────
  private final SparkMax leftMotor1  = new SparkMax(Constants.LMOTOR1ID,  MotorType.kBrushless);
  private final SparkMax leftMotor2  = new SparkMax(Constants.LMOTOR2ID,  MotorType.kBrushless);
  private final SparkMax rightMotor1 = new SparkMax(Constants.RMOTOR1ID, MotorType.kBrushless);
  private final SparkMax rightMotor2 = new SparkMax(Constants.RMOTOR2ID, MotorType.kBrushless);
  
  private final RelativeEncoder leftEncoder  = leftMotor1.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();
  
  public double getLeftPosition() { return leftEncoder.getPosition(); }
  public double getLeftSpeed() { return leftEncoder.getVelocity(); }
  public double getRightPosition() { return rightEncoder.getPosition(); }
  public double getRightSpeed() { return rightEncoder.getVelocity(); }

  public DriveTrain() {
    // Configure encoder conversion factors on the leader motors so that position
    // and velocity readings are already in meters / meters-per-second.
    SparkMaxConfig leftConfig1 = new SparkMaxConfig();
    leftConfig1.encoder.positionConversionFactor(Constants.DRIVETRAIN_ENCODER_POSITION_CONVERSION);
    leftConfig1.encoder.velocityConversionFactor(Constants.DRIVETRAIN_ENCODER_VELOCITY_CONVERSION);
    leftConfig1.inverted(false);
    SparkMaxConfig leftConfig2 = new SparkMaxConfig();
    leftConfig2.encoder.positionConversionFactor(Constants.DRIVETRAIN_ENCODER_POSITION_CONVERSION);
    leftConfig2.encoder.velocityConversionFactor(Constants.DRIVETRAIN_ENCODER_VELOCITY_CONVERSION);
    leftConfig2.inverted(false);
    SparkMaxConfig rightConfig1 = new SparkMaxConfig();
    rightConfig1.encoder.positionConversionFactor(Constants.DRIVETRAIN_ENCODER_POSITION_CONVERSION);
    rightConfig1.encoder.velocityConversionFactor(Constants.DRIVETRAIN_ENCODER_VELOCITY_CONVERSION);
    rightConfig1.inverted(true);
    SparkMaxConfig rightConfig2 = new SparkMaxConfig();
    rightConfig2.encoder.positionConversionFactor(Constants.DRIVETRAIN_ENCODER_POSITION_CONVERSION);
    rightConfig2.encoder.velocityConversionFactor(Constants.DRIVETRAIN_ENCODER_VELOCITY_CONVERSION);
    rightConfig2.inverted(true);

    leftMotor1.configure(leftConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor2.configure(leftConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor1.configure(rightConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor2.configure(rightConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();
  }

  @Override
  public void periodic() {}

  /** Sets the left side speed [-1, 1] after applying the global speed reduction. */
  public void setLeftMotors(double speed) {
    speed = Functions.clamp(speed);
    leftMotor1.set(speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
    leftMotor2.set(speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
    SmartDashboard.putNumber("Left motor input", Functions.clamp(speed));
  }

  /** Sets the right side speed [-1, 1] after applying the global speed reduction. */
  public void setRightMotors(double speed) {
    speed = Functions.clamp(speed);
    rightMotor1.set(speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
    rightMotor2.set(speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
    SmartDashboard.putNumber("Right motor input", Functions.clamp(speed));
  }

  public void setLeftMotorsBoosted(double speed) {
    leftMotor1.set(Functions.clamp(speed * Constants.DRIVETRAIN_SPEED_REDUCTION));
    leftMotor2.set(Functions.clamp(speed * Constants.DRIVETRAIN_SPEED_REDUCTION));
  }

  public void setRightMotorsBoosted(double speed) {
    rightMotor1.set(Functions.clamp(speed * Constants.DRIVETRAIN_SPEED_REDUCTION));
    rightMotor2.set(Functions.clamp(speed * Constants.DRIVETRAIN_SPEED_REDUCTION));
  }

  public void setLeftMotorsSmoothly(double speed) {
    speed = Functions.clamp(speed);
    double leftVelocity = getLeftSpeed();
    if(Math.abs(leftVelocity) < Constants.DRIVETRAIN_MAX_INPUT_AT)
    {
      double m = Constants.DRIVETRAIN_MIN_INPUT;
      double k = (1-m)/Constants.DRIVETRAIN_MAX_INPUT_AT;
      setLeftMotors(speed * (Math.abs(k*leftVelocity)+m));
    }
    else
    {
      setLeftMotors(speed);
    }
  }
  public void setRightMotorsSmoothly(double speed) {
    speed = Functions.clamp(speed);
    double rightVelocity = getRightSpeed();
    if(Math.abs(rightVelocity) < Constants.DRIVETRAIN_MAX_INPUT_AT)
    {
      double m = Constants.DRIVETRAIN_MIN_INPUT;
      double k = (1-m)/Constants.DRIVETRAIN_MAX_INPUT_AT;
      setRightMotors(speed * (Math.abs(k*rightVelocity)+m));
    }
    else
    {
      setRightMotors(speed);
    }
  }

  /** Arcade drive: xSpeed is forward/backward [-1, 1], rot is turn rate [-1, 1]. */
  public void arcadeDrive(double xSpeed, double rotation) {
    setLeftMotors(xSpeed + rotation);
    setRightMotors(xSpeed - rotation);
  }

  /** Stops all drive motors immediately. */
  public void stop() {
    setLeftMotors(0);
    setRightMotors(0);
  }

  /** Resets both drive encoders to zero. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}