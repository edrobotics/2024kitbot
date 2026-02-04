// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  // motors
  private final SparkMax leftMotor1 = new SparkMax(Constants.LMOTOR1ID, MotorType.kBrushed);
  private final SparkMax leftMotor2 = new SparkMax(Constants.LMOTOR2ID, MotorType.kBrushed);
  private final SparkMax rightMotor1 = new SparkMax(Constants.RMOTOR1ID, MotorType.kBrushed);
  private final SparkMax rightMotor2 = new SparkMax(Constants.RMOTOR2ID, MotorType.kBrushed);

  // encoders - if using brushed motors you'll need external encoders instead
  private final RelativeEncoder leftEncoder = leftMotor1.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();

  // navx gyro
  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

  // kinematics stuff
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry;

  // field viz for glass/shuffleboard
  private final Field2d field2d = new Field2d();

  public DriveTrain() {
    // encoder config - uncomment if using neos
    // leftEncoder.setPositionConversionFactor(Constants.ENCODER_POSITION_CONVERSION);
    // leftEncoder.setVelocityConversionFactor(Constants.ENCODER_VELOCITY_CONVERSION);
    // rightEncoder.setPositionConversionFactor(Constants.ENCODER_POSITION_CONVERSION);
    // rightEncoder.setVelocityConversionFactor(Constants.ENCODER_VELOCITY_CONVERSION);

    resetEncoders();

    // start at origin
    odometry = new DifferentialDriveOdometry(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        new Pose2d());

    SmartDashboard.putData("Field", field2d);

    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          this::driveRobotRelative,
          new PPLTVController(0.02),
          config,
          () -> {
            // flip path for red alliance
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);
    } catch (Exception e) {
      DriverStation.reportWarning("PathPlanner config failed (normal in simulation): " + e.getMessage(), false);
    }
  }

  @Override
  public void periodic() {
    // update position tracking
    odometry.update(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

    field2d.setRobotPose(getPose());

    // logging
    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Left Distance", getLeftDistanceMeters());
    SmartDashboard.putNumber("Right Distance", getRightDistanceMeters());
  }

  // basic drive methods

  public void setLeftMotors(double speed) {
    speed = Math.max(-1, Math.min(1, speed));
    leftMotor1.set(-speed * Constants.speedReduction);
    leftMotor2.set(-speed * Constants.speedReduction);
  }

  public void setRightMotors(double speed) {
    speed = Math.max(-1, Math.min(1, speed));
    rightMotor1.set(speed * Constants.speedReduction);
    rightMotor2.set(speed * Constants.speedReduction);
  }

  public void arcadeDrive(double xSpeed, double rot) {
    double leftSpeed = xSpeed + rot;
    double rightSpeed = xSpeed - rot;
    setLeftMotors(leftSpeed);
    setRightMotors(rightSpeed);
  }

  // used by pathplanner
  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

    // simple proportional - could add pid later for better tracking
    double leftOutput = leftVelocity / Constants.MAX_VELOCITY_MPS;
    double rightOutput = rightVelocity / Constants.MAX_VELOCITY_MPS;

    leftMotor1.set(-leftOutput);
    leftMotor2.set(-leftOutput);
    rightMotor1.set(rightOutput);
    rightMotor2.set(rightOutput);
  }

  public void stop() {
    leftMotor1.set(0);
    leftMotor2.set(0);
    rightMotor1.set(0);
    rightMotor2.set(0);
  }

  // sensor stuff

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle()); // negative for ccw positive
  }

  public void zeroHeading() {
    navx.reset();
  }

  public double getLeftDistanceMeters() {
    return leftEncoder.getPosition() * Constants.ENCODER_POSITION_CONVERSION;
  }

  public double getRightDistanceMeters() {
    return rightEncoder.getPosition() * Constants.ENCODER_POSITION_CONVERSION;
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftEncoder.getVelocity() * Constants.ENCODER_VELOCITY_CONVERSION,
        rightEncoder.getVelocity() * Constants.ENCODER_VELOCITY_CONVERSION);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  // odometry

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        pose);
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
}
