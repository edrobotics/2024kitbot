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

//StudicaLib
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//Pathplanner
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
  
  // Simulated encoder values for PathPlanner in simulation
  private double simulatedLeftDistance = 0;
  private double simulatedRightDistance = 0;
  
  // Track whether PathPlanner was successfully configured
  private static boolean autoBuilderConfigured = false;
  
  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }
  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }
  public double getRightPosition() {
    return -rightEncoder.getPosition();
  }
  public double getRightSpeed() {
    return -rightEncoder.getVelocity();
  }

  // ── Kinematics & Odometry ──────────────────────────────────────────────────

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry;

  // Field visualization widget shown in Glass / Shuffleboard.
  private final Field2d field2d = new Field2d();

  // ── Constructor ────────────────────────────────────────────────────────────

  public DriveTrain() {
    // Configure encoder conversion factors on the leader motors so that position
    // and velocity readings are already in meters / meters-per-second.
    SparkMaxConfig leftEncoderConfig = new SparkMaxConfig();
    leftEncoderConfig.encoder.positionConversionFactor(Constants.DRIVETRAIN_ENCODER_POSITION_CONVERSION);
    leftEncoderConfig.encoder.velocityConversionFactor(Constants.DRIVETRAIN_ENCODER_VELOCITY_CONVERSION);

    SparkMaxConfig rightEncoderConfig = new SparkMaxConfig();
    rightEncoderConfig.encoder.positionConversionFactor(Constants.DRIVETRAIN_ENCODER_POSITION_CONVERSION);
    rightEncoderConfig.encoder.velocityConversionFactor(Constants.DRIVETRAIN_ENCODER_VELOCITY_CONVERSION);

    leftMotor1.configure(leftEncoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor1.configure(rightEncoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();

    // Initialize odometry at the origin facing forward.
    odometry = new DifferentialDriveOdometry( Robot.gyroscope.getHeading(), getLeftDistanceMeters(), getRightDistanceMeters(), new Pose2d());

    SmartDashboard.putData("Field", field2d);

    configureAutoBuilder();
  }

  // ── PathPlanner Setup ──────────────────────────────────────────────────────

  private void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          this::driveRobotRelative,
          new PPLTVController(Constants.LOOP_PERIOD_SECONDS),
          config,
          () -> {
            // Mirror paths to the red alliance side of the field.
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);
      autoBuilderConfigured = true;
      DriverStation.reportWarning("PathPlanner AutoBuilder successfully configured", false);
    } catch (Exception e) {
      // PathPlanner requires a valid robot config JSON exported from the GUI.
      // A missing config file is expected and harmless during simulation.
      autoBuilderConfigured = false;
      DriverStation.reportWarning("PathPlanner config failed: " + e.getMessage(), false);
    }
  }
  
  public static boolean isAutoBuilderConfigured() {
    return autoBuilderConfigured;
  }

  // ── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // Integrate wheel speeds to update simulated distances
    DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
    simulatedLeftDistance += wheelSpeeds.leftMetersPerSecond * Constants.LOOP_PERIOD_SECONDS;
    simulatedRightDistance += wheelSpeeds.rightMetersPerSecond * Constants.LOOP_PERIOD_SECONDS;
    
    odometry.update(Robot.gyroscope.getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    field2d.setRobotPose(getPose());

    SmartDashboard.putNumber("Robot X",        getPose().getX());
    SmartDashboard.putNumber("Robot Y",        getPose().getY());
    SmartDashboard.putNumber("Robot Heading",  Robot.gyroscope.getHeading().getDegrees());
    SmartDashboard.putNumber("Left Distance",  getLeftDistanceMeters());
    SmartDashboard.putNumber("Right Distance", getRightDistanceMeters());
    SmartDashboard.putNumber("Left Speed m/s", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed m/s", wheelSpeeds.rightMetersPerSecond);
  }

  // ── Motor Control ──────────────────────────────────────────────────────────

  /** Sets the left side speed [-1, 1] after applying the global speed reduction. */
  public void setLeftMotors(double speed) {
    speed = Functions.clamp(speed);
    leftMotor1.set(speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
    leftMotor2.set(speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
  }

  /** Sets the right side speed [-1, 1] after applying the global speed reduction. */
  public void setRightMotors(double speed) {
    speed = Functions.clamp(speed);
    rightMotor1.set(-speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
    rightMotor2.set(-speed * Constants.DRIVETRAIN_SPEED_REDUCTION);
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

  public void autoRotate(double targetDeltaDegrees, double deltaDegrees) {
    double speed = (targetDeltaDegrees-deltaDegrees)/Math.abs(targetDeltaDegrees);
    setLeftMotors(speed*Constants.AUTO_TURN_SPEED+0.05);
    setRightMotors(-speed*Constants.AUTO_TURN_SPEED+0.05);
  }

  public void autoDriveMotors(double targetX, double targetY, double currentX, double currentY, double startX, double startY) {
    double totalDistance = Math.sqrt(Math.pow(startX - targetX, 2) + Math.pow(startY - targetY, 2));
    double currentDistance = Math.sqrt(Math.pow(currentX - targetX, 2) + Math.pow(currentY - targetY, 2));
    double speed = (currentDistance) / totalDistance;
    setLeftMotors(speed+0.1);
    setRightMotors(speed+0.1);
  }

  /**
   * Drives the robot at the requested chassis speeds. Called by PathPlanner during autonomous.
   * Bypasses the driver speed reduction so PathPlanner has full, unscaled authority over velocity.
   *
   * TODO: Replace the proportional normalization below with a PID velocity controller
   *       for more accurate path tracking (requires working encoders first).
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    // Normalize velocity to a [-1, 1] motor output using the configured max velocity.
    double leftOutput  = wheelSpeeds.leftMetersPerSecond  / Constants.MAX_VELOCITY_MPS;
    double rightOutput = wheelSpeeds.rightMetersPerSecond / Constants.MAX_VELOCITY_MPS;
    
    // Clamp outputs to valid motor range [-1, 1]
    leftOutput = Functions.clamp(leftOutput);
    rightOutput = Functions.clamp(rightOutput);
    
    // Log requested speeds for diagnostics
    SmartDashboard.putNumber("PathPlanner Vx m/s", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("PathPlanner Omega rad/s", speeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Wheel Left m/s", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Wheel Right m/s", wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Left Motor Output", leftOutput);
    SmartDashboard.putNumber("Right Motor Output", rightOutput);

    setRawOutputs(leftOutput, rightOutput);
  }

  /** Stops all drive motors immediately. */
  public void stop() {
    setLeftMotors(0);
    setRightMotors(0);
  }

  // ── Sensors ────────────────────────────────────────────────────────────────


  /** Returns the distance the left side has traveled in meters since the last encoder reset. */
  public double getLeftDistanceMeters() {
    // For simulation, integrate wheel speeds over time to estimate distance
    return simulatedLeftDistance;
  }

  /** Returns the distance the right side has traveled in meters since the last encoder reset. */
  public double getRightDistanceMeters() {
    // For simulation, integrate wheel speeds over time to estimate distance
    return simulatedRightDistance;
  }

  /** Resets both drive encoders to zero. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
  }

  /** Returns the current wheel speeds in meters per second. */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // For simulation, derive wheel speeds from motor outputs scaled to max velocity
    double leftSpeed = -leftMotor1.get() * Constants.MAX_VELOCITY_MPS;
    double rightSpeed = rightMotor1.get() * Constants.MAX_VELOCITY_MPS;
    
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  /** Returns the current chassis speeds derived from wheel speeds. */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  // ── Odometry ──────────────────────────────────────────────────────────────

  /** Returns the robot's estimated field-relative pose. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the robot's odometry to the given pose and zeroes the encoders. */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(Robot.gyroscope.getHeading(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
  }

  /** Returns the drivetrain kinematics object (used by PathPlanner and other commands). */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  // ── Private Helpers ────────────────────────────────────────────────────────

  /**
   * Writes motor outputs directly, bypassing the driver speed reduction.
   * Used by PathPlanner (driveRobotRelative) to preserve precise velocity authority.
   */
  private void setRawOutputs(double leftOutput, double rightOutput) {
    // Set motor outputs directly (already in [-1, 1] range and clamped)
    // Left motors are negated because they're mounted backwards
    leftMotor1.set(-leftOutput);
    leftMotor2.set(-leftOutput);
    rightMotor1.set(rightOutput);
    rightMotor2.set(rightOutput);
  }
}
