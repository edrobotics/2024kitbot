// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
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

  // ── Motor Controllers ──────────────────────────────────────────────────────

  private final SparkMax leftMotor1  = new SparkMax(Constants.LMOTOR1ID,  MotorType.kBrushless);
  private final SparkMax leftMotor2  = new SparkMax(Constants.LMOTOR2ID,  MotorType.kBrushless);
  private final SparkMax rightMotor1 = new SparkMax(Constants.RMOTOR1ID, MotorType.kBrushless);
  private final SparkMax rightMotor2 = new SparkMax(Constants.RMOTOR2ID, MotorType.kBrushless);

  // TODO: Encoders are disabled because the drivetrain uses brushed motors, which
  //       cannot use the SparkMax's built-in relative encoder. To re-enable odometry:
  //       1. Wire external quadrature encoders into the SparkMax encoder ports.
  //       2. Uncomment the two encoder fields below.
  //       3. Restore encoder reads in getLeftDistanceMeters(), getRightDistanceMeters(),
  //          resetEncoders(), and getWheelSpeeds().
  //       4. (Optional) Once a VisionSubsystem is providing AprilTag poses, feed them
  //          into a DifferentialDrivePoseEstimator to fuse vision + wheel odometry.
  
  private final RelativeEncoder leftEncoder  = leftMotor1.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();
  
  // Simulated encoder values for PathPlanner in simulation
  private double simulatedLeftDistance = 0;
  private double simulatedRightDistance = 0;
  
  // Track whether PathPlanner was successfully configured
  private static boolean autoBuilderConfigured = false;
  
  public double getPosition() {
        // return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        return leftEncoder.getPosition();
    }

  // ── Gyro (NavX) ──────────────────────────────────────────────────────────
  // The gyro is accessed exclusively through getHeading(), getRotation2d(),
  // and zeroHeading(). To swap to a different gyro (e.g. Pigeon2) or extract
  // into a standalone GyroSubsystem, replace this field and update those three
  // methods — no other code references the navx directly.

  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

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
    // TODO: This encoder config only takes effect when encoders are enabled (see TODO above).
    SparkMaxConfig encoderConfig = new SparkMaxConfig();
    encoderConfig.encoder.positionConversionFactor(Constants.ENCODER_POSITION_CONVERSION);
    encoderConfig.encoder.velocityConversionFactor(Constants.ENCODER_VELOCITY_CONVERSION);

    leftMotor1.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor1.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();

    // Initialize odometry at the origin facing forward.
    odometry = new DifferentialDriveOdometry(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        new Pose2d());

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
    
    odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    field2d.setRobotPose(getPose());

    SmartDashboard.putNumber("Robot X",        getPose().getX());
    SmartDashboard.putNumber("Robot Y",        getPose().getY());
    SmartDashboard.putNumber("Robot Heading",  getHeading().getDegrees());
    SmartDashboard.putNumber("Left Distance",  getLeftDistanceMeters());
    SmartDashboard.putNumber("Right Distance", getRightDistanceMeters());
    SmartDashboard.putNumber("Left Speed m/s", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed m/s", wheelSpeeds.rightMetersPerSecond);
  }

  // ── Motor Control ──────────────────────────────────────────────────────────

  /** Sets the left side speed [-1, 1] after applying the global speed reduction. */
  public void setLeftMotors(double speed) {
    speed = clamp(speed);
    leftMotor1.set(-speed * Constants.speedReduction);
    leftMotor2.set(-speed * Constants.speedReduction);
  }

  /** Sets the right side speed [-1, 1] after applying the global speed reduction. */
  public void setRightMotors(double speed) {
    speed = clamp(speed);
    rightMotor1.set(speed * Constants.speedReduction);
    rightMotor2.set(speed * Constants.speedReduction);
  }

  /** Arcade drive: xSpeed is forward/backward [-1, 1], rot is turn rate [-1, 1]. */
  public void arcadeDrive(double xSpeed, double rot) {
    setLeftMotors(xSpeed + rot);
    setRightMotors(xSpeed - rot);
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
    leftOutput = clamp(leftOutput);
    rightOutput = clamp(rightOutput);
    
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

  /**
   * Returns the robot's current heading as a Rotation2d.
   * The NavX angle is negated so that counter-clockwise rotation is positive
   * (standard WPILib / field-coordinate convention).
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  /** Zeroes the NavX gyro. Call at match start or after a known-good heading is established. */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the robot's current heading as a Rotation2d.
   * Alias for {@link #getHeading()} — provided for API consistency with WPILib examples
   * that expect a getRotation2d() method.
   */
  public Rotation2d getRotation2d() {
    return getHeading();
  }

  /** Returns the distance the left side has traveled in meters since the last encoder reset. */
  public double getLeftDistanceMeters() {
    // TODO: Encoders disabled — brushed motors require external encoders. See class-level TODO.
    // For simulation, integrate wheel speeds over time to estimate distance
    return simulatedLeftDistance;
  }

  /** Returns the distance the right side has traveled in meters since the last encoder reset. */
  public double getRightDistanceMeters() {
    // TODO: Encoders disabled — brushed motors require external encoders. See class-level TODO.
    // For simulation, integrate wheel speeds over time to estimate distance
    return simulatedRightDistance;
  }

  /** Resets both drive encoders to zero. */
  public void resetEncoders() {
    // TODO: Encoders disabled — brushed motors require external encoders. See class-level TODO.
    
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
  }

  /** Returns the current wheel speeds in meters per second. */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // TODO: Encoders disabled — brushed motors require external encoders. See class-level TODO.
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
    odometry.resetPosition(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        pose);
  }

  /** Returns the drivetrain kinematics object (used by PathPlanner and other commands). */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  // ── Private Helpers ────────────────────────────────────────────────────────

  /**
   * Writes motor outputs directly, bypassing the driver speed reduction.
   * Used by PathPlanner (driveRobotRelative) to preserve precise velocity authority.
   * Left motors are negated because they are physically mounted inverted relative to the right.
   */
  private void setRawOutputs(double leftOutput, double rightOutput) {
    // Set motor outputs directly (already in [-1, 1] range and clamped)
    // Left motors are negated because they're mounted backwards
    leftMotor1.set(-leftOutput);
    leftMotor2.set(-leftOutput);
    rightMotor1.set(rightOutput);
    rightMotor2.set(rightOutput);
  }

  /** Clamps a motor speed value to the valid SparkMax input range [-1, 1]. */
  private double clamp(double speed) {
    return Math.max(-1, Math.min(1, speed));
  }
}
