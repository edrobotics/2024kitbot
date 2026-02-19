// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Optional;

/**
 * Stub subsystem for camera-based AprilTag pose estimation.
 *
 * <p>This subsystem is structured so that PhotonVision (or another vision
 * library) can be dropped in with minimal refactoring:
 * <ol>
 *   <li>Add the PhotonLib vendor dependency.</li>
 *   <li>Create a {@code PhotonCamera} and {@code PhotonPoseEstimator} in the constructor.</li>
 *   <li>In {@link #periodic()}, poll the estimator and store the latest result.</li>
 *   <li>Have the drivetrain call {@link #getEstimatedPose()} each cycle to feed
 *       the result into a {@code DifferentialDrivePoseEstimator}.</li>
 * </ol>
 */
public class VisionSubsystem extends SubsystemBase {

  // TODO: Add PhotonVision fields once the vendor dependency is installed:
  //   private final PhotonCamera camera = new PhotonCamera(Constants.CAMERA_NAME);
  //   private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
  //       AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
  //       PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
  //       Constants.ROBOT_TO_CAMERA);

  private Pose3d lastEstimatedPose = null;
  private double lastTimestampSeconds = -1;
  private boolean hasTarget = false;

  public VisionSubsystem() {
    // TODO: Initialize PhotonCamera and PhotonPoseEstimator here once PhotonLib is added.
  }

  @Override
  public void periodic() {
    // TODO: Replace this stub with actual PhotonVision pose estimation:
    //   var result = poseEstimator.update();
    //   if (result.isPresent()) {
    //     var estimatedPose = result.get();
    //     lastEstimatedPose = estimatedPose.estimatedPose;
    //     lastTimestampSeconds = estimatedPose.timestampSeconds;
    //     hasTarget = true;
    //   } else {
    //     hasTarget = false;
    //   }

    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
    if (lastEstimatedPose != null) {
      SmartDashboard.putNumber("Vision/X", lastEstimatedPose.getX());
      SmartDashboard.putNumber("Vision/Y", lastEstimatedPose.getY());
      SmartDashboard.putNumber("Vision/Z", lastEstimatedPose.getZ());
    }
  }

  /**
   * Returns the most recent vision-estimated pose if a valid target was seen.
   * The drivetrain should call this each cycle and, when present, feed the pose
   * into its {@code DifferentialDrivePoseEstimator.addVisionMeasurement()}.
   *
   * @return the estimated 3D pose, or empty if no target is visible
   */
  public Optional<Pose3d> getEstimatedPose() {
    if (hasTarget && lastEstimatedPose != null) {
      return Optional.of(lastEstimatedPose);
    }
    return Optional.empty();
  }

  /**
   * Returns the timestamp of the last valid pose estimate in seconds.
   * Pass this to {@code addVisionMeasurement()} alongside the pose.
   */
  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  /**
   * Returns the last estimated pose as a Pose2d for 2D odometry fusion.
   *
   * @return the estimated 2D pose, or empty if no target is visible
   */
  public Optional<Pose2d> getEstimatedPose2d() {
    return getEstimatedPose().map(Pose3d::toPose2d);
  }

  /** Returns whether the camera currently sees an AprilTag target. */
  public boolean hasTarget() {
    return hasTarget;
  }
}
