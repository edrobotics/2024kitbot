// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Functions;
import frc.robot.Robot;
import frc.robot.Constants;

public class Gyroscope extends SubsystemBase {
  public Gyroscope() {}

  @Override
  public void periodic() {
  }

  //private AHRS navx = new AHRS(NavXComType.kMXP_SPI);
  private AHRS navx = new AHRS(NavXComType.kUSB1);
  
  /**
   * Returns the robot's current heading as a Rotation2d.
   * Alias for {@link #getHeading()} — provided for API consistency with WPILib examples
   * that expect a getRotation2d() method.
   */
  public Rotation2d getRotation2d() {
    return Robot.gyroscope.getHeading();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle());
  }
  public Rotation2d getCompassHeading() {
    return Rotation2d.fromDegrees(navx.getCompassHeading());
  }
  public void resetHeading() {
    navx.reset();
  }
  public float getPitch() { return navx.getPitch(); }
  public float getYaw() { return navx.getYaw(); }
  public float getRoll() { return navx.getRoll(); }
}



//The following explains what yaw, roll and pitch mean for those who don't remember
/*
              Roll
              <-->

         |-----------|
      ˄  |  ⟳-Yaw   |  ˄
Pitch |  |roboRIO 2.0|  | Pitch
      ˅  |-----------|  ˅

              <-->
              Roll
*/