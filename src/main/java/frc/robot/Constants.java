// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // drivetrain motor ids
  public static final int LMOTOR1ID = 1;
  public static final int LMOTOR2ID = 2;
  public static final int RMOTOR1ID = 3;
  public static final int RMOTOR2ID = 4;
  public static final double speedReduction = 0.6;

  // robot dimensions - measure these on your actual robot!
  public static final double TRACK_WIDTH_METERS = 0.55; // distance between wheel centers
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0); // 6" wheels
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

  // encoder stuff for neos
  public static final double GEARING = 8.45; // kitbot gearing
  public static final double ENCODER_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE_METERS / GEARING;
  public static final double ENCODER_VELOCITY_CONVERSION = ENCODER_POSITION_CONVERSION / 60.0; // rpm to m/s

  // pathplanner pid - tune these later
  public static final double DRIVE_KP = 2.0;
  public static final double DRIVE_KI = 0.0;
  public static final double DRIVE_KD = 0.0;

  // max speeds for auto
  public static final double MAX_VELOCITY_MPS = 2.0;
  public static final double MAX_ACCELERATION_MPSS = 1.5;
  public static final double MAX_ANGULAR_VELOCITY_RAD = Math.PI;
  public static final double MAX_ANGULAR_ACCELERATION_RAD = Math.PI;

  // intake
  public static final boolean intakeConnected = true;
  public static final int intakeMotorId = 1;

  // controller mappings - different for each controller type
  public static final String controllerType = "logitech"; // "logitech" or "ps4"
  public static final int CONTROL_1_ID = 0;

  // ps4
  public static final int ps4_leftStickX = 0;
  public static final int ps4_leftStickY = 1;
  public static final int ps4_leftTrigger = 3;
  public static final int ps4_rightTrigger = 4;
  public static final int ps4_rightStickY = 5;
  public static final int ps4_circleButton = 3;

  // logitech
  public static final int logitech_leftStickX = 0;
  public static final int logitech_leftStickY = 1;
  public static final int logitech_rightStickX = 2;
  public static final int logitech_rightStickY = 3;
  public static final int logitech_buttonX = 0;
  public static final int logitech_buttonA = 1;
  public static final int logitech_buttonB = 3;
  public static final int logitech_buttonY = 4;
  public static final int logitech_leftTrigger = 7; // these are buttons not axes
  public static final int logitech_rightTrigger = 8;
}
