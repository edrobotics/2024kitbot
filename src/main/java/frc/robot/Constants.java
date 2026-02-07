// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //Drivetrain
  public static final int LMOTOR1ID = 6;
  public static final int LMOTOR2ID = 2;
  public static final int RMOTOR1ID = 3;
  public static final int RMOTOR2ID = 4;
  public static final double wheelBaseWidth = 0.555; // in meters
  public static final double leftWheelVelocity = 0.7; // in meters per second, this is in no way the correct velocity, just a rough estimate
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

  //Intake
  public static final boolean intakeConnected = true;
  public static final int intakeMotorId = 1;

  //Controller
  //As the numbering of the axes and buttons differs on different controllers, separate constants are made for the different types
  public static final String controllerType = "ps4"; //"logitech" or "ps4"
  public static final int CONTROL_1_ID = 0;
  //For a ps4 controller:
  public static final int ps4_leftStickX = 0;
  public static final int ps4_leftStickY = 1;
  public static final int ps4_rightStickX = 2;
  public static final int ps4_leftTrigger = 3;
  public static final int ps4_rightTrigger = 4;
  public static final int ps4_rightStickY = 5;
  public static final int ps4_buttonSquare = 1;
  public static final int ps4_buttonX = 2;
  public static final int ps4_buttonCircle = 3;
  public static final int ps4_buttonTriangle = 4;
  public static final int ps4_buttonL1 = 5;
  public static final int ps4_buttonR1 = 6;
  public static final int ps4_buttonL2 = 7;
  public static final int ps4_buttonR2 = 8;
  public static final int ps4_buttonShare = 9;
  public static final int ps4_buttonOptions = 10;
  public static final int ps4_buttonLeftStick = 11;
  public static final int ps4_buttonRightStick = 12;
  public static final int ps4_buttonPS = 13;
  public static final int ps4_buttonTouchpad = 14;
  //For a logitech controller:
  public static final int ps4_dpadUp = 4;
  public static final int ps4_dpadDown = 5;
  public static final int ps4_dpadLeft = 7;
  public static final int ps4_dpadRight = 6;

  public static final int logitech_leftStickX = 0;
  public static final int logitech_leftStickY = 1;
  public static final int logitech_rightStickX = 2;
  public static final int logitech_rightStickY = 3;
  public static final int logitech_buttonX = 1;
  public static final int logitech_buttonA = 2;
  public static final int logitech_buttonB = 3;
  public static final int logitech_buttonY = 4;
  public static final int logitech_buttonLB = 5;
  public static final int logitech_buttonRB = 6;
  public static final int logitech_buttonLT = 7; //Note that this only counts as a button
  public static final int logitech_buttonRT = 8; //Note that this only counts as a button
  public static final int logitech_buttonBack = 9;
  public static final int logitech_buttonStart = 10;
  public static final int logitech_buttonLeftStick = 11;
  public static final int logitech_buttonRightStick = 12;

  // climb
  // set this to the CAN id (or other id) of your climb motor controller
  public static final int CLIMB_MOTOR_ID = 5;
  // button id used in RobotContainer for the climb (adjust to your joystick
  // mapping)
  public static final int CLIMB_BUTTON = 1;

  //Autonomous
  public static final long autonomousTime = 20000; // in milliseconds
}
