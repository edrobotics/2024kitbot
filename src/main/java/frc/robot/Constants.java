// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //Drivetrain
  public static final int LMOTOR1ID = 1;
  public static final int LMOTOR2ID = 2;
  public static final int RMOTOR1ID = 3;
  public static final int RMOTOR2ID = 4;
  public static final double DRIVETRAIN_SPEED_REDUCTION = 1;
  // robot dimensions - measure these on your actual robot!
  public static final double TRACK_WIDTH_METERS = 0.555; // distance between wheel centers (measured)
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0); // 6" wheels
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
  // pathplanner pid - tune these later
  public static final double DRIVE_KP = 2.0;
  public static final double DRIVE_KI = 0.0;
  public static final double DRIVE_KD = 0.0;
  // encoder stuff for neos
  public static final double DRIVETRAIN_GEARING = 9.13; // kitbot gearing
  public static final double DRIVETRAIN_ENCODER_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE_METERS / DRIVETRAIN_GEARING;
  public static final double DRIVETRAIN_ENCODER_VELOCITY_CONVERSION = DRIVETRAIN_ENCODER_POSITION_CONVERSION / 60.0; // rpm to m/s
  // for smoother driving
  public static final double DRIVETRAIN_MIN_INPUT = 0.1;  // the minimum input that is given for the drivetrain
  public static final double DRIVETRAIN_MAX_INPUT_AT = 1; // at this speed the input is not reduced anymore

  //Autonomous (pathplanner, not working)
  public static final double LOOP_PERIOD_SECONDS = 0.02; // WPILib default scheduler period (20 ms)
  public static final double MAX_VELOCITY_MPS = 2.0;
  public static final double MAX_ACCELERATION_MPSS = 1.5;
  public static final double MAX_ANGULAR_VELOCITY_RAD = Math.PI;
  public static final double MAX_ANGULAR_ACCELERATION_RAD = Math.PI;
  public static final long AUTO_DRIVE_TIME_MS = 3000; // how long to drive forward in auto
  public static final double AUTO_DRIVE_SPEED = 0.1; // motor speed during auto drive

  //Working auto
  public static final double AUTO_TURN_SPEED = 0.5; // maximum turning speed during auto
  public static final long AUTO_TIME = 20000; // in milliseconds
  public static final long AUTO_ANGULAR_DEADBAND = 5;

  //Intake
  public static final int intakeMotorId = 5;
  public static final int leftIntakeMotorArmID = 6;
  public static final int rightIntakeMotorArmID = 7;
  public static final double INTAKE_ARMS_TARGET_ROTATIONS = 0.2; // rotation target per toggle
  public static final double INTAKE_ARMS_DOWN_SPEED = 0.2; // motor speed for intake arms going upwards
  public static final double INTAKE_ARMS_UP_SPEED = 1;
  public static final double INTAKE_ARMS_SPEED = 0.75;
  public static final double INTAKE_ARMS_GEARING = 1;
  public static final double INTAKE_ARMS_SPEED_REDUCTION = 0.2; // pecentage of minimum speed for the motor when approaching the target position
  public static final double INTAKE_ARMS_DEADBAND = 0.05;

  // Vision / AprilTags
  // TODO: Set CAMERA_NAME to match the name configured in PhotonVision.
  public static final String CAMERA_NAME = "front_camera";
  // TODO: Measure the camera's position relative to the robot center (meters + radians).
  //       Translation3d(forward, left, up), Rotation3d(roll, pitch, yaw).
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
      new Translation3d(0.3, 0.0, 0.5),   // 30 cm forward, 50 cm up from center
      new Rotation3d(0, Math.toRadians(-15), 0) // tilted 15° down
  );

  //Climber
  public static final int CLIMBER_WINCH_MOTOR_ID = 8; // the id of the motor which will power the climb, wont use encoder position because of high gearing + winch design
  public static final int CLIMBER_ENCODER_MOTOR_ID = 9; // the id of the motor whose job it is to keep the climb arm stowed away when nessessary, come up with a better name if you can

  public static final double CLIMBER_WINCH_UP_SPEED = 0.1;     //Speed of the winch going up
  public static final double CLIMBER_WINCH_DOWN_SPEED = -0.03; //Speed of the winch going down
  public static final double CLIMBER_WINCH_HOLD_IN_PLACE = 0.07; //Input to hold the climber in place
  public static final double CLIMBER_ENCODER_SPEED = 0.5; //Speed of the climber encoder motor;

  public static final double CLIMBER_MOTOR_GEARING = 125; // the gearing of the climb motor
  public static final double CLIMBER_DEADBAND = 0.05; // the deadband for the climb encoder motor, to prevent it from jittering when it reaches the target position

  //Controller
  public static final int CONTROL_PILOT_ID = 0;
  public static final int CONTROL_COPILOT_ID = 1;

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

  public static final int ps4_dpadUp = 4;    //???
  public static final int ps4_dpadDown = 5;  //???
  public static final int ps4_dpadLeft = 7;  //???
  public static final int ps4_dpadRight = 6; //???

  //For a logitech controller:
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

  //For an Xbox controller
  // The logitech controllers can be turned into an xbox with a switch on the lower side. D=logitech, X=xbox
  public static final int xbox_leftStickX = 0;
  public static final int xbox_leftStickY = 1;
  public static final int xbox_leftTrigger = 2;
  public static final int xbox_rightTrigger = 3;
  public static final int xbox_rightStickX = 4;
  public static final int xbox_rightStickY = 5;

  public static final int xbox_buttonA = 1;
  public static final int xbox_buttonB = 2;
  public static final int xbox_buttonX = 3;
  public static final int xbox_buttonY = 4;
  public static final int xbox_buttonLB = 5;
  public static final int xbox_buttonRB = 6;
  public static final int xbox_buttonBack = 7;
  public static final int xbox_buttonStart = 8;
  public static final int xbox_buttonLeftStick = 9;
  public static final int xbox_buttonRightStick = 10;

  //Enums
  public static enum ControllerType { PS4, LOGITECH, XBOX }
  public static enum ArmPriority { INTAKE, CLIMBER }
}
