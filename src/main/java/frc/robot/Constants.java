// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //Drivetrain
  public static final int LMOTOR1ID = 1;
  public static final int LMOTOR2ID = 2;
  public static final int RMOTOR1ID = 3;
  public static final int RMOTOR2ID = 4;
  public static final double speedReduction = 0.6;
  
  //Intake
  public static final boolean intakeConnected = true;
  public static final int intakeMotorId = 1;

  //Controller
  //As the numbering of the axes and buttons differs on different controllers, separate constants are made for the different types
  public static final String controllerType = "logitech"; //"logitech" or "ps4"
  public static final int CONTROL_1_ID = 0;
  //For a ps4 controller:
  public static final int ps4_leftStickX = 0;
  public static final int ps4_leftStickY = 1;
  public static final int ps4_leftTrigger = 3;
  public static final int ps4_rightTrigger = 4;
  public static final int ps4_rightStickY = 5;
  public static final int ps4_circleButton = 3;
  //For a logitech controller:
  public static final int logitech_leftStickX = 0;
  public static final int logitech_leftStickY = 1;
  public static final int logitech_rightStickX = 2;
  public static final int logitech_rightStickY = 3;
  public static final int logitech_buttonX = 0;
  public static final int logitech_buttonA = 1;
  public static final int logitech_buttonB = 3;
  public static final int logitech_buttonY = 4;
  public static final int logitech_leftTrigger = 7;  //Note that this only counts as a button
  public static final int logitech_rightTrigger = 8; //Note that this only counts as a button
}
