// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Functions;

public class OI {

  // ── Controller type detection ────────────────────────────────────────────
  // Names are matched against the string the driver station reports for the HID.
  // Add more entries here if you connect a different controller model.

  private final GenericHID pilotController   = new GenericHID(Constants.CONTROL_PILOT_ID);
  private final GenericHID copilotController = new GenericHID(Constants.CONTROL_COPILOT_ID);

  public final Constants.ControllerType pilotType;
  public final Constants.ControllerType copilotType;

  public OI() {
    //pilotType   = detectType(pilotController, Constants.ControllerType.XBOX);
    //copilotType = detectType(copilotController, Constants.ControllerType.XBOX);

    pilotType = Constants.ControllerType.PS4;
    copilotType = Constants.ControllerType.XBOX;
    
    // Publish detected types so you can verify in the dashboard
    SmartDashboard.putString("Pilot Controller",   pilotType.name());
    SmartDashboard.putString("Copilot Controller", copilotType.name());
  }

  private static Constants.ControllerType detectType(GenericHID hid, Constants.ControllerType useIfNotFound) {
    String name = hid.getName().toLowerCase();
    if (name.contains("wireless controller") || name.contains("dualshock") || name.contains("ps4")) {
      return Constants.ControllerType.PS4;
    }
    else if (name.contains("logitech")) {
      return Constants.ControllerType.LOGITECH;
    }
    else if(name.contains("xbox")) {
      return Constants.ControllerType.XBOX;
    }
    else {
      return useIfNotFound;
    }
  }

  // ── Pilot (driver) semantic inputs ──────────────────────────────────────
  // PS4 steering is squared for a softer feel near center; Logitech is linear.

  /** Steering input [-1, 1]. PS4 applies quadratic shaping; Logitech is linear. */
  public double getPilotSteering() {
    if(pilotType == Constants.ControllerType.PS4) {
      return getPilotRawAxis(Constants.ps4_leftStickX);
    }
    else if(pilotType == Constants.ControllerType.LOGITECH) {
      return getPilotRawAxis(Constants.logitech_leftStickX);
    }
    else if(pilotType == Constants.ControllerType.XBOX) {
      return getPilotRawAxis(Constants.xbox_leftStickX);
    }
    else {
      Functions.printInTerminal("Pilot controller type not supported");
      return 0;
    }
  }

  /**
   * Forward/backward throttle [-1, 1].
   * PS4: analog triggers (right = forward, left = reverse), halved to stay in range.
   * Logitech: digital trigger buttons, full [-1, 1] range.
   */
  public double getPilotThrottle() {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    if(pilotType == Constants.ControllerType.PS4) {
      double lt = getPilotRawAxis(Constants.ps4_leftTrigger);
      double rt = getPilotRawAxis(Constants.ps4_rightTrigger);
      return (rt-lt)/2;
    }
    else if(pilotType == Constants.ControllerType.LOGITECH) {
      double lt = pilotController.getRawButton(Constants.logitech_buttonLT) ? 1 : 0;
      double rt = pilotController.getRawButton(Constants.logitech_buttonRT) ? 1 : 0;
      return rt - lt;
    }
    else if(pilotType == Constants.ControllerType.XBOX) {
      double lt = getPilotRawAxis(Constants.xbox_leftTrigger);
      double rt = getPilotRawAxis(Constants.xbox_rightTrigger);
      return rt - lt;
    }
    else {
      Functions.printInTerminal("Pilot controller type not supported");
      return 0;
    }
  }

  /** Left stick Y axis, used for the left side in tank drive. */
  public double getPilotLeftStickY() {
    if(pilotType == Constants.ControllerType.PS4) {
      return getPilotRawAxis(Constants.ps4_leftStickY);
    }
    else if(pilotType == Constants.ControllerType.LOGITECH) {
      return getPilotRawAxis(Constants.logitech_leftStickY);
    }
    else if(pilotType == Constants.ControllerType.XBOX) {
      return getPilotRawAxis(Constants.xbox_leftStickY);
    }
    else {
      Functions.printInTerminal("Pilot controller type not supported");
      return 0;
    }
  }

  /** Right stick Y axis, used for the right side in tank drive. */
  public double getPilotRightStickY() {
    if(pilotType == Constants.ControllerType.PS4) {
      return getPilotRawAxis(Constants.ps4_rightStickY);
    }
    else if(pilotType == Constants.ControllerType.LOGITECH) {
      return getPilotRawAxis(Constants.logitech_rightStickY);
    }
    else if(pilotType == Constants.ControllerType.XBOX) {
      return getPilotRawAxis(Constants.xbox_rightStickY);
    }
    else {
      Functions.printInTerminal("Pilot controller type not supported");
      return 0;
    }
  }

  // ── Copilot (operator) semantic inputs ──────────────────────────────────

  /** Intake-in button. PS4: Circle. Logitech: B. */
  public boolean getCopilotIntakeIn() {
    if(copilotType == Constants.ControllerType.PS4) {
      return getCopilotRawButton(Constants.ps4_buttonR2);
    }
    else if(copilotType == Constants.ControllerType.LOGITECH) {
      return getCopilotRawButton(Constants.logitech_buttonB);
    }
    else if(copilotType == Constants.ControllerType.XBOX) {
      return getCopilotRawAxis(Constants.xbox_rightTrigger) > 0;
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return false;
    }
  }

  /** Intake-out button. PS4: Square. Logitech: X. */
  public boolean getCopilotIntakeOut() {
    if(copilotType == Constants.ControllerType.PS4) {
      return getCopilotRawButton(Constants.ps4_buttonL2);
    }
    else if(copilotType == Constants.ControllerType.LOGITECH) {
      return getCopilotRawButton(Constants.logitech_buttonX);
    }
    else if(copilotType == Constants.ControllerType.XBOX) {
      return getCopilotRawAxis(Constants.xbox_leftTrigger) > 0;
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return false;
    }
  }

  /** Intake arms toggle button. PS4: Triangle. Logitech: Y. */
  private boolean intakeArmWasPressed = false;
  public boolean getCopilotIntakeArms() {
    if(copilotType == Constants.ControllerType.PS4) {
      boolean returnValue = !intakeArmWasPressed && getCopilotRawButton(Constants.ps4_buttonCircle);
      intakeArmWasPressed = getCopilotRawButton(Constants.ps4_buttonCircle);
      return returnValue;
    }
    else if(copilotType == Constants.ControllerType.LOGITECH) {
      boolean returnValue = !intakeArmWasPressed && getCopilotRawButton(Constants.logitech_buttonB);
      intakeArmWasPressed = getCopilotRawButton(Constants.logitech_buttonB);
      return returnValue;
    }
    else if(copilotType == Constants.ControllerType.XBOX) {
      boolean returnValue = !intakeArmWasPressed && getCopilotRawButton(Constants.xbox_buttonB);
      intakeArmWasPressed = getCopilotRawButton(Constants.xbox_buttonB);
      return returnValue;
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return false;
    }
  }

  public double getCopilotManualIntakeArms() {
    if(copilotType == Constants.ControllerType.PS4) {
      return getCopilotRawAxis(Constants.ps4_leftStickY);
    }
    else if(copilotType == Constants.ControllerType.LOGITECH) {
      return getCopilotRawAxis(Constants.logitech_leftStickY);
    }
    else if(copilotType == Constants.ControllerType.XBOX) {
      return getCopilotRawAxis(Constants.xbox_leftStickY);
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return 0;
    }
  }

  private boolean climberWasPressed = false;
  public boolean getCopilotClimber() {
    if(copilotType == Constants.ControllerType.PS4) {
      boolean returnValue = !climberWasPressed && getCopilotRawButton(Constants.ps4_buttonTriangle);
      climberWasPressed = getCopilotRawButton(Constants.ps4_buttonTriangle);
      return returnValue;
    }
    else if(copilotType == Constants.ControllerType.LOGITECH) {
      boolean returnValue = !climberWasPressed && getCopilotRawButton(Constants.logitech_buttonY);
      climberWasPressed = getCopilotRawButton(Constants.logitech_buttonY);
      return returnValue;
    }
    else if(copilotType == Constants.ControllerType.XBOX) {
      boolean returnValue = !climberWasPressed && getCopilotRawButton(Constants.xbox_buttonY);
      climberWasPressed = getCopilotRawButton(Constants.xbox_buttonY);
      return returnValue;
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return false;
    }
  }

  public double getCopilotManualClimber() {
    if(copilotType == Constants.ControllerType.PS4) {
      return getCopilotRawAxis(Constants.ps4_rightStickY);
    }
    else if(copilotType == Constants.ControllerType.LOGITECH) {
      return getCopilotRawAxis(Constants.logitech_rightStickY);
    }
    else if(copilotType == Constants.ControllerType.XBOX) {
      return getCopilotRawAxis(Constants.xbox_rightStickY);
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return 0;
    }
  }

  // ── Low-level pass-throughs ──────────────────────────────────────────────
  // Kept so any code that needs a specific raw axis/button can still reach it.
  public double  getPilotRawAxis(int axis)       { return pilotController.getRawAxis(axis); }
  public boolean getPilotRawButton(int button)   { return pilotController.getRawButton(button); }
  public int     getPilotPOV()                   { return pilotController.getPOV(); }
  public double  getCopilotRawAxis(int axis)     { return copilotController.getRawAxis(axis); }
  public boolean getCopilotRawButton(int button) { return copilotController.getRawButton(button); }
  public int     getCopilotPOV()                 { return copilotController.getPOV(); }

  public int getPilotButtonCount() { return pilotController.getButtonCount(); }
  public int getCopilotButtonCount() { return copilotController.getButtonCount(); }
}
