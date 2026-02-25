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

  public enum ControllerType { PS4, LOGITECH, OTHER }

  private final GenericHID pilotController   = new GenericHID(Constants.CONTROL_PILOT_ID);
  private final GenericHID copilotController = new GenericHID(Constants.CONTROL_COPILOT_ID);

  public final ControllerType pilotType;
  public final ControllerType copilotType;

  public OI() {
    pilotType   = detectType(pilotController, ControllerType.PS4);
    copilotType = detectType(copilotController, ControllerType.LOGITECH);
    Functions.printInTerminal(Functions.roundToDecimalPlaces(314.159265358979323846, -2));
    // Publish detected types so you can verify in the dashboard
    SmartDashboard.putString("Pilot Controller",   pilotType.name());
    SmartDashboard.putString("Copilot Controller", copilotType.name());
  }

  private static ControllerType detectType(GenericHID hid, ControllerType useIfNotFound) {
    String name = hid.getName().toLowerCase();
    if (name.contains("wireless controller") || name.contains("dualshock") || name.contains("ps4")) {
      return ControllerType.PS4;
    }
    else if (name.contains("logitech")) {
      return ControllerType.LOGITECH;
    }
    else if(name.length() == 0) {
      return useIfNotFound;
    }
    else {
      Functions.printInTerminal(name, "Controller not supported:");
      return ControllerType.OTHER;
    }
  }

  // ── Pilot (driver) semantic inputs ──────────────────────────────────────
  // PS4 steering is squared for a softer feel near center; Logitech is linear.

  /** Steering input [-1, 1]. PS4 applies quadratic shaping; Logitech is linear. */
  public double getPilotSteering() {
    if(pilotType == ControllerType.PS4) {
      return getPilotRawAxis(Constants.ps4_leftStickX);
    }
    else if(pilotType == ControllerType.LOGITECH) {
      return getPilotRawAxis(Constants.logitech_leftStickX);
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
    if(pilotType == ControllerType.PS4) {
      double lt = getPilotRawAxis(Constants.ps4_leftTrigger);
      double rt = getPilotRawAxis(Constants.ps4_rightTrigger);
      return (rt-lt)/2;
    }
    else if(pilotType == ControllerType.LOGITECH) {
      double lt = pilotController.getRawButton(Constants.logitech_buttonLT) ? 1 : 0;
      double rt = pilotController.getRawButton(Constants.logitech_buttonRT) ? 1 : 0;
      return rt - lt;
    }
    else {
      Functions.printInTerminal("Pilot controller type not supported");
      return 0;
    }
  }

  /** Left stick Y axis, used for the left side in tank drive. */
  public double getPilotLeftStickY() {
    if(pilotType == ControllerType.PS4) {
      return getPilotRawAxis(Constants.ps4_leftStickY);
    }
    else if(pilotType == ControllerType.LOGITECH) {
      return getPilotRawAxis(Constants.logitech_leftStickY);
    }
    else {
      Functions.printInTerminal("Pilot controller type not supported");
      return 0;
    }
  }

  /** Right stick Y axis, used for the right side in tank drive. */
  public double getPilotRightStickY() {
    if(pilotType == ControllerType.PS4) {
      return getPilotRawAxis(Constants.ps4_rightStickY);
    }
    else if(pilotType == ControllerType.LOGITECH) {
      return getPilotRawAxis(Constants.logitech_rightStickY);
    }
    else {
      Functions.printInTerminal("Pilot controller type not supported");
      return 0;
    }
  }

  // ── Copilot (operator) semantic inputs ──────────────────────────────────

  /** Intake-in button. PS4: Circle. Logitech: B. */
  public boolean getCopilotIntakeIn() {
    if(copilotType == ControllerType.PS4) {
      return getCopilotRawButton(Constants.ps4_buttonCircle);
    }
    else if(copilotType == ControllerType.LOGITECH) {
      return getCopilotRawButton(Constants.logitech_buttonB);
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return false;
    }
  }

  /** Intake-out button. PS4: Square. Logitech: X. */
  public boolean getCopilotIntakeOut() {
    if(copilotType == ControllerType.PS4) {
      return getCopilotRawButton(Constants.ps4_buttonSquare);
    }
    else if(copilotType == ControllerType.LOGITECH) {
      return getCopilotRawButton(Constants.logitech_buttonX);
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return false;
    }
  }

  /** Intake arms toggle button. PS4: Triangle. Logitech: Y. */
  public boolean getCopilotIntakeArms() {
    if(copilotType == ControllerType.PS4) {
      return getCopilotRawButton(Constants.ps4_buttonTriangle);
    }
    else if(copilotType == ControllerType.LOGITECH) {
      return getCopilotRawButton(Constants.logitech_buttonY);
    }
    else {
      Functions.printInTerminal("Copilot controller type not supported");
      return false;
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
}
