// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {

  // ── Controller type detection ────────────────────────────────────────────
  // Names are matched against the string the driver station reports for the HID.
  // Add more entries here if you connect a different controller model.

  public enum ControllerType { PS4, LOGITECH, UNKNOWN }

  private final GenericHID pilotController   = new GenericHID(Constants.CONTROL_PILOT_ID);
  private final GenericHID copilotController = new GenericHID(Constants.CONTROL_COPILOT_ID);

  public final ControllerType pilotType;
  public final ControllerType copilotType;

  public OI() {
    pilotType   = detectType(pilotController);
    copilotType = detectType(copilotController);

    // Publish detected types so you can verify in the dashboard
    SmartDashboard.putString("Pilot Controller",   pilotType.name());
    SmartDashboard.putString("Copilot Controller", copilotType.name());
  }

  private static ControllerType detectType(GenericHID hid) {
    String name = hid.getName().toLowerCase();
    if (name.contains("wireless controller") || name.contains("dualshock") || name.contains("ps4")) {
      return ControllerType.PS4;
    } else if (name.contains("logitech")) {
      return ControllerType.LOGITECH;
    }
    return ControllerType.UNKNOWN;
  }

  // ── Pilot (driver) semantic inputs ──────────────────────────────────────
  // PS4 steering is squared for a softer feel near center; Logitech is linear.

  /** Steering input [-1, 1]. PS4 applies quadratic shaping; Logitech is linear. */
  public double getPilotSteering() {
    return switch (pilotType) {
      case PS4 -> {
        double s = pilotController.getRawAxis(Constants.ps4_leftStickX);
        yield s * Math.abs(s);
      }
      case LOGITECH -> pilotController.getRawAxis(Constants.logitech_leftStickX);
      default -> 0;
    };
  }

  /**
   * Forward/backward throttle [-1, 1].
   * PS4: analog triggers (right = forward, left = reverse), halved to stay in range.
   * Logitech: digital trigger buttons, full [-1, 1] range.
   */
  public double getPilotThrottle() {
    return switch (pilotType) {
      case PS4 -> {
        double lt = pilotController.getRawAxis(Constants.ps4_leftTrigger);
        double rt = pilotController.getRawAxis(Constants.ps4_rightTrigger);
        yield (rt - lt) / 2;
      }
      case LOGITECH -> {
        double lt = pilotController.getRawButton(Constants.logitech_buttonLT) ? 1 : 0;
        double rt = pilotController.getRawButton(Constants.logitech_buttonRT) ? 1 : 0;
        yield rt - lt;
      }
      default -> 0;
    };
  }

  /** Left stick Y axis, used for the left side in tank drive. */
  public double getPilotLeftStickY() {
    return switch (pilotType) {
      case PS4      -> pilotController.getRawAxis(Constants.ps4_leftStickY);
      case LOGITECH -> pilotController.getRawAxis(Constants.logitech_leftStickY);
      default -> 0;
    };
  }

  /** Right stick Y axis, used for the right side in tank drive. */
  public double getPilotRightStickY() {
    return switch (pilotType) {
      case PS4      -> pilotController.getRawAxis(Constants.ps4_rightStickY);
      case LOGITECH -> pilotController.getRawAxis(Constants.logitech_rightStickY);
      default -> 0;
    };
  }

  // ── Copilot (operator) semantic inputs ──────────────────────────────────

  /** Intake-in button. PS4: Circle. Logitech: B. */
  public boolean getCopilotIntakeIn() {
    return switch (copilotType) {
      case PS4      -> copilotController.getRawButton(Constants.ps4_buttonCircle);
      case LOGITECH -> copilotController.getRawButton(Constants.logitech_buttonB);
      default -> false;
    };
  }

  /** Intake-out button. PS4: Square. Logitech: X. */
  public boolean getCopilotIntakeOut() {
    return switch (copilotType) {
      case PS4      -> copilotController.getRawButton(Constants.ps4_buttonSquare);
      case LOGITECH -> copilotController.getRawButton(Constants.logitech_buttonX);
      default -> false;
    };
  }

  /** Intake arms toggle button. PS4: Triangle. Logitech: Y. */
  public boolean getCopilotIntakeArms() {
    return switch (copilotType) {
      case PS4      -> copilotController.getRawButton(Constants.ps4_buttonTriangle);
      case LOGITECH -> copilotController.getRawButton(Constants.logitech_buttonY);
      default -> false;
    };
  }

  /** D-pad / POV value. Returns -1 if not pressed, else degrees clockwise (0 = up). */
  public int getCopilotPOV() {
    return copilotController.getPOV();
  }

  // ── Low-level pass-throughs ──────────────────────────────────────────────
  // Kept so any code that needs a specific raw axis/button can still reach it.

  public double  GetPilotRawAxis(int axis)       { return pilotController.getRawAxis(axis); }
  public boolean GetPilotRawButton(int button)   { return pilotController.getRawButton(button); }
  public int     GetPilotPOV()                   { return pilotController.getPOV(); }
  public double  GetCopilotRawAxis(int axis)     { return copilotController.getRawAxis(axis); }
  public boolean GetCopilotRawButton(int button) { return copilotController.getRawButton(button); }
}
