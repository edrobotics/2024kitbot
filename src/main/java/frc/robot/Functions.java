package frc.robot;

public class Functions {
  // Add any custom functions you want to use across your code here. You can call these functions from any other class like this: Robot.functions.functionName()

  // Clamps a value to the desired range
  public static double clamp(double speed, double min, double max) {
    return Math.max(min, Math.min(max, speed));
  }

  // Rounds a double to the specified number of decimal places
  public static double roundToDecimalPlaces(double value, int decimalPlaces) {
    double scale = Math.pow(10, decimalPlaces);
    return Math.round(value * scale) / scale;
  }
}
