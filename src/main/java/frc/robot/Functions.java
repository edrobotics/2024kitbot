package frc.robot;

public class Functions {
  //Prints text in the terminal
  //  On the first line the title is written and on the second the value you want to display
  //  You can skip adding a title, and then only the given value is displayed
  public static void printInTerminal(Object value, String title) {
    System.out.println(title);
    System.out.println(value);
  }
  public static void printInTerminal(Object value) {
    System.out.println(value);
  }

  //If only one parameter is given, the value is clamped to be between -1 and 1
  //If a second and third parameter is added, the value is clamped to be between those parameters
  public static double clamp(double speed) {
    return Math.max(-1, Math.min(1, speed));
  }
  public static double clamp(double speed, double min, double max) {
    return Math.max(min, Math.min(max, speed));
  }

  //Rounds a number to a given number of decimal places
  public static double roundToDecimalPlaces(double number, int decimalPlaces) {
    return Math.round(number*Math.pow(10, decimalPlaces))/Math.pow(10.0, decimalPlaces);
  }

  public static double pythagoranTheorem(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2));
  }
  public static double pythagoranTheorem(double xDistance, double yDistance) {
    return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
  }
}
