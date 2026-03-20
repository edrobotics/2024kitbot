package frc.robot;

public class AutoPath {
  public double[] waypointsX;
  public double[] waypointsY;
  public double[] waypointsDegrees;
  public double intakeX;
  public double intakeY;
  
  public AutoPath(double[] xCoords, double[] yCoords, double[] degrees, double intakeDownAtX, double intakeDownAtY) {
    waypointsX = xCoords;
    waypointsY = yCoords;
    waypointsDegrees = degrees;
    intakeX = intakeDownAtX;
    intakeY = intakeDownAtY;
  }
}
