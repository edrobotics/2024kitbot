package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class DeadReck extends Command {
  public DeadReck () {
    //addRequirements(Robot.driveTrain, Robot.gyroscope);
  }

  private double x;
  private double y;
  private double lastLeftPosition;
  private double lastRightPosition;

  public void initialize () {
    x = 0;
    y = 0;
    lastLeftPosition = Robot.driveTrain.getLeftPosition();
    lastRightPosition = Robot.driveTrain.getRightPosition();
  }

  public void execute () {
    double currentLeftPosition = Robot.driveTrain.getLeftPosition();
    double currentRightPosition = Robot.driveTrain.getRightPosition();
    double radians = Math.toRadians(Robot.gyroscope.getYaw());

    double deltaLeftPosition = currentLeftPosition - lastLeftPosition;
    double deltaRightPosition = currentRightPosition - lastRightPosition;

    double deltaForwardPosition = (deltaLeftPosition + deltaRightPosition) / 2;

    x += deltaForwardPosition * Math.sin(radians);
    y += deltaForwardPosition * Math.cos(radians);

    lastLeftPosition = currentLeftPosition;
    lastRightPosition = currentRightPosition;

    //Functions.printInTerminal("x: " + x + ", y: " + y);
  }

  public double getRobotX() { return x; }
  public double getRobotY() { return y; }
  public double getRobotHeading() { return Robot.gyroscope.getYaw(); }
}

