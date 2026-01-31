// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

// autonomous using pathplanner
// create paths in the pathplanner app and put them in deploy/pathplanner/paths/
public class Auto extends Command {
  private Command pathCommand = null;

  // change this to match your path file name
  private static final String PATH_NAME = "ExamplePath";

  public Auto() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(PATH_NAME);
      pathCommand = AutoBuilder.followPath(path);
      pathCommand.initialize();
    } catch (Exception e) {
      DriverStation.reportError("couldn't load path '" + PATH_NAME + "': " + e.getMessage(), e.getStackTrace());
      // fallback - just drive forward
      pathCommand = createFallbackAuto();
      pathCommand.initialize();
    }
  }

  @Override
  public void execute() {
    if (pathCommand != null) {
      pathCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted);
    }
    Robot.driveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    return pathCommand != null && pathCommand.isFinished();
  }

  // backup auto if path file is missing - just drives forward for 2 sec
  private Command createFallbackAuto() {
    return Commands.run(
        () -> Robot.driveTrain.arcadeDrive(0.3, 0),
        Robot.driveTrain).withTimeout(2.0).andThen(
            Commands.runOnce(() -> Robot.driveTrain.stop()));
  }

  // helper for auto chooser in robotcontainer
  public static Command getAutoCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("couldn't load path: " + pathName, e.getStackTrace());
      return Commands.none();
    }
  }
}
