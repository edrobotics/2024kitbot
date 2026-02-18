// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.TankDrive;
import frc.robot.commands.GTADrive;
import frc.robot.commands.Auto;
import frc.robot.commands.PickUpFuel;
import frc.robot.commands.IntakeArmsCommand;
import frc.robot.commands.ClimbCommand; 
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArms;
import frc.robot.subsystems.ClimbSubsystem;


public class Robot extends TimedRobot {
  public static DriveTrain driveTrain = new DriveTrain();
  public static Intake intake = new Intake();
  public static IntakeArms intakeArms = new IntakeArms();
  public static ClimbSubsystem climb = new ClimbSubsystem();
  public static OI m_oi;

  private Pose3d poseA = new Pose3d();
  private Pose3d poseB = new Pose3d();

  private StructPublisher<Pose3d> publisher;
  private StructArrayPublisher<Pose3d> arrayPublisher;


  private Command m_autonomousCommand = new Auto();

  //Called when the robot is started
  public void robotInit() {
    m_oi = new OI();

    driveTrain.setDefaultCommand(new GTADrive());
    intake.setDefaultCommand(new PickUpFuel());
    intakeArms.setDefaultCommand(new IntakeArmsCommand());
    climb.setDefaultCommand(new ClimbCommand());

    publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose3d.struct).publish();

    arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

  }

  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  //Runs every 20 ms
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    publisher.set(poseA);
    arrayPublisher.set(new Pose3d[] { poseA, poseB });
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // DriveTrain driveTrain = new DriveTrain(1); // CAN ID 1
  }

  //Called periodically during autonomous
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  //Called periodically during teleoperated
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  //Called once when the robot is first started up
  @Override
  public void simulationInit() {}

  //Called periodically whilst in simulation
  @Override
  public void simulationPeriodic() {}
}
