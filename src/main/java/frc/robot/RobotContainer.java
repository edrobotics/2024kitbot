// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
// import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ClimbCommand;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Simplified: Test all buttons 1-14 every cycle
    SmartDashboard.putBoolean("Testing", true);
    
    // Try button 4 for up
    /*new Trigger(() -> driver.getHID().getRawButton(4))
        .onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> System.out.println("BUTTON 4 PRESSED")))
        .whileTrue(new ClimbCommand(Robot.climb, 0.6));
    
    // Try button 5 for down
    new Trigger(() -> driver.getHID().getRawButton(5))
        .onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> System.out.println("BUTTON 5 PRESSED")))
        .whileTrue(new ClimbCommand(Robot.climb, -0.6));*/
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}