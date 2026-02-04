// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax climbMotor = new SparkMax(Constants.CLIMB_MOTOR_ID, MotorType.kBrushed);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    // any init if needed
  }

  /** Run the climb motor at given speed (-1..1). */
  public void run(double speed) {
    speed = Math.max(-1.0, Math.min(1.0, speed));
    climbMotor.set(speed);
  }

  /** Stop the climb motor. */
  public void stop() {
    climbMotor.set(0);
  }

  @Override
  public void periodic() {
    // nothing here; commands control the motor
  }
}
