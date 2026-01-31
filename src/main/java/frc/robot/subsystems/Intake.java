// Copyright (c) FIRST and other WPILib contributors../
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor = new SparkMax(Constants.intakeMotorId, MotorType.kBrushed);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Do NOT touch the following function unless it is needed. They make sure both the motors on the same side go in the same direction
  public void setIntakeMotor(double speed) {
    if(Constants.intakeConnected) {
      intakeMotor.set(speed);
    }
  }
}
