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
  public void periodic() {}

  public void setIntakeMotor(double speed) {
    if(Constants.intakeConnected) {
      intakeMotor.set(speed);
    }
  }
}
