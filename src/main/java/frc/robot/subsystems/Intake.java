// Copyright (c) FIRST and other WPILib contributors../
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //The Victor SPX does not work in simulation, so if you want to simulate the code, use spark max instead

  private SparkMax intakeMotor = new SparkMax(Constants.intakeMotorId, MotorType.kBrushed);
  //private VictorSPX intakeMotor = new VictorSPX(Constants.intakeMotorId);

  public Intake() {}

  @Override
  public void periodic() {}

  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
    //intakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public Constants.ArmPriority armPriority = Constants.ArmPriority.INTAKE;
}
