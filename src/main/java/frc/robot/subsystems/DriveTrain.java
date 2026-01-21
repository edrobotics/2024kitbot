// Copyright (c) FIRST and other WPILib contributors../
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  //private SparkMax leftMotor1 = new SparkMax(Constants.LMOTOR1ID, MotorType.kBrushed);
  private SparkMax leftMotor2 = new SparkMax(Constants.LMOTOR2ID, MotorType.kBrushed); 
  
  //private SparkMax rightMotor1 = new SparkMax(Constants.RMOTOR1ID, MotorType.kBrushed);
  //private SparkMax rightMotor2 = new SparkMax(Constants.RMOTOR2ID, MotorType.kBrushed);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLeftMotors(double speed) {
    //leftMotor1.set(speed);
    leftMotor2.set(speed);
  }
  public void setRightMotors(double speed) {
    //rightMotor1.set(-speed);
    //rightMotor2.set(-speed);
  }
}
