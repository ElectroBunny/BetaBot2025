// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkFlex climberMotor;
  private static Climber instance = null;

  private Climber() {
    climberMotor = new SparkFlex(Constants.CLIMBER_SPARK_ID, MotorType.kBrushless);
  }

  public void setPower(double power) {
    climberMotor.set(power);
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }

    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
