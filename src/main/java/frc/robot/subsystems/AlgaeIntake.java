// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase 
{
  private SparkFlex intakeMotor;
  private SparkFlexConfig motorConfig;
  private RelativeEncoder motorEncoder;
  private static AlgaeIntake instance = null;

  public AlgaeIntake() 
  {
    intakeMotor = new SparkFlex(Constants.ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(Constants.ALGAE_INTAKE_CURRENT_LIMIT);
    intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    motorEncoder = intakeMotor.getEncoder();
  }

  public void setPower(double power) 
  {
    intakeMotor.set(power);
  }

  public boolean isBelowVelocity(double velocity)
  {
    return motorEncoder.getVelocity() < velocity;
  }

  public static AlgaeIntake getInstance() 
  {
    if (instance == null) {
      instance = new AlgaeIntake();
    }

    return instance;
  }

  @Override
  public void periodic() {
  }
}
