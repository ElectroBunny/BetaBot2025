// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralScorer extends SubsystemBase 
{
  private SparkFlex scorerMotor;
  private SparkFlexConfig motorConfig;
  private static CoralScorer instance = null;

  public CoralScorer() 
  {
    scorerMotor = new SparkFlex(Constants.CoralScorerConstants.CORAL_SCORER_MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(Constants.CoralScorerConstants.CORAL_SCORER_CURRENT_LIMIT);
    scorerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setPower(double power) 
  {
    scorerMotor.set(power);
  }

  public static CoralScorer getInstance() 
  {
    if (instance == null) {
      instance = new CoralScorer();
    }

    return instance;
  }

  @Override
  public void periodic() 
  {
  }
}
