// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private static AlgaeArm instance = null;
  private DutyCycleEncoder absEncoder;
  private RelativeEncoder motorEncoder;

  public AlgaeArm() {
    this.motor = new SparkFlex(Constants.ALGAE_ARM_MOTOR_ID, MotorType.kBrushless);
    this.closedLoopController = this.motor.getClosedLoopController();
    this.absEncoder = new DutyCycleEncoder(Constants.ALGAE_ENCODER_DIO);

    this.motorEncoder = this.motor.getEncoder();
    // this.encoder = new DutyCycleEncoder(Constants.ALGAE_ENCODER_DIO, 1, 0.1);
    
    motorConfig = new SparkFlexConfig();
    this.motorConfig.encoder.positionConversionFactor(Constants.ALGAE_ARM_CONVERSION_FACTOR);
    this.motorConfig.idleMode(IdleMode.kBrake);

    this.motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(Constants.ALGAE_ARM_P)
    .i(Constants.ALGAE_ARM_I)
    .d(Constants.ALGAE_ARM_D)
    .outputRange(-1, 1);

    this.motor.configure(this.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.motorEncoder.setPosition(absEncoder.get());
  }

  /**
   * The function moves the arm to a desired angle.
   * @param angle - the desired angle.
   */
  public void setAngle(double angle)
  {
    this.closedLoopController.setReference(angle, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  /***
   * The function checks if the arm has reached the desired angle.
   * @param angle - the desired angle.
   * @return whether the motor has reached the angle.
   */
  public boolean isAtAngle(double angle) {
    return Math.abs(angle - absEncoder.get()) <= Constants.ALGAE_ARM_TOLERANCE;
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void stopArm() {
    motor.stopMotor();
  }

  /**
   * Returns an instance of the class.
   * @return an instance of the class
   */
   public static AlgaeArm getInstance()
   {
     if(instance == null)
     {
       instance = new AlgaeArm();
     }
     return instance;
   }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Abs Encoder", absEncoder.get());
  }
}
