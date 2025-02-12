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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  private static AlgaeArm instance = null;

  public AlgaeArm() {
    this.motor = new SparkFlex(Constants.ALGAE_ARM_MOTOR_ID, MotorType.kBrushless);
    this.closedLoopController = this.motor.getClosedLoopController();
    this.encoder = this.motor.getEncoder();

    motorConfig = new SparkFlexConfig();
    this.motorConfig.encoder.positionConversionFactor(Constants.ALGAE_ARM_CONVERSION_FACTOR);
    this.motorConfig.idleMode(IdleMode.kBrake);

    this.motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(Constants.ALGAE_ARM_P)
    .i(Constants.ALGAE_ARM_I)
    .d(Constants.ALGAE_ARM_D)
    .outputRange(-1, 1);

    this.motorConfig.closedLoop.maxMotion
    .maxVelocity(Constants.ALGAE_ARM_MAX_VEL)
    .maxAcceleration(Constants.ALGAE_ARM_MAX_ACCEL)
    .allowedClosedLoopError(1);

    this.motor.configure(this.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.encoder.setPosition(this.motor.getAbsoluteEncoder().getPosition());
  }

  /**
   * The function moves the arm to a desired angle.
   * @param angle - the desired angle.
   */
  public void setAngle(double angle)
  {
    this.closedLoopController.setReference(angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  /***
   * The function checks if the arm has reached the desired angle.
   * @param angle - the desired angle.
   * @return whether the motor has reached the angle.
   */
  public boolean isAtAngle(double angle) {
    return Math.abs(angle - encoder.getPosition()) <= Constants.ALGAE_ARM_TOLERANCE;
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
    // This method will be called once per scheduler run
  }
}
