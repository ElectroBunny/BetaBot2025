// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  private static IntakeArm instance=null;
  /** Creates a new Arm. */
  public IntakeArm() {
    this.motor = new SparkFlex(Constants.ARM_MOTOR_ID, MotorType.kBrushless); // the type is in need to be changed
    this.closedLoopController = this.motor.getClosedLoopController();
    this.encoder = this.motor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkFlexConfig();

    // configure encoder to specific conversion factor.
    this.motorConfig.encoder.positionConversionFactor(Constants.ARM_POSITION_CONVERTION_FACTOR);
    
    //set the motor to break mode so that the motor wont move.
    this.motorConfig.idleMode(IdleMode.kBrake);

    //configuring the encoders close loop 
    this.motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(Constants.ARM_P)
    .i(Constants.ARM_I)
    .d(Constants.ARM_D)
    .outputRange(Constants.MIN_OUTPUT_RANGE, Constants.MAX_OUTPUT_RANGE);

    //set max acceleration and velocity.
    this.motorConfig.closedLoop.maxMotion
    .maxVelocity(Constants.MAX_ARM_VELOCITY)
    .maxAcceleration(Constants.MAX_ARM_ACCELERATION)
    .allowedClosedLoopError(1);

    // configuring the motor to save the settings.
    this.motor.configure(this.motorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    // in question if the encoder position in need of setting to 0.
    this.encoder.setPosition(this.motor.getAbsoluteEncoder().getPosition());
  }

  /**
   * Returns an instance of the class.
   * @return an instance of the class
   */

  public static IntakeArm getInstance()
  {
    if(instance == null)
    {
      instance = new IntakeArm();
    }
    return instance;
  }

  /***
   * The function moves the motor to the wanted angle
   * 
   * @param angle - the angle to move the motor to
   */
  public void setAngle(double angle)
  {
    //sets the arm to a target angle
    this.closedLoopController.setReference(angle,  ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0);
  }

  /***
   * The function checks if the motor has reached the wanted angle
   * 
   * @param angle - The angle we want the motor to move to
   * @return if the motor has reached the wanted angle
   */
  public boolean isAtAngle(double angle)
  {
    return encoder.getPosition() <= angle + Constants.ARM_TOLARANCE &&
    encoder.getPosition() >= angle - Constants.ARM_TOLARANCE;
  }


  public double getCurrentAngle_In_Rads()
  {
    return this.encoder.getPosition();
  }

  public void setVoltage(double voltage)
  {
    this.motor.setVoltage(voltage);
  }

  public void stopMotor()
  {
    this.motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
