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

public class Elevator extends SubsystemBase {

  private static Elevator instance = null;

  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private Elevator() 
  {
    motor = new SparkFlex(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT);

    motorConfig.encoder.positionConversionFactor(1);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.ELEVATOR_P)
        .i(Constants.ELEVATOR_I)
        .d(Constants.ELEVATOR_D)
        .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
        .maxVelocity(Constants.ELEVATOR_MAX_VELO)
        .maxAcceleration(Constants.ELEVATOR_MAX_ACCELLERATION)
        .allowedClosedLoopError(1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void resetPosition() 
  {
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {}

  public static Elevator getInstance() 
  {
    if (instance == null) 
    {
      instance = new Elevator();
    }
    return instance;
  }

  /**
   * Moves the elevator to the specified location
   * @param point the specified location            
   */
  public void moveElevator(double point)
  {
    closedLoopController.setReference(point, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
  }

  public void stopMotor() 
  {
    motor.stopMotor();
  }

  public void setPower(double power) 
  {
    motor.set(power);
  }

  /**
   *              Checks whether the elevator is approximately at the specified location.
   * @param point the specified location
   * @return True if the elevator is within the tolerance range of the specified location, 
   *         else false. 
   */
  public boolean isInPoint(double point) 
  {
    return (Math.abs(encoder.getPosition() - point) <= Constants.ELEVATOR_POSITION_TOLERANCE);
  }
}
