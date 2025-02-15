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

	private SparkFlex masterMotor, followerMotor;
	private SparkFlexConfig masterMotorConfig, followerMotorConfig;
	private SparkClosedLoopController closedLoopController;
	private RelativeEncoder encoder;

	private Elevator() {
		masterMotor = new SparkFlex(Constants.ELEVATOR_MASTER_MOTOR_ID, MotorType.kBrushless);
		followerMotor = new SparkFlex(Constants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
		closedLoopController = masterMotor.getClosedLoopController();
		encoder = masterMotor.getEncoder();

		masterMotorConfig = new SparkFlexConfig();
		followerMotorConfig = new SparkFlexConfig();
		masterMotorConfig.idleMode(IdleMode.kBrake);
		followerMotorConfig.idleMode(IdleMode.kBrake);
		masterMotorConfig.smartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT);
		followerMotorConfig.smartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT);

		masterMotorConfig.encoder.positionConversionFactor(Constants.ELEVATOR_CONVERSION_FACTOR);

		masterMotorConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.ELEVATOR_P)
				.i(Constants.ELEVATOR_I)
				.d(Constants.ELEVATOR_D)
				.outputRange(-1, 1);

		masterMotorConfig.closedLoop.maxMotion
				.maxVelocity(Constants.ELEVATOR_MAX_VELO)
				.maxAcceleration(Constants.ELEVATOR_MAX_ACCELLERATION)
				.allowedClosedLoopError(1);

		masterMotor.configure(masterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		followerMotorConfig.follow(masterMotor);
		followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	public void resetPosition() {
		encoder.setPosition(0);
	}

	/**
	 * Moves the elevator to the specified location
	 * 
	 * @param point the specified location
	 */
	public void moveElevator(double point) {
		closedLoopController.setReference(point, ControlType.kMAXMotionPositionControl,
				ClosedLoopSlot.kSlot0);
	}

	public void stop() {
		masterMotor.stopMotor();
	}

	public void setPower(double power) {
		masterMotor.set(power);
	}

	/**
	 * Checks whether the elevator is approximately at the specified location.
	 * 
	 * @param point the specified location
	 * @return True if the elevator is within the tolerance range of the specified
	 *         location,
	 *         else false.
	 */
	public boolean isInPoint(double point) {
		return (Math.abs(encoder.getPosition() - point) <= Constants.ELEVATOR_POSITION_TOLERANCE);
	}

	public static Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}
		return instance;
	}

	@Override
	public void periodic() {
	}
}
