// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import java.util.logging.LogManager;
import java.util.logging.Logger;
import java.util.random.RandomGenerator.ArbitrarilyJumpableGenerator;

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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
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

		masterMotorConfig = new SparkFlexConfig();
		followerMotorConfig = new SparkFlexConfig();

		// sets the elevator to coast prematch and it will be set to break when match
		// starts
		masterMotorConfig.idleMode(IdleMode.kBrake);
		followerMotorConfig.idleMode(IdleMode.kBrake);

		masterMotorConfig.smartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT);
		followerMotorConfig.smartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT);

		masterMotorConfig.encoder
				.positionConversionFactor(
						2 * Math.PI * Constants.ELEVATOR_ROLLER_RAIDUS / Constants.ELEVATOR_CONVERSION_FACTOR)
				.velocityConversionFactor(
						2 * Math.PI * Constants.ELEVATOR_ROLLER_RAIDUS / Constants.ELEVATOR_CONVERSION_FACTOR);

		masterMotorConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.ELEVATOR_P)
				.i(Constants.ELEVATOR_I)
				.d(Constants.ELEVATOR_D)
				.outputRange(-1, 1);

		masterMotorConfig.closedLoop.maxMotion
				.maxVelocity(Constants.ELEVATOR_MAX_VELO)
				.maxAcceleration(Constants.ELEVATOR_MAX_ACCELLERATION)
				.allowedClosedLoopError(Constants.ELEVATOR_POSITION_TOLERANCE);

		masterMotorConfig.inverted(Constants.Elevator_INVERTED);
		followerMotorConfig.inverted(!Constants.Elevator_INVERTED);

		masterMotor.configure(masterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		followerMotorConfig.follow(masterMotor, true);
		followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		
		encoder = masterMotor.getEncoder();

		routine =
		new SysIdRoutine(
			// Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
			new SysIdRoutine.Config(null, Voltage.ofBaseUnits(4, Volts),null),
			new SysIdRoutine.Mechanism(
				masterMotor::setVoltage,
				log -> {
				  // Record a frame for the shooter motor.
				  log.motor("elevator")
					  .voltage(
						  m_appliedVoltage.mut_replace(
							  masterMotor.getAppliedOutput(), Volts))
					  .linearPosition(m_angle.mut_replace(encoder.getPosition(), Meters))
					  .linearVelocity(m_velocity.mut_replace(encoder.getVelocity(), MetersPerSecond));
				},
				this));
	}

	/**
	 * used to reset the elevator do not use in game
	 */
	public void setIdleMode(IdleMode idleMode) {
		masterMotorConfig.idleMode(idleMode);
		followerMotorConfig.idleMode(idleMode);

		masterMotor.configureAsync(masterMotorConfig, ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
		followerMotor.configureAsync(followerMotorConfig, ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
	}

	public void resetPosition() {
		encoder.setPosition(0);
	}

	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0087967 * 7168, 0.025837 * 7168, 0.00021483 * 7168);

	/**
	 * Moves the elevator to the specified location
	 * 
	 * @param point the specified location
	 */
	public void moveElevatorToPose(double point) {
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
		SmartDashboard.putNumber("elevatorPose", encoder.getPosition());
		SmartDashboard.putNumber("elevatorSpeed", encoder.getVelocity());

	}


	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}
	
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_angle = Centimeters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  
  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine routine;

}
