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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralScorer extends SubsystemBase {
	private SparkFlex scorerMotor;
	private SparkFlexConfig motorConfig;
	private static CoralScorer instance = null;
	private RelativeEncoder encoder;
	private SparkClosedLoopController closedLoopController;

	private CoralScorer() {
		scorerMotor = new SparkFlex(Constants.CORAL_SCORER_MOTOR_ID, MotorType.kBrushless);
		motorConfig = new SparkFlexConfig();
		closedLoopController = scorerMotor.getClosedLoopController();

		motorConfig.idleMode(IdleMode.kBrake);
		motorConfig.smartCurrentLimit(Constants.CORAL_SCORER_CURRENT_LIMIT);

		motorConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.CORAL_SCORER_P)
				.i(Constants.CORAL_SCORER_I)
				.d(Constants.CORAL_SCORER_D)
				.outputRange(-1, 1);


		scorerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		encoder = scorerMotor.getEncoder();
	}

	public void setPower(double power) {
		scorerMotor.setVoltage(power *12);
	}

	public void resetPosition() {
		this.encoder.setPosition(0);
	}

	public void moveCoralScorerToPose(double pose) {
		closedLoopController.setReference(pose, ControlType.kPosition,
				ClosedLoopSlot.kSlot0);
	}

	public double getCurrentPosition() {
		return encoder.getPosition();
	}

	public void moveCoralScorer(double rotations) {
		moveCoralScorerToPose(this.encoder.getPosition() + rotations);
	}

	public boolean isInPoint(double point) {
		return (Math.abs(encoder.getPosition() - point) <= Constants.CORAL_SCORER_POSITION_TOLERANCE);
	}

	public boolean atIntakeCurrentLimit() {
		return this.scorerMotor.getOutputCurrent() >= Constants.CORAL_SCORER_INTAKE_CURRENT_LIMIT;
	}

	public double getPose(){
		return encoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("current", scorerMotor.getOutputCurrent());
		SmartDashboard.putNumber("coralVel", encoder.getVelocity());
	}

	public static CoralScorer getInstance() {
		if (instance == null) {
			instance = new CoralScorer();
		}

		return instance;
	}
}
