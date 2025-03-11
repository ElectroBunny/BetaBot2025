// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPlace extends Command {
	private Elevator elevator;
	private double targetPose;

	private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
			Constants.ELEVATOR_MAX_VELO,
			Constants.ELEVATOR_MAX_ACCELLERATION);

	private final ProfiledPIDController pidController = new ProfiledPIDController(Constants.ELEVATOR_P,
			Constants.ELEVATOR_I, Constants.ELEVATOR_D, m_constraints);

	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0086531, 0.029608, 0.000215);

	public MoveElevatorToPlace(double targetPose, double tolerance) {
		this.targetPose = targetPose;
		elevator = Elevator.getInstance();
		addRequirements(elevator);

		pidController.setTolerance(tolerance);
	}

	@Override
	public void initialize() {
		elevator.setDefaultPose(targetPose);
	}

	@Override
	public void execute() {
		double power = pidController.calculate(elevator.getPose(), targetPose)
				+ elevatorFeedforward.calculate(pidController.getSetpoint().velocity);

		if (pidController.getPositionError() < 0 && power < -0.3) {
			power = -0.3;
		}

		elevator.setPower(power);
	}

	@Override
	public void end(boolean interrupted) {
		elevator.setPower(0);
	}

	@Override
	public boolean isFinished() {
		return pidController.atSetpoint();
	}
}
