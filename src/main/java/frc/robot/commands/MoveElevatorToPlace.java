// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPlace extends Command {
	private Elevator elevator;
	private double targetPose;

	public MoveElevatorToPlace(double targetPose) {
		this.targetPose = targetPose;
		elevator = Elevator.getInstance();
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		elevator.moveElevatorToPose(targetPose);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return elevator.isInPoint(targetPose);
	}
}
