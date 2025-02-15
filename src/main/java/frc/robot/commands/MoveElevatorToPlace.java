// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPlace extends Command {
	private static Elevator elevator;
	private double point;

	public MoveElevatorToPlace(double point) {
		this.point = point;
		elevator = Elevator.getInstance();
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		elevator.moveElevator(point);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		elevator.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return elevator.isInPoint(point);
	}
}
