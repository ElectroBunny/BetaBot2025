// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class ScoreAlgae extends Command {
	private AlgaeIntake algaeIntake;
	private double power;

	public ScoreAlgae(double power) {
		this.power = power;
		algaeIntake = AlgaeIntake.getInstance();
		addRequirements(algaeIntake);
	}

	@Override
	public void initialize() {
		algaeIntake.setPower(power);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		algaeIntake.setPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
