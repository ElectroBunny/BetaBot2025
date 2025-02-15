// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralScorer;

public class ScoreCoral extends Command {
	private CoralScorer coralScorer;
	private double power;

	public ScoreCoral(double power) {
		this.power = power;
		coralScorer = CoralScorer.getInstance();
		addRequirements(coralScorer);
	}

	@Override
	public void initialize() {
		coralScorer.setPower(power);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		coralScorer.setPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
