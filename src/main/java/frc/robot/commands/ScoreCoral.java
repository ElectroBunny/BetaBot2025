// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;

public class ScoreCoral extends Command {
	private CoralScorer coralScorer;
	private double power;
	private boolean powerByElevator;

	public ScoreCoral(double power, boolean powerByElevator) {
		this.power = power;
		coralScorer = CoralScorer.getInstance();
		this.powerByElevator = powerByElevator;
		addRequirements(coralScorer);
	}

	@Override
	public void initialize() {
		if(powerByElevator)
		{
			if(Elevator.getDefaultPose() == Constants.L4_HEIGHT) {
				coralScorer.setPower(0.2);
			}
			else{
				coralScorer.setPower(0.15);
			}
		}
		else{
			coralScorer.setPower(power);
		}
	}

	@Override
	public void execute() {
		SmartDashboard.putBoolean("HasCoral", coralScorer.atIntakeCurrentLimit());
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
