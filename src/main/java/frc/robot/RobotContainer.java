// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveElevatorToPlace;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.IntakeCoralByCurrent;
import frc.robot.commands.IntakeCoralPID;
import frc.robot.commands.MoveElevatorManually;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.SwerveInputStream;

public class RobotContainer {
	final CommandPS5Controller driverController = new CommandPS5Controller(0);
	final CommandPS5Controller operatorController = new CommandPS5Controller(1);
	final CommandJoystick logiJoystick = new CommandJoystick(1);

	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
			"swerve"));

	DoubleSupplier swerveSpeedScaleTranslation = () -> 1;
	DoubleSupplier swerveSpeedScaleRotation = () -> 1;

	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> driverController.getLeftY() * -1 * swerveSpeedScaleTranslation.getAsDouble(),
			() -> driverController.getLeftX() * -1 * swerveSpeedScaleTranslation.getAsDouble())
			.withControllerRotationAxis(
					() -> driverController.getRightX() * -1 * swerveSpeedScaleRotation.getAsDouble())
			.deadband(OperatorConstants.DEADBAND)
			.cubeRotationControllerAxis(true)
			.cubeRotationControllerAxis(true)
			.allianceRelativeControl(true);

	Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

	SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX())
			.withControllerRotationAxis(() -> driverController.getRawAxis(2))
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
			.withControllerHeadingAxis(() -> Math.sin(
					driverController.getRawAxis(
							2) * Math.PI)
					* (Math.PI * 2),
					() -> Math.cos(
							driverController.getRawAxis(
									2) * Math.PI)
							*
							(Math.PI * 2))
			.headingWhile(true);

	Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

	Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

	private Elevator elevator;
	private CoralScorer coralScorer;

	public RobotContainer() {
		// Creating a named command for the auto part
		// NamedCommands.registerCommand("MoveArmToPosAuto", new
		// MoveElevatorToPlace(Constants.AUTO_POSITION));

		// Configure the trigger bindings
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));

		elevator = Elevator.getInstance();
		coralScorer = CoralScorer.getInstance();

		new Trigger(() -> RobotController.getUserButton())
				.onTrue(new InstantCommand(() -> resetEncoderPositions()));
	}

	private void configureBindings() {
		// (Condition) ? Return-On-True : Return-on-False
		drivebase.setDefaultCommand(
				!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);

		driverController.L2().whileTrue(new ScoreCoral(0.05));
		// driverController.square().onTrue(new )

		operatorController.povUp().whileTrue(new MoveElevatorManually(0.3));
		operatorController.povDown().whileTrue(new MoveElevatorManually(-0.2));

		operatorController.L2().whileTrue(new ScoreCoral(-Constants.CORAL_SCORE_POWER));
		operatorController.R2().onTrue(new MoveElevatorToPlace(Constants.CLOSED_HEIGHT)
				.andThen(new IntakeCoralByCurrent(0.25))
				.andThen(new MoveElevatorToPlace(Constants.INTAKE_HEIGHT)
						.andThen(new IntakeCoralPID(Constants.CORAL_SCORE_POWER))
						.andThen(new MoveElevatorToPlace(Constants.CLOSED_HEIGHT))));
		operatorController.square().onTrue(new InstantCommand(() -> coralScorer.setPower(0)));

		operatorController.circle().onTrue(new MoveElevatorToPlace(Constants.L3_HEIGHT));
		operatorController.triangle().onTrue(new MoveElevatorToPlace(Constants.L2_HEIGHT));
		operatorController.cross().onTrue(new MoveElevatorToPlace(Constants.CLOSED_HEIGHT));

		driverController.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));

		driverController.R2().onTrue(new InstantCommand(() -> {
			swerveSpeedScaleTranslation = () -> 0.3;
			swerveSpeedScaleRotation = () -> 0.5;
		}))
				.onFalse(new InstantCommand(() -> {
					swerveSpeedScaleTranslation = () -> 1;
					swerveSpeedScaleRotation = () -> 1;
				}));

	}

	public void logInitialize() {
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return drivebase.getAutonomousCommand("New Auto");
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	public void resetEncoderPositions() {
		elevator.resetPosition();
		coralScorer.resetPosition();
	}
}
