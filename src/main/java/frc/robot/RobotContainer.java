// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveElevatorToPlace;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.CollectAlgae;
import frc.robot.commands.MoveAlgaeArmToAngle;
import frc.robot.commands.MoveElevatorManually;
import frc.robot.commands.MoveElevatorToPlace;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.Elevator;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.time.Instant;

import swervelib.SwerveInputStream;

public class RobotContainer {

	final CommandPS5Controller driverController = new CommandPS5Controller(0);
	final CommandJoystick logiJoystick = new CommandJoystick(1);

	private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
			"swerve"));

	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> driverController.getLeftY() * -1,
			() -> driverController.getLeftX() * -1)
			.withControllerRotationAxis(driverController::getRightX)
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
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

	public RobotContainer() {
		// Creating a named command for the auto part
		// NamedCommands.registerCommand("MoveArmToPosAuto", new MoveElevatorToPlace(Constants.AUTO_POSITION));

		// Configure the trigger bindings
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));

		elevator = Elevator.getInstance();

		new Trigger(() -> RobotController.getUserButton())
				.onTrue(new InstantCommand(() -> resetEncoderPositions()));
	}

	private void configureBindings() {
		// (Condition) ? Return-On-True : Return-on-False
		drivebase.setDefaultCommand(
				!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);

		// // Algae commands
		// driverController.L1().onTrue(new
		// MoveAlgaeArmToAngle(Constants.ALGAE_ARM_REEF_ANGLE).
		// andThen(new CollectAlgae(Constants.ALGAE_INTAKE_POWER)));

		// driverController.R1().onTrue(new
		// MoveAlgaeArmToAngle(Constants.ALGAE_ARM_REEF_ANGLE).
		// andThen(new ScoreAlgae(-Constants.ALGAE_INTAKE_POWER)));

		// Elevator commands
		// driverController.square().whileTrue(new
		// MoveElevatorManually(Constants.ELEVATOR_MANUAL_POWER));
		// driverController.circle().whileTrue(new
		// MoveElevatorManually(-Constants.ELEVATOR_MANUAL_POWER));
		// driverController.y().whileTrue(new MoveElevatorManually(1));
		// driverController.a().whileTrue(new MoveElevatorManually(-0.2));
		
		// driverController.y().onFalse(new MoveElevatorManually(0));
		// driverController.a().onFalse(new MoveElevatorManually(0));

		
		driverController.povUp().whileTrue(new MoveElevatorToPlace(25));
		driverController.povDown().whileTrue(new MoveElevatorToPlace(10));

		// driverController.povUp().whileTrue(Elevator.getInstance().sysIdQuasistatic(Direction.kForward));
		// driverController.povDown().whileTrue(Elevator.getInstance().sysIdQuasistatic(Direction.kReverse));
		// driverController.povLeft().whileTrue(Elevator.getInstance().sysIdDynamic(Direction.kForward));
		// driverController.povRight().whileTrue(Elevator.getInstance().sysIdDynamic(Direction.kReverse));

		// driverController.circle().whileTrue(new ScoreCoral(0.5));

		// driverController.povRight().onTrue(new
		// MoveElevatorToPlace(Constants.L1_HEIGHT));
		// driverController.povLeft().onTrue(new
		// MoveElevatorToPlace(Constants.L2_HEIGHT));
		// driverController.povDown().onTrue(new
		// MoveElevatorToPlace(Constants.L3_HEIGHT));
		// driverController.povUp().onTrue(new
		// MoveElevatorToPlace(Constants.L4_HEIGHT));
		// driverController.triangle().onTrue(new
		// MoveElevatorToPlace(Constants.CLOSED_HEIGHT));
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
	}
}
