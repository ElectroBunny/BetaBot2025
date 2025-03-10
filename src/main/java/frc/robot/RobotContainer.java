// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveElevatorToPlace;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.IntakeCoralByCurrent;
import frc.robot.commands.IntakeCoralPID;
import frc.robot.commands.MoveAlgaeArmDown;
import frc.robot.commands.MoveAlgaeArmManually;
import frc.robot.commands.MoveAlgaeArmToAngle;
import frc.robot.commands.MoveElevatorManually;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.swervedrive.auto.AutoDiagonalL2;
import frc.robot.commands.swervedrive.auto.AutoDiagonalL2EndWithDrive;
import frc.robot.commands.swervedrive.auto.AutoDiagonalL4;
import frc.robot.commands.swervedrive.auto.AutoForward;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.SwerveInputStream;

public class RobotContainer {
	final CommandPS5Controller driverController = new CommandPS5Controller(0);
	final CommandPS5Controller operatorController = new CommandPS5Controller(1);
	// final CommandPS5Controller test = new CommandPS5Controller(2);
	// final CommandJoystick logiJoystick = new CommandJoystick(2);

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

	SendableChooser<Command> m_chooser = new SendableChooser<>();


	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));

		elevator = Elevator.getInstance();
		coralScorer = CoralScorer.getInstance();

		
		elevator.setDefaultCommand(new ElevatorDefaultCommand());

		m_chooser.addOption("L2Right", new AutoDiagonalL2(drivebase, true));
		m_chooser.addOption("L2Left", new AutoDiagonalL2(drivebase, false));
		m_chooser.addOption("RightStartL2RightScoreDrive", new AutoDiagonalL2EndWithDrive(drivebase, true, true));
		m_chooser.addOption("RightStartL2LeftScoreDrive", new AutoDiagonalL2EndWithDrive(drivebase, false, true));
		m_chooser.addOption("LeftStartL2RightScoreDrive", new AutoDiagonalL2EndWithDrive(drivebase, true, false));
		m_chooser.addOption("LeftStartL2LeftScoreDrive", new AutoDiagonalL2EndWithDrive(drivebase, false, false));
		m_chooser.addOption("forward", new AutoForward(drivebase));

		SmartDashboard.putData(m_chooser);
	}

	private void configureBindings() {

		drivebase.setDefaultCommand(
				!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);
		
		// Algae
		// operatorController.povRight().onTrue(new MoveAlgaeArmToAngle(0.2, Constants.ALGAE_ARM_REEF_POSE, true));
		// operatorController.povLeft().onTrue(new MoveAlgaeArmToAngle(-0.1, Constants.ALGAE_ARM_CLOSED_POSE, false));
		operatorController.povRight().whileTrue(new MoveAlgaeArmManually(0.2));
		operatorController.povLeft().whileTrue(new MoveAlgaeArmManually(-0.1));

		// Coral reverse
		operatorController.L1().whileTrue(new ScoreCoral(-0.1));

		// Auto intake
		operatorController.R1().onTrue(new MoveElevatorToPlace(0, 1.5)
				.andThen(new IntakeCoralByCurrent(0.25))
				.andThen(new MoveElevatorToPlace(Constants.INTAKE_HEIGHT, Constants.ELEVATOR_POSITION_TOLERANCE)
						.andThen(new IntakeCoralPID(Constants.AUTO_CORAL_INTAKE_POWER))
						.andThen(new MoveElevatorToPlace(Constants.CLOSED_HEIGHT, Constants.ELEVATOR_POSITION_TOLERANCE))));

		// Stop auto intake
		operatorController.R2().onTrue(new InstantCommand(() -> coralScorer.setPower(0)));
		
		// Elevator manual
		operatorController.povUp().whileTrue(new MoveElevatorManually(0.3));
		operatorController.povDown().whileTrue(new MoveElevatorManually(-0.2));

		// Elevator auto poses
		operatorController.triangle().onTrue(new MoveElevatorToPlace(Constants.L3_HEIGHT, Constants.ELEVATOR_POSITION_TOLERANCE));
		operatorController.square().onTrue(new MoveElevatorToPlace(Constants.L2_HEIGHT, Constants.ELEVATOR_POSITION_TOLERANCE));
		operatorController.cross().onTrue(new MoveElevatorToPlace(Constants.INTAKE_HEIGHT, Constants.ELEVATOR_POSITION_TOLERANCE)
		.andThen(new MoveElevatorToPlace(Constants.CLOSED_HEIGHT, Constants.ELEVATOR_POSITION_TOLERANCE)));
		operatorController.circle().onTrue(new MoveElevatorToPlace(Constants.L4_HEIGHT, Constants.ELEVATOR_POSITION_TOLERANCE));

		// Reef alignment
		driverController.povRight().onTrue(new AlignToReefTagRelative(true, drivebase).withTimeout(7));
		driverController.povLeft().onTrue(new AlignToReefTagRelative(false, drivebase).withTimeout(7));

		// Reset swerve and elevator positions
		driverController.triangle().onTrue((Commands.runOnce(()->drivebase.zeroGyro(), drivebase)));
		driverController.create().onTrue(new InstantCommand(() -> elevator.resetPosition()));

		// Coral score
		driverController.L1().whileTrue(new ScoreCoral(Constants.CORAL_SCORE_POWER));
		driverController.L2().whileTrue(new ScoreCoral(0.2));

		// Slow drive
		driverController.R2().onTrue(new InstantCommand(() -> {
			swerveSpeedScaleTranslation = () -> 0.3;
			swerveSpeedScaleRotation = () -> 0.7;
		}))
				.onFalse(new InstantCommand(() -> {
					swerveSpeedScaleTranslation = () -> 1;
					swerveSpeedScaleRotation = () -> 1;
				}));

				
		// test.povUp().whileTrue(Elevator.getInstance().sysIdQuasistatic(Direction.kForward));
		// test.povDown().whileTrue(Elevator.getInstance().sysIdQuasistatic(Direction.kReverse));
		// test.povLeft().whileTrue(Elevator.getInstance().sysIdDynamic(Direction.kForward));
		// test.povRight().whileTrue(Elevator.getInstance().sysIdDynamic(Direction.kReverse));
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
		return m_chooser.getSelected();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	public void resetEncoderPositions() {
		elevator.resetPosition();
		coralScorer.resetPosition();
		drivebase.zeroGyro();
	}

	public void startCamera() {
    	CameraServer.startAutomaticCapture().setResolution(320, 180);
  	}
}
