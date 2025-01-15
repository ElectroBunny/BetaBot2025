// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CollectAlgae;
import frc.robot.commands.MoveElevatorManually;
import frc.robot.commands.MoveElevatorToPlace;
import frc.robot.commands.ScoreCoral;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer 
{
	private final CommandPS5Controller driverController = new CommandPS5Controller(
			OperatorConstants.kDriverControllerPort);

	public RobotContainer() 
  {
		configureBindings();
	}

	private void configureBindings() 
  {
    // Coral command
    driverController.cross().whileTrue(new ScoreCoral(Constants.CORAL_SCORE_POWER));

    // Algae commands
    driverController.L1().onTrue(new CollectAlgae(Constants.ALGAE_INTAKE_POWER));
    driverController.R1().onTrue(new CollectAlgae(-Constants.ALGAE_INTAKE_POWER));

    // Elevator commands
    driverController.square().onTrue(new MoveElevatorManually(Constants.ELEVATOR_MANUAL_POWER));
    driverController.circle().onTrue(new MoveElevatorManually(-Constants.ELEVATOR_MANUAL_POWER));

    driverController.povRight().onTrue(new MoveElevatorToPlace(Constants.L1_HEIGHT));
    driverController.povLeft().onTrue(new MoveElevatorToPlace(Constants.L2_HEIGHT));
    driverController.povDown().onTrue(new MoveElevatorToPlace(Constants.L3_HEIGHT));
    driverController.povUp().onTrue(new MoveElevatorToPlace(Constants.L4_HEIGHT));
    driverController.triangle().onTrue(new MoveElevatorToPlace(Constants.L4_HEIGHT));

	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Commands.none();
	}
}
