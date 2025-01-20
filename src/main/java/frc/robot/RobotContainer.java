// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CollectAlgae;
import frc.robot.commands.ManualAlgaeCollection;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {

  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.circle().whileTrue(new ManualAlgaeCollection(Constants.ALGAE_INTAKE_TEST_VELOCITY));
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
}
