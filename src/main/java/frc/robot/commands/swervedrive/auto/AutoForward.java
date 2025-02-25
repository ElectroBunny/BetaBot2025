// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoForward extends SequentialCommandGroup {
  public AutoForward(SwerveSubsystem drivebase, double distance, double speed) {
    addCommands(
      new InstantCommand(()-> drivebase.resetOdometry(new Pose2d(0,0,drivebase.getHeading()))),
      new WaitCommand(0),
      new InstantCommand(() -> drivebase.driveToDistanceCommand(distance, speed)));
  }
}
