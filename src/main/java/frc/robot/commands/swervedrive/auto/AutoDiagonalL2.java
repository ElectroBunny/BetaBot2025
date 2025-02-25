// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.MoveElevatorToPlace;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoDiagonalL2 extends SequentialCommandGroup {
  public AutoDiagonalL2(SwerveSubsystem drivebase, boolean isRightScore) {
        addCommands(
      new InstantCommand(()-> drivebase.resetOdometry(new Pose2d(0,0,drivebase.getHeading()))),
      new WaitCommand(0),
      new RunCommand(() -> drivebase.drive(new Translation2d(1.3,0), 0, false), drivebase).withTimeout(2.3),
      new InstantCommand(()->drivebase.drive(new Translation2d(0,0), 0,false)),
      new AlignToReefTagRelative(isRightScore, drivebase).withTimeout(4),
      new MoveElevatorToPlace(Constants.L2_HEIGHT),
      new ScoreCoral(0.1).withTimeout(2));
  }
}
