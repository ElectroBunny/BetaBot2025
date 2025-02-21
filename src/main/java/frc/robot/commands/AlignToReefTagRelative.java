// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class AlignToReefTagRelative extends Command {
  private PIDController xController;
  private PIDController yController;
  private PIDController rotController;
  private boolean isRightScore;
  private Timer stopTimer;
  SwerveSubsystem drivebase;

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    rotController = new PIDController(0, 0, 0);
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {

    this.stopTimer = new Timer();
    this.stopTimer.start();

    rotController.setSetpoint(0);
    rotController.setTolerance(0.5);

    xController.setSetpoint(isRightScore ? 0 : -0);
    xController.setTolerance(0.5);

    yController.setSetpoint(0);
    yController.setTolerance(0.5);
  }

  @Override
  public void execute() {

    if (LimelightHelpers.getTV("")) {
      this.stopTimer.reset();
      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");

      double xSpeed = xController.calculate(postions[0]);
      double ySpeed = yController.calculate(postions[1]);
      double rotValue = rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
      drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    return this.stopTimer.hasElapsed(0.2) &&
        rotController.atSetpoint() &&
        yController.atSetpoint() &&
        xController.atSetpoint();
  }
}