// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReifFeildRelative extends Command {
  private PIDController xController;
  private PIDController yController;
  private PIDController rotController;
  private boolean isRightScore;

  public AlignToReifFeildRelative(boolean isRightScore) {
    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    rotController = new PIDController(0, 0, 0);
    this.isRightScore = isRightScore;

    addRequirements(RobotContainer.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotController.setSetpoint(0);
    rotController.setTolerance(0.5);

    xController.setSetpoint(isRightScore ? 0 : -0);
    xController.setTolerance(0.5);

    yController.setSetpoint(0);
    yController.setTolerance(0.5);

    Pose2d robotPose = RobotContainer.drivebase.getPose();

    Rotation2d angle = Constants.REEF_POSE2D.minus(robotPose).getRotation();
    if(angle > Constants.ZONE_ONE)
    {

    }
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PathPlannerTrajectory trajectory = PathPlannerTrajectory.generatePath()
    //Constants.CORAL_POSE2D - RobotContainer.drivebase.getPose();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
