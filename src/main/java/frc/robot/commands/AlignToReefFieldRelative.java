// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix.Util;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utility;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReefFieldRelative extends Command {
  private PIDController xController;
  private PIDController yController;
  private PIDController rotController;
  private boolean isRightScore;

  public AlignToReefFieldRelative(boolean isRightScore) {
    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    rotController = new PIDController(0, 0, 0);
    this.isRightScore = isRightScore;

    addRequirements(RobotContainer.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d wantedPose = constructPath();

    rotController.setSetpoint(wantedPose.getRotation().getDegrees());
    rotController.setTolerance(0.5);

    xController.setSetpoint(wantedPose.getX());
    xController.setTolerance(0.5);

    yController.setSetpoint(wantedPose.getY());
    yController.setTolerance(0.5);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Pose2d robotPose = RobotContainer.drivebase.getPose();

      double xSpeed = xController.calculate(robotPose.getX());
      double ySpeed = yController.calculate(robotPose.getY());
      double rotValue = rotController.calculate(robotPose.getRotation().getDegrees());

      RobotContainer.drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  /**
   * @return find closest rif position from robots current location.
   */
  private Pose2d constructPath()
  {
    Pose2d robotPose = RobotContainer.drivebase.getPose();

    Rotation2d angle = Constants.REEF_POSE2D.minus(robotPose).getRotation();

    if (Utility.isAngleBetween(Utility.convertTo360Range(angle.getDegrees()) , Constants.ZONE_SIX.abs(Degrees), Constants.ZONE_ONE.abs(Degrees))) 
    {
      if(isRightScore)
      {
        return Constants.ZONE_ONE_RIGHT;
      }
      else
      {
        return Constants.ZONE_ONE_LEFT;
      }
    }
    else if(Utility.isAngleBetween(Utility.convertTo360Range(angle.getDegrees()) , Constants.ZONE_ONE.abs(Degrees), Constants.ZONE_TWO.abs(Degrees)))
    {
      if(isRightScore)
      {
        return Constants.ZONE_TWO_RIGHT;
      }
      else
      {
        return Constants.ZONE_TWO_LEFT;
      }
    }
    else if (Utility.isAngleBetween(Utility.convertTo360Range(angle.getDegrees()) , Constants.ZONE_TWO.abs(Degrees), Constants.ZONE_THREE.abs(Degrees))) 
    {
      if(isRightScore)
      {
        return Constants.ZONE_THREE_RIGHT;
      }
      else
      {
        return Constants.ZONE_THREE_RIGHT;
      }
    }
    else if (Utility.isAngleBetween(Utility.convertTo360Range(angle.getDegrees()) , Constants.ZONE_THREE.abs(Degrees), Constants.ZONE_FOUR.abs(Degrees))) {
      if(isRightScore)
      {
        return Constants.ZONE_FOUR_RIGHT;
      }
      else
      {
        return Constants.ZONE_FOUR_LEFT;
      }
    }
    else if (Utility.isAngleBetween(Utility.convertTo360Range(angle.getDegrees()) , Constants.ZONE_FOUR.abs(Degrees), Constants.ZONE_FIVE.abs(Degrees))) 
    {
      if(isRightScore)
      {
        return Constants.ZONE_FIVE_RIGHT;
      }
      else
      {
        return Constants.ZONE_FIVE_LEFT;
      }
    }
    else if (Utility.isAngleBetween(Utility.convertTo360Range(angle.getDegrees()) , Constants.ZONE_FIVE.abs(Degrees), Constants.ZONE_SIX.abs(Degrees))) 
    {
      if(isRightScore)
      {
        return Constants.ZONE_SIX_RIGHT;
      }
      else
      {
        return Constants.ZONE_SIX_LEFT;
      }
    }

    return new Pose2d();
  }
}
