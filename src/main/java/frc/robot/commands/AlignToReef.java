// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.RawFiducial;
public class AlignToReef extends Command 
{
  private PIDController xController;
  private PIDController yController;
  private PIDController rotController;
  private boolean isRightScore;


  public AlignToReef(boolean isRightScore) {
    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    rotController = new PIDController(0, 0, 0);
    this.isRightScore = isRightScore;

    addRequirements(RobotContainer.drivebase);
  }

  @Override
  public void initialize() {

        rotController.setSetpoint(0);
        rotController.setTolerance(0.5);


        xController.setSetpoint(isRightScore ? 0 : -0);
        xController.setTolerance(0.5);

        yController.setSetpoint(0);
        yController.setTolerance(0.5);

  }

  @Override
  public void execute() {
    
    if(LimelightHelpers.getTV("")){

      RawFiducial[] botPose = LimelightHelpers.getRawFiducials("");
      
      // double xSpeed = xController.calculate(closestFiducial.);
      // double ySpeed = yController.calculate(botPose.getY());
      // double rotValue = rotController.calculate(LimelightHelpers.getDetectorClassIndext)


      RobotContainer.drivebase.drive(new Translation2d(xSpeed, ySpeed), 0, isRightScore);
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
