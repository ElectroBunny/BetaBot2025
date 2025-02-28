// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class MoveAlgaeArmDown extends Command {
  private double speed, angle;
  private AlgaeArm arm;
  private boolean stayStable;

  public MoveAlgaeArmDown(double speed, double angle, boolean stayStable) {
    this.speed = speed;
    this.angle = angle;
    this.stayStable = stayStable;
    this.arm = AlgaeArm.getInstance();
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
    this.arm.setSpeed(AlgaeArm.getInstance().getPose() < angle ? 0.0: speed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
