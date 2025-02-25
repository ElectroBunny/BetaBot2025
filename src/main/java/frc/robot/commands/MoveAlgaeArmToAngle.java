// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class MoveAlgaeArmToAngle extends Command {
  private double speed, angle;
  private AlgaeArm arm;
  private boolean stayStable;

  public MoveAlgaeArmToAngle(double speed, double angle, boolean stayStable) {
    this.speed = speed;
    this.angle = angle;
    this.stayStable = stayStable;
    this.arm = AlgaeArm.getInstance();
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    this.arm.setSpeed(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(stayStable ? 0.01 : 0);
  }

  @Override
  public boolean isFinished() {
    return this.arm.isAtAngle(this.angle);
  }
}
