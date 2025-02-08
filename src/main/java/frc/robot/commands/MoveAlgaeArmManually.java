// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class MoveAlgaeArmManually extends Command {
  private double speed;
  private AlgaeArm arm;

  public MoveAlgaeArmManually(double speed) {
    this.speed = speed;
    this.arm = AlgaeArm.getInstance();
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    this.arm.setSpeed(this.speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    this.arm.stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
