// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralScorer;

public class IntakeCoralByCurrent extends Command {
  private CoralScorer coralScorer;
  private double power;
  Timer timer;

  public IntakeCoralByCurrent(double power) {
    this.power = power;
    this.coralScorer = CoralScorer.getInstance();
    timer = new Timer();
		addRequirements(coralScorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralScorer.setPower(this.power);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralScorer.setPower(0);
    SmartDashboard.putBoolean("HasCoral", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralScorer.atIntakeCurrentLimit()&&timer.hasElapsed(0.1);
  }
}
