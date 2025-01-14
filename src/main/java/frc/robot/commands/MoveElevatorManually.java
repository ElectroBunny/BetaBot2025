// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorManually extends Command 
{
  private static Elevator elevator;
  private double power;

  public MoveElevatorManually(double power) 
  {
    this.power = power;

    elevator = Elevator.getInstance();
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    elevator.setPower(this.power);
  }

  @Override
  public void end(boolean interrupted) 
  {
    elevator.stopMotor();
  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
