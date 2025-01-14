// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorManually extends Command 
{
  private static Elevator elevator;
  private double point;
  private DoubleSupplier doubleSupplier;

  public MoveElevatorManually(DoubleSupplier doubleSupplier) 
  {
    this.doubleSupplier = doubleSupplier;

    elevator = Elevator.getInstance();
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    elevator.setPower(doubleSupplier.getAsDouble());
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
