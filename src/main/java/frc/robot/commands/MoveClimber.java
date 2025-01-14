// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimber extends Command {
  private Climber climber;
  private DoubleSupplier joystick;

  /**
   * MoveClimber moves the climber up and down
   * @param joystick the value from the joystick
   */
  public MoveClimber(DoubleSupplier joystick) {
    this.joystick = joystick;

    climber = Climber.getInstance();
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.setPower(joystick.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    climber.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
