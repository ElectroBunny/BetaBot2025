// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final int kDriverControllerPort = 0;
  }

  // Constants used by the Elevator
  public static class ElevatorConstants
  {
    public static final double ELEVATOR_POSITION_TOLERANCE = 0.5; 
    public static final int ELEVATOR_MOTOR_ID = 0; 
    public static final double ELEVATOR_MAX_VELO = 1000;
    public static final double ELEVATOR_MAX_ACCELLERATION = 1000;
    public static final double ELEVATOR_P = 0.4;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.0;
  }

  public static final int ARM_MOTOR_ID = 0;

  public static final double ARM_POSITION_CONVERTION_FACTOR = 360;

  public static final double ARM_P = 1;
  public static final double ARM_D = 1;
  public static final double ARM_I = 1;

  public static final double MAX_OUTPUT_RANGE = 1;
  public static final double MIN_OUTPUT_RANGE = -1;

  public static final double MAX_ARM_VELOCITY = 1000;
  public static final double MAX_ARM_ACCELERATION = 1000;

  public static final double SPARK_MAX_ENCODER_SPINs_NUM = 4096;

  public static final double ARM_TOLARANCE = 0;

  // Algae intake constants
  public static final int ALGAE_INTAKE_MOTOR_ID = 0;
  public static final int ALGAE_INTAKE_CURRENT_LIMIT = 40;
  public static final double ALGAE_INTAKE_VELOCITY_GAIN_TIME = 0.5;
  public static final double ALGAE_INTAKE_STOP_VELOCITY = 0;
  
  public static class CoralScorerConstants
  {
    public static final int CORAL_SCORER_MOTOR_ID = 0;
    public static final int CORAL_SCORER_CURRENT_LIMIT = 40;
  }
}
