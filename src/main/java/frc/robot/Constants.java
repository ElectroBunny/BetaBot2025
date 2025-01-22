// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import swervelib.math.Matter;

public final class Constants {
	public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	// Maximum speed of the robot in meters per second, used to limit acceleration.

	public static final class DrivebaseConstants {
		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants {
		// Joystick Deadband
		public static final double DEADBAND = 0.1;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
		public static final int kDriverControllerPort = 0;
	}

	// Elevator constants
	public static final double ELEVATOR_POSITION_TOLERANCE = 0.5;
	public static final int ELEVATOR_MOTOR_ID = 0;
	public static final double ELEVATOR_MAX_VELO = 1000;
	public static final double ELEVATOR_MAX_ACCELLERATION = 1000;
	public static final double ELEVATOR_P = 0.4;
	public static final double ELEVATOR_I = 0.0;
	public static final double ELEVATOR_D = 0.0;
	public static final int ELEVATOR_CURRENT_LIMIT = 40;

	// Algae intake constants
	public static final int ALGAE_INTAKE_MOTOR_ID = 0;
	public static final int ALGAE_INTAKE_CURRENT_LIMIT = 40;
	public static final double ALGAE_INTAKE_VELOCITY_GAIN_TIME = 0.5;
	public static final double ALGAE_INTAKE_STOP_VELOCITY = 0;

	// Coral scores constants
	public static final int CORAL_SCORER_MOTOR_ID = 0;
	public static final int CORAL_SCORER_CURRENT_LIMIT = 40;
  
  //alignment constant
  public static final Pose2d REEF_POSE2D = new Pose2d();

  public static final Angle ZONE_ONE = Angle.ofBaseUnits(0, Degrees);
  public static final Angle ZONE_TWO = Angle.ofBaseUnits(60, Degrees);
  public static final Angle ZONE_THREE = Angle.ofBaseUnits(120, Degrees);
  public static final Angle ZONE_FOUR = Angle.ofBaseUnits(180, Degrees);
  public static final Angle ZONE_FIVE = Angle.ofBaseUnits(240, Degrees);
  public static final Angle ZONE_SIX = Angle.ofBaseUnits(300, Degrees);
}
