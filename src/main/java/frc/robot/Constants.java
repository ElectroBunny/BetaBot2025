// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
