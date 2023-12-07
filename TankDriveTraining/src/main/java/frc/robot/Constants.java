// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //From now on, uppercase for constants
  //TODO: find actual CAN IDs
  public static final int CAN_LEFT_MOTOR_1 = 1;
  public static final int CAN_LEFT_MOTOR_2 = 2;
  public static final int CAN_RIGHT_MOTOR_1 = 3;
  public static final int CAN_RIGHT_MOTOR_2 = 4;
  //TODO: Track width of the tank drive
  public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  //TODO: Max speeds of tank drive
  public static final double MAX_LINEAR_SPEED_METER_PER_SECOND = 8;
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SECOND = 3;
  public static final double MAX_WHEEL_SPEED_METER_PER_SECOND = 12;
  
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(5);
  public static final double GEAR_RATIO = 1;

  //PID constants for tank drive
  public static final double TANK_P = 15;
  public static final double TANK_I = 0;
  public static final double TANK_D = 0.5;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
