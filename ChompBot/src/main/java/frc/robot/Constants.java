// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   // pid drive p was .0020645
   // Max velocity in json waas 14.5
   public static final double ROBOT_MASS = (40.0) * 0.453592; // 32lbs * kg per pound
   public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
   public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag


   public static final class Auton {

      public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
      public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
      public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

      public static final double MAX_SPEED = 4;
      public static final double MAX_ACCELERATION = 2;
   }

   public static final class Drivebase {

      // Hold time on motor brakes when disabled
      public static final double WHEEL_LOCK_TIME = 10; // seconds
      public static final double MAX_ROBOT_SPEED = 4; 
      // Drive conversion factor = 60 * pi * diameter / gear ratio
      // wheel diameter = 0.09398 meters (3.7 in)
      // Gear ratio = 8.14:1
      public static final double DRIVE_GEAR_RATIO = 8.14;
      public static final double DRIVE_WHEEL_DIAMETER = 0.09398;
      public static final double DRIVE_CONVERSION_FACTOR = ((60 * Math.PI * 0.09398) / 8.14);
      // Steering conversion factor = 1 / (steer ratio * 360)
      // Steering gear ratio = 12.8:1
      public static final double STEERING_GEAR_RATIO = 12.8;
      public static final double STEERING_CONVERSION_FACTOR = (1/(12.8*360.0));
   }

   public static final class ArmConstants {
      public static final int CANID_SHOOTER_SPARKMAX_1 = 14;
      public static final int CANID_SHOOTER_SPARKMAX_2 = 15;
      public static final int CANID_INTAKE_VICTOR = 16;
   }

   public static class OperatorConstants {

      // Joystick Deadband
      public static final double LEFT_X_DEADBAND = 0.01;
      public static final double LEFT_Y_DEADBAND = 0.01;
      public static final int JOYSTICK_INPUT_POWER_SCALE = 2;

      public static final int USB_PORT_XBOX_DRIVER = 0;
      public static final int USB_PORT_XBOX_OPERATOR = 1;

   }
}
