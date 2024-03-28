// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

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
   public static final double ROBOT_MASS = (125.0) * 0.453592; // 32lbs * kg per pound
   public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
   public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag


   public static final class Auton {

      //public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
      //public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
      //public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

      public static final double AUTO_SHOOT_DURATION = 3.0;
      public static final double AUTO_INTAKE_MAX_DURATION = 5.0;
      
      public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
      public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);

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
      public static final double STEERING_GEAR_RATIO_OLD = 12.8;
      public static final double STEERING_GEAR_RATIO = (150.0/7.0); // FROM SDS
      //public static final double STEERING_CONVERSION_FACTOR = (1/(12.8*360.0));
   }

   public static final class ArmConstants {
      public static final int CANID_SHOOTER_SPARKMAX_1 = 14;
      public static final int CANID_SHOOTER_SPARKMAX_2 = 15;
      public static final int CANID_INTAKE_VICTOR = 16;
      public static final int CANID_ARM_TALON_1 = 17;
      public static final int CANID_ARM_VICTOR_2 = 18;

      public static final int CANID_INTAKE_LASER_1 = 21; // laser at nose of intake
      public static final int CANID_INTAKE_LASER_2 = 22; // laser under the middle wheel of intake

      public static final double INTAKE_MOTOR_SPEED_MAX = 0.8;
      public static final double INTAKE_MOTOR_SPEED = 0.8; // was 0.75 with the 775Pro
      public static final double INTAKE_MOTOR_SPEED_PAST_LASER1 = 0.55;
      public static final double INTAKE_MOTOR_SPEED_SLOWER = 0.7;
      public static final double INTAKE_MOTOR_SHOOT_SPEED = 1.0; // vibe based engineering
      public static final double INTAKE_BACK_SLOW_SPEED = -0.35;
      public static final double INTAKE_BACKUP_TIME = 0.1;

      public static final double LASER_BEAM_BREAK_THRESHOLD = 13.1; // If laser reading is < this value, we have a note inside
   
      public static final double ARM_ENCODER_TICS_PER_DEG = 360.0/4096.0;
      public static final double ARM_MAX_OUTPUT_POWER = 0.6;
      /*
      Shooter PID values from 2020 robot but these seem like junk
       #define SHOOTER_PID_P 0.000040
         #define SHOOTER_PID_I 0.0000004
         #define SHOOTER_PID_D 0.000000
         #define SHOOTER_PID_F 0.000005
       */
      // TODO: TUNE //P0085 and f00009 
      public static final double SHOOTER_PID_P = 0.00000;
      public static final double SHOOTER_PID_F = 0.00018
      ;
      public static final double SHOOTER_PID_I = 0.0;
      public static final double SHOOTER_PID_D = 0.0;
  
      public static final double SHOOTER_MAX_NATIVE_VELOCITY = 21650.0;
      public static final double SHOOTER_VELOCITY_PER_RPM = 3.3934; // asuming 21650/6380
      public static final double SHOOTER_VELOCITY_RANGE = 500.0;
      public static final double SHOOTER_TOO_FAST_FOR_INTAKE_SPEED = 2000.0;
      

      public static final double ARM_SOFT_LIMIT_LOWER = 2700;
      public static final double ARM_SOFT_LIMIT_UPPER = 3300;  // old was 3870 before moving up

      public static final double ARM_SOFT_LIMIT_TEST_CLOSE_LOWER = 2700;
      public static final double ARM_SOFT_LIMIT_TEST_CLOSE_UPPER = 3300;

      public static final double ARM_DOWN_ENCODER_VALUE = 2380;
      public static final double ARM_CLOSE_SHOT_SETPOINT = 2920; // was 2837
      public static final double ARM_LONG_SHOT_SETPOINT = 3125;
      public static final double ARM_AMP_SHOT_SETPOINT = 3050;

      public static final double ARM_CLOSE_SHOT_ANGLE_OFFSET = 55;
      //2815-2860 close shot
      // 3010 - 3050 long shot

      public static final double ARM_PEAK_UP_POWER = 0.5;
      public static final double ARM_PEAK_DOWN_POWER = -0.25;

      // Might not need these, use arbitrary FF term instead
      public static final double ARM_KG = 0.59;
      public static final double ARM_KV = 3.29;
      public static final double ARM_KA = 0.05;

      public static final double ARM_MAX_PID_ERROR = 1.0;
      public static final double ARM_MAX_PID_POWER = 0.6;

      public static final double ARM_PID_P = 1.5;
      public static final double ARM_PID_I = 0.0;
      public static final double ARM_PID_D = 0.5; // was 4.0 
      public static final double ARM_PID_F = 0.0;
      
      public static final double ARM_ARBITRARY_FF_MAX = 0.1;

   }

   public static final class ClimberConstants
   {
      public static final int CANID_CLIMBER_VICTOR_L = 24;
      public static final int CANID_CLIMBER_VICTOR_R = 25;

      public static final int CHANNEL_CLIMBER_RELAY_L = 0;
      public static final int CHANNEL_CLIMBER_RELAY_R = 1;

      public static final int CHANNEL_CLIMBER_DIGITALINPUT_L = 0;
      public static final int CHANNEL_CLIMBER_DIGITALINPUT_R = 1;

      public static final double CLIMBER_UP_SPEED = -0.4;

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
