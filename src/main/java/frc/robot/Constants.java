// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

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

  public static final int PDH_ID = 22;

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.01;
    public static final double LEFT_Y_DEADBAND  = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class IntakeConstants {
    public static final int TOP_MOTOR_ID = 14;
    public static final int BOTTOM_MOTOR_ID = 13;
    public static final int LEFT_MOTOR_ID = 20;
    public static final int RIGHT_MOTOR_ID = 21;

    public static final double TOP_GEAR_RATIO = 2.89;
    public static final double BOTTOM_GEAR_RATIO = 1.33;

    public static final double VELOCITY_CONVERSION_TOP = (1/60.0) * (1/TOP_GEAR_RATIO) * (4 * Math.PI) * (1/39.3701);
    public static final double VELOCITY_CONVERSION_BOTTOM = (1/60.0) * (1/BOTTOM_GEAR_RATIO) * (2 * Math.PI) * (1/39.3701);

    public static final double UPPER_FF = 0.0001704174943016842;
    public static final double LOWER_FF = 0.0001934174943016842;

    public static final double SHOOTING_SPEED = 1.75;

    public static final double MINIMUM_DRIVETRAIN_INTAKE_SPEED_METERS_PER_SECOND = 1.5;

    public static final int NOTE_BEAM_BREAK_CHANNEL = 0;

    public static final double NOTE_BEAM_BREAK_VOLTAGE_THRESHOLD = 1.0;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 16;
    public static final int SHOOTER_MOTOR_ID2 = 19;
    public static final int INDEX_MOTOR_ID = 15;
    public static final double INDEX_RAMP_RATE = 0.25;
    public static final double SHOOTER_GEARBOX_RATIO = 1.33;
    public static final double SHOOTER_SETPOINT_TOLERANCE = 400.0;
    
    public static final double SHOOTER_P = 0.00005;
    public static final double SHOOTER_D = 0.005;
    public static final double SHOOTER_FF = 0.00022101750073488802;

    public static final int SPEAKER_SHOOTING_SPEED_RPM = 6000;
    public static final int AUTO_SPEAKER_PRE_SPOOL_SPEED_RPM = 6000;
    public static final int TELEOP_SPEAKER_PRE_SPOOL_SPEED_RPM = 3000;
    public static final int AMP_SHOOTING_SPEED_RPM = 800;

    public static final double PRE_SPOOL_DISTANCE_METERS = 3.0;
  }

  public static class WhipperConstants {
    public static final int WHIPPER_MOTOR_ID = 17;
    public static final int LINEAR_ACTUATOR_LEFT = 0;
    public static final int LINEAR_ACTUATOR_RIGHT = 1;
  }
  
  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 18;
    public static final double CLIMBER_MOTOR_CONVERSION = (1/48.0) * (1 * Math.PI);
    public static final float CLIMBER_HEIGHT_LIMIT = 18.0f;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
  }

  public static class FieldConstants {
    public static final Translation2d SPEAKER_POSE_BLUE = new Translation2d(0, 5.55);
    public static final Translation2d SPEAKER_POSE_RED = new Translation2d(16.52, 5.55);
  }

  public static class LEDConstants {
    public static final int LED_PORT = 2;
    public static final int TOTAL_LEDS = 34;
    public static final int MID_POINT_START_LED_INDEX = 13;
  }
}
