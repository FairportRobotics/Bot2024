// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kSystemCheckControllerPort = 4;
  }

  public static class AdvantageKitConstants {
    public enum RobotType {
      SIM,
      REAL,
      REPLAY
    }

    public static final RobotType CURRENT_MODE = RobotType.REAL;
  }


  public static class ShooterContants{
    public static final int SHOOTER_TOP_MOTOR_ID = 58;
    public static final int SHOOTER_BOTTOM_MOTOR_ID = 4;
  }

  public static class IntakeConstants {
    public static final int INTAKE_LEFT_MOTOR_ID = 14;
    public static final int INTAKE_RIGHT_MOTOR_ID = 23;

    public static final int FEEDER_MOTOR_ID = 3;
    public static final int FEEDER_SENSOR_ID = 9;

    public static final int SHOOTER_SENSOR_ID = 7;
  }

  public static class ClimberConstants{
    public static final int CLIMBER_LEFT_MOTOR_ID = 5;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 21;
  }

  public static class ElevatorConstants{
    public static final int ELEVATOR_LEFT_MOTOR_ID = 20;
    public static final int ELEVATOR_RIGHT_MOTOR_ID = 50;

    public static final int ELEVATOR_BOTTOM_SWITCH_ID = 8;
  }

}
