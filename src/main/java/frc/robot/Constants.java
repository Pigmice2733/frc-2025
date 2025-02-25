// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  public static final double AXIS_THRESHOLD = 0.1;

  public static class CANConfig {
    public static final int ELEVATOR_LEFT = 14;
    public static final int ELEVATOR_RIGHT = 15;
    public static final int PIVOT = 16;
    public static final int ALGAE_GRABBER = 17;
    public static final int CORAL_GRABBER = 18;
    public static final int SHOOTER_FLYWHEELS_LEFT = 20;
    public static final int SHOOTER_FLYWHEELS_RIGHT = 21;
    public static final int SHOOTER_PIVOT_LEFT = 22;
    public static final int SHOOTER_PIVOT_RIGHT = 23;
    public static final int INDEXER = 24;
  }

  public static class SensorConfig {
    public static final int ELEVATOR_LIMIT_SWITCH_CHANNEL = 0;
    public static final int CORAL_BEAM_BREAK_CHANNEL = 1;
  }

  public static class DrivetrainConfig {
    public static final double MAX_DRIVE_SPEED = 10.0; // m/s
    public static final double MAX_TURN_SPEED = 20.0; // rad/s
    public static final double SLOWMODE_FACTOR = 0.4;
    public static final PIDConstants DRIVE_PID = new PIDConstants(0.5, 0, 0);
    public static final PIDConstants TURN_PID = new PIDConstants(0.5, 0, 0);
  }

  public static class ElevatorConfig {
    public static final PIDController ELEVATOR_PID = new PIDController(0.14, 0, 0.01);
    public static final double ELEVATOR_TOLERANCE = 0.1; // in.
    public static final double MAX_ELEVATOR_SPEED = 0.5;
    public static final double ELEVATOR_CONVERSION = 0.46; // inches per rotation
    public static final double ELEVATOR_UPPER_LIMIT = 20; // in.
  }

  public static class ArmConfig {
    // TODO nothing should run for a set amount of time, stuff should generally be
    // toggle or while held. --Nathan (see also ShooterConfig)
    public static final PIDController PIVOT_PID = new PIDController(0.005, 0, 0);
    public static final double PIVOT_TOLERANCE = 2; // deg
    public static final double MAX_PIVOT_SPEED = 0.5;
    public static final double PIVOT_CONVERSION = 360; // degrees per rotation
    public static final double PIVOT_ANGLE_OFFSET = -137.18; // makes 0 down
    public static final double PIVOT_LOWER_LIMIT = 9.5; // deg
    public static final double PIVOT_UPPER_LIMIT = 270; // deg

    public static final double CORAL_INTAKE_SPEED = 1.0;
    public static final double CORAL_OUTTAKE_SPEED = -1.0;
    public static final double CORAL_INTAKE_TIME = 1.0;
    public static final double CORAL_OUTTAKE_TIME = 1.0;

    public static final double GRABBER_SPEED = 1.0;
    public static final double GRABBER_TIME = 7.0;
  }

  public static class ShooterConfig {
    public static final PIDController PIVOT_PID = new PIDController(0.1, 0, 0);
    public static final double PIVOT_TOLERANCE = 2; // deg
    public static final double MAX_PIVOT_SPEED = 0.5;
    public static final double PIVOT_CONVERSION = 1;
    public static final double PIVOT_LOWER_LIMIT = -12; // deg
    public static final double PIVOT_UPPER_LIMIT = 90; // deg

    public static final double FLYWHEEL_PROCESSOR_SPEED = 0.2;
    public static final double FLYWHEEL_NET_SPEED = 0.8;
    public static final double INDEXER_SPEED = 0.5;

    public static final double SPINUP_TIME = 2.0;
    public static final double SHOOT_TIME = 2.0;
    public static final double INTAKE_TIME = 0.5;
  }

  public static class ClimberConfig {
    public static final double CLIMBER_DEFAULT_POSITION = 0;
    public static final double CLIMBER_CLIMB_POSITION = 0.5;
  }

  public static enum ElevatorPosition {
    HUMAN_PLAYER(120, 0), SCORE_L1(260, 0), SCORE_L2(225, 0.15), SCORE_L3(225, 0.35), SCORE_L4(200, 20),
    REEF_L2(210, 0.2), REEF_L3(10, 12), STOW(10, 0), CLIMB(10, 0.1), ALGAE_L3(250, 21), ALGAE_L2(250, 0);

    private double pivotAngle, elevatorHeight;

    private ElevatorPosition(double pivot, double elevator) {
      pivotAngle = pivot;
      elevatorHeight = elevator;
    }

    public double getPivotAngle() {
      return pivotAngle;
    }

    public double getElevatorHeight() {
      return elevatorHeight;
    }
  }

  public static enum ShooterPosition {
    INTAKE(-8), PROCESSOR(0), NET(40), STOW(90);

    private double angle;

    private ShooterPosition(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }

  public static enum OperatorMode {
    NONE,
    REEF,
    ELEVATOR,
    SHOOTER
  }

  public static void sendNumberToElastic(String name, double num, double places) {
    double newNum = Math.round(num * Math.pow(10, places)) / Math.pow(10, places);
    SmartDashboard.putNumber(name, newNum);
  }

  public static void sendBooleanToElastic(String name, boolean val) {
    SmartDashboard.putBoolean(name, val);
  }
}
