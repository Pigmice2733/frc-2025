// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

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
    public static final int GRABBER = 17;
    public static final int SHOOTER_FLYWHEELS_LEFT = 19;
    public static final int SHOOTER_FLYWHEELS_RIGHT = 20;
    public static final int SHOOTER_PIVOT = 22;
    public static final int INDEXER = 21;
  }

  public static class SensorConfig {
    public static final int ELEVATOR_LIMIT_SWITCH_CHANNEL = 0;
    public static final int CORAL_BEAM_BREAK_CHANNEL = 1;
    public static final int SHOOTER_UPPER_ALGAE_LIMIT_CHANNEL = 2;
    public static final int SHOOTER_LOWER_ALGAE_LIMIT_CHANNEL = 3;
  }

  public static class DrivetrainConfig {
    public static final double MAX_DRIVE_SPEED = 10.0; // m/s
    public static final double MAX_TURN_SPEED = 10.0; // rad/s
    public static final double SLOWMODE_FACTOR = 0.1;

    public static final PIDConstants DRIVE_PID = new PIDConstants(4.0, 0, 1.3);
    public static final double DRIVE_POSITION_TOLERANCE = 0.02; // m
    public static final double DRIVE_VELOCITY_TOLERANCE = 0.02; // m/s

    public static final PIDConstants TURN_PID = new PIDConstants(2.5, 0, 0.0);
    public static final double TURN_POSITION_TOLERANCE = 2; // deg
    public static final double TURN_VELOCITY_TOLERANCE = 0.01; // deg/s
  }

  public static class ElevatorConfig {
    public static final PIDController ELEVATOR_PID = new PIDController(0.144, 0.0, 0.009);
    public static final double ELEVATOR_P_UP = 0.57;
    public static final double ELEVATOR_P_DOWN = 0.144;
    public static final double ELEVATOR_TOLERANCE = 0.2; // in.
    public static final double ELEVATOR_VELOCITY_TOLERANCE = 0.1; // in./s
    public static final double ELEVATOR_KG = 0.0;

    public static final double MAX_ELEVATOR_DELTA = 2;
    public static final double ELEVATOR_CONVERSION = 0.46; // inches per rotation
    public static final double ELEVATOR_UPPER_LIMIT = 21.4; // in.
  }

  public static class ArmConfig {
    public static final PIDController PIVOT_PID = new PIDController(0.01, 0, 0);
    public static final double PIVOT_KG = 0.033;

    public static final double PIVOT_TOLERANCE = 4; // deg
    public static final double MAX_PIVOT_DELTA = 20;
    public static final double PIVOT_CONVERSION = 360; // degrees per rotation
    public static final double PIVOT_ANGLE_OFFSET = -137.18; // makes 0 down
    public static final double PIVOT_LOWER_LIMIT = 20; // deg
    public static final double PIVOT_UPPER_LIMIT = 270; // deg

    public static final double GRABBER_SPEED = 1.0;
  }

  public static class ShooterConfig {
    public static final PIDController PIVOT_PID = new PIDController(0.01, 0, 0);

    public static final double PIVOT_TOLERANCE = 2; // deg
    public static final double MAX_PIVOT_SPEED = 0.5;
    public static final double PIVOT_CONVERSION = 4.8; // deg
    public static final double PIVOT_LOWER_LIMIT = 0; // deg
    public static final double PIVOT_UPPER_LIMIT = 102; // deg

    public static final double PIVOT_START_ANGLE = -45; // deg
    public static final double PIVOT_INTAKE_ANGLE = 85; // deg
    public static final double PIVOT_STOW_ANGLE = 0; // deg
    public static final double PIVOT_NET_ANGLE = 25; // deg
    public static final double PIVOT_PROCESSOR_ANGLE = 72; // deg

    public static final PIDController FLYWHEEL_PID = new PIDController(0.002, 0.0015, 0.0);
    public static final double FLYWHEEL_TOLERANCE = 30; // rpm
    public static final double FLYWHEEL_LOW_SPEED = 500; // rpm
    public static final double FLYWHEEL_HIGH_SPEED = 800; // rpm

    public static final double INDEXER_SPEED = 0.5;
  }

  public static class ClimberConfig {
    public static final double CLIMBER_DEFAULT_POSITION = 0;
    public static final double CLIMBER_CLIMB_POSITION = 0.5;
  }

  public static class LEDConfig {
    public static final int LED_PORT = 0;
    public static final int LED_LEN = 91;

    public static final Color OFF = new Color(0, 0, 0);
    public static final Color PURPLE = new Color(180, 0, 255);
    public static final Color RED = new Color(255, 0, 0);
    public static final Color BLUE = new Color(0, 0, 255);
  }

  public static class VisionTargetIds {
    public static final int REEF = 7;
  }

  public static enum ArmPosition {
    HUMAN_PLAYER(40, 21),
    SCORE_L1(260, 0),
    SCORE_L2(250, 0),
    SCORE_L3(215, 1),
    SCORE_L4(187, 21),
    STOW(20, 0),
    CLIMB(270, 10),
    ALGAE_L3(240, 17),
    ALGAE_L2(240, 1.5);

    private double pivotAngle, elevatorHeight;

    /**
     * Elevator and pivot setpoints.
     * 
     * @param pivot    target angle in degrees
     * @param elevator target height in inches
     */
    private ArmPosition(double pivot, double elevator) {
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
